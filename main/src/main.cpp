#include "audio_input.h"

#include "keyboard_input.h"
#include "lvgl/lvgl.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <signal.h>
#include <unistd.h>

#if LV_USE_SDL
#include "lvgl/src/drivers/sdl/lv_sdl_window.h"
#endif

#if LV_USE_EVDEV
#include <linux/input.h>
#include <pthread.h>
#endif

namespace {

constexpr int kScreenWidth = 320;
constexpr int kScreenHeight = 170;
constexpr int kTopHeight = 28;
constexpr int kBarBaseY = 145;
constexpr int kBarMaxHeight = 105;
constexpr int kBarX = 8;
constexpr int kBarAreaWidth = 304;
constexpr int kBarGap = 2;

lv_indev_t *g_keyboard_indev = nullptr;
lv_group_t *g_key_group = nullptr;
lv_obj_t *g_root = nullptr;
lv_obj_t *g_mode_label = nullptr;
lv_obj_t *g_effect_label = nullptr;
lv_obj_t *g_source_label = nullptr;
lv_obj_t *g_status_label = nullptr;
lv_obj_t *g_meter_label = nullptr;
lv_obj_t *g_bars[SPECTRUM_BAND_COUNT] = {};
lv_timer_t *g_ui_timer = nullptr;
volatile sig_atomic_t g_quit_requested = 0;
uint32_t g_escape_first_tick = 0;
uint32_t g_escape_last_tick = 0;
int g_escape_repeat_count = 0;
uint32_t g_status_hold_until = 0;
float g_peak_levels[SPECTRUM_BAND_COUNT] = {};
uint32_t g_visual_tick = 0;

enum class VisualEffect {
    Bars = 0,
    Mirror,
    Halo,
    Pulse,
    Starburst,
    Aurora,
    Tunnel,
    Count,
};

VisualEffect g_effect = VisualEffect::Bars;

const char *getenv_default(const char *name, const char *fallback)
{
    const char *value = std::getenv(name);
    return value ? value : fallback;
}

void request_quit()
{
    g_quit_requested = 1;
}

void handle_signal(int)
{
    request_quit();
}

int get_st7789v_fbdev(char *dev_path, size_t buf_size)
{
    if (!dev_path || buf_size == 0) return -1;
    FILE *fp = std::fopen("/proc/fb", "r");
    if (!fp) return -1;

    char line[256];
    int fb_num = -1;
    while (std::fgets(line, sizeof(line), fp)) {
        if (std::strstr(line, "fb_st7789v") &&
            std::sscanf(line, "%d", &fb_num) == 1) {
            break;
        }
    }
    std::fclose(fp);
    if (fb_num < 0) return -1;
    std::snprintf(dev_path, buf_size, "/dev/fb%d", fb_num);
    return 0;
}

#if LV_USE_EVDEV
int evdev_to_lv_key(uint16_t code);
void keypad_read_cb(lv_indev_t *, lv_indev_data_t *data);
void lv_linux_indev_init();
#endif

#if LV_USE_LINUX_FBDEV
void lv_linux_disp_init()
{
    char fbdev[64] = {};
    const char *device = getenv_default("LV_LINUX_FBDEV_DEVICE", nullptr);
    if (!device && get_st7789v_fbdev(fbdev, sizeof(fbdev)) == 0) {
        device = fbdev;
    }
    if (!device) device = "/dev/fb0";

    std::printf("Using framebuffer device: %s\n", device);
    lv_display_t *disp = lv_linux_fbdev_create();
    if (!disp) {
        std::printf("Failed to create fbdev display\n");
        return;
    }
    lv_linux_fbdev_set_file(disp, device);
}

#if !LV_USE_EVDEV && !LV_USE_LIBINPUT
void lv_linux_indev_init() {}
#endif

#elif LV_USE_SDL
void lv_linux_disp_init()
{
    const int width = std::atoi(getenv_default("LV_SDL_VIDEO_WIDTH", "320"));
    const int height = std::atoi(getenv_default("LV_SDL_VIDEO_HEIGHT", "170"));
    lv_display_t *disp = lv_sdl_window_create(width, height);
    lv_sdl_window_set_title(disp, "Audio Spectrum");
}

void lv_linux_indev_init()
{
    lv_sdl_mouse_create();
    g_keyboard_indev = lv_sdl_keyboard_create();
}
#else
#error Unsupported display configuration
#endif

lv_color_t band_color(int index, float level)
{
    float t = static_cast<float>(index) / static_cast<float>(SPECTRUM_BAND_COUNT - 1);
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    if (t < 0.36f) {
        float k = t / 0.36f;
        r = static_cast<uint8_t>(28 + 50 * k);
        g = static_cast<uint8_t>(178 + 55 * k);
        b = static_cast<uint8_t>(180 - 80 * k);
    } else if (t < 0.72f) {
        float k = (t - 0.36f) / 0.36f;
        r = static_cast<uint8_t>(74 + 170 * k);
        g = static_cast<uint8_t>(214 - 28 * k);
        b = static_cast<uint8_t>(92 - 48 * k);
    } else {
        float k = (t - 0.72f) / 0.28f;
        r = static_cast<uint8_t>(244 - 4 * k);
        g = static_cast<uint8_t>(176 - 88 * k);
        b = static_cast<uint8_t>(44 + 164 * k);
    }
    int lift = static_cast<int>(level * 34.0f);
    r = static_cast<uint8_t>(std::min(255, r + lift));
    g = static_cast<uint8_t>(std::min(255, g + lift / 2));
    b = static_cast<uint8_t>(std::min(255, b + lift));
    return lv_color_make(r, g, b);
}

int clampi(int value, int lo, int hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

lv_opa_t opa_from_level(int base, float level, float scale)
{
    return static_cast<lv_opa_t>(clampi(base + static_cast<int>(level * scale), 0, 255));
}

float band_average(const SpectrumSnapshot &snapshot, int first, int last)
{
    first = clampi(first, 0, SPECTRUM_BAND_COUNT - 1);
    last = clampi(last, first + 1, SPECTRUM_BAND_COUNT);
    float sum = 0.0f;
    for (int i = first; i < last; ++i) {
        sum += snapshot.bands[i];
    }
    return sum / static_cast<float>(last - first);
}

void style_fill(lv_obj_t *obj, lv_color_t color, lv_opa_t opa, int radius)
{
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_border_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(obj, color, 0);
    lv_obj_set_style_bg_opa(obj, opa, 0);
    lv_obj_set_style_radius(obj, radius, 0);
}

void style_outline(lv_obj_t *obj, lv_color_t color, lv_opa_t opa, int width, int radius)
{
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, width, 0);
    lv_obj_set_style_border_color(obj, color, 0);
    lv_obj_set_style_border_opa(obj, opa, 0);
    lv_obj_set_style_radius(obj, radius, 0);
}

const char *effect_name(VisualEffect effect)
{
    switch (effect) {
    case VisualEffect::Mirror:
        return "MIRROR";
    case VisualEffect::Halo:
        return "HALO";
    case VisualEffect::Pulse:
        return "PULSE";
    case VisualEffect::Starburst:
        return "STAR";
    case VisualEffect::Aurora:
        return "AURORA";
    case VisualEffect::Tunnel:
        return "TUNNEL";
    case VisualEffect::Bars:
    default:
        return "BARS";
    }
}

void next_effect()
{
    int next = static_cast<int>(g_effect) + 1;
    if (next >= static_cast<int>(VisualEffect::Count)) next = 0;
    g_effect = static_cast<VisualEffect>(next);
    std::fill(std::begin(g_peak_levels), std::end(g_peak_levels), 0.0f);
    if (g_status_label) {
        char status[64];
        std::snprintf(status, sizeof(status), "FX %s", effect_name(g_effect));
        lv_label_set_text(g_status_label, status);
    }
}

void hold_status(const char *text, uint32_t duration_ms = 1200)
{
    if (!g_status_label) return;
    lv_label_set_text(g_status_label, text);
    g_status_hold_until = lv_tick_get() + duration_ms;
}

void handle_app_key(uint32_t key, char ch)
{
    if (ch) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (key == ' ' || ch == ' ') {
        g_escape_repeat_count = 0;
        spectrum_audio_toggle_mode();
    } else if (key == 'm' || ch == 'm') {
        g_escape_repeat_count = 0;
        spectrum_audio_toggle_mic_source();
        char status[64];
        std::snprintf(status, sizeof(status), "MIC source %s", spectrum_audio_mic_source_name());
        hold_status(status);
        if (g_source_label) lv_label_set_text(g_source_label, status);
    } else if (key == 'c' || ch == 'c') {
        g_escape_repeat_count = 0;
        spectrum_audio_start_calibration();
        hold_status("Calibrating quiet 2s", 2200);
    } else if (key == '+' || key == '=' || ch == '+' || ch == '=') {
        g_escape_repeat_count = 0;
        spectrum_audio_adjust_sensitivity(1.25f);
        char status[96];
        spectrum_audio_format_settings(status, sizeof(status));
        hold_status(status, 1400);
    } else if (key == '-' || key == '_' || ch == '-' || ch == '_') {
        g_escape_repeat_count = 0;
        spectrum_audio_adjust_sensitivity(0.80f);
        char status[96];
        spectrum_audio_format_settings(status, sizeof(status));
        hold_status(status, 1400);
    } else if (key == LV_KEY_ENTER || key == LV_KEY_RIGHT || key == 'e' || ch == 'e') {
        g_escape_repeat_count = 0;
        next_effect();
    } else if (key == LV_KEY_ESC || key == LV_KEY_HOME) {
        uint32_t now = lv_tick_get();
        if (g_escape_repeat_count == 0 || lv_tick_elaps(g_escape_last_tick) > 220) {
            g_escape_first_tick = now;
            g_escape_repeat_count = 1;
            if (g_status_label) lv_label_set_text(g_status_label, "Hold Esc to exit");
        } else {
            ++g_escape_repeat_count;
            if (lv_tick_elaps(g_escape_first_tick) >= 650 || g_escape_repeat_count >= 8) {
                request_quit();
            }
        }
        g_escape_last_tick = now;
    }
}

void root_key_event(lv_event_t *event)
{
    uint32_t key = lv_event_get_key(event);
    char ch = key >= 32 && key < 127 ? static_cast<char>(key) : 0;
    handle_app_key(key, ch);
}

void keyboard_queue_event(lv_event_t *event)
{
    auto *key = static_cast<key_item *>(lv_event_get_param(event));
    if (!key || key->key_state == KBD_KEY_RELEASED) return;

    char ch = 0;
    if (key->utf8[0] && !key->utf8[1]) {
        ch = key->utf8[0];
    }

    uint32_t lv_key = key->key_code;
    switch (key->key_code) {
    case KEY_UP:
        lv_key = LV_KEY_UP;
        break;
    case KEY_DOWN:
        lv_key = LV_KEY_DOWN;
        break;
    case KEY_RIGHT:
        lv_key = LV_KEY_RIGHT;
        break;
    case KEY_LEFT:
        lv_key = LV_KEY_LEFT;
        break;
    case KEY_ENTER:
        lv_key = LV_KEY_ENTER;
        break;
    case KEY_ESC:
        lv_key = LV_KEY_ESC;
        break;
    case KEY_HOME:
        lv_key = LV_KEY_HOME;
        break;
    default:
        break;
    }
    handle_app_key(lv_key, ch);
}

void create_label(lv_obj_t **label, lv_obj_t *parent, const char *text,
                  int x, int y, int w, const lv_font_t *font, uint32_t color)
{
    *label = lv_label_create(parent);
    lv_obj_set_pos(*label, x, y);
    lv_obj_set_size(*label, w, font == &lv_font_montserrat_10 ? 11 : 15);
    lv_obj_set_style_text_font(*label, font, 0);
    lv_obj_set_style_text_color(*label, lv_color_hex(color), 0);
    lv_label_set_long_mode(*label, LV_LABEL_LONG_CLIP);
    lv_label_set_text(*label, text);
}

void build_ui()
{
    g_root = lv_screen_active();
    lv_obj_set_size(g_root, kScreenWidth, kScreenHeight);
    lv_obj_clear_flag(g_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(g_root, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_set_style_bg_color(g_root, lv_color_hex(0x080A0D), 0);
    lv_obj_set_style_bg_opa(g_root, LV_OPA_COVER, 0);
#if LV_USE_SDL
    lv_obj_add_event_cb(g_root, root_key_event, LV_EVENT_KEY, nullptr);
#endif
    lv_obj_add_event_cb(g_root, keyboard_queue_event, static_cast<lv_event_code_t>(LV_EVENT_KEYBOARD), nullptr);

    lv_obj_t *top = lv_obj_create(g_root);
    lv_obj_remove_style_all(top);
    lv_obj_set_pos(top, 0, 0);
    lv_obj_set_size(top, kScreenWidth, kTopHeight);
    lv_obj_set_style_bg_color(top, lv_color_hex(0x121820), 0);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = nullptr;
    create_label(&title, top, "SPECTRUM", 8, 6, 80, &lv_font_montserrat_14, 0xF4F7FA);
    create_label(&g_mode_label, top, "MIC", 92, 6, 48, &lv_font_montserrat_14, 0x2DE2E6);
    create_label(&g_effect_label, top, "BARS", 144, 6, 66, &lv_font_montserrat_12, 0xB7F34A);
    create_label(&g_meter_label, top, "LVL 0%", 214, 6, 76, &lv_font_montserrat_12, 0xD5E1EA);

    lv_obj_t *base_line = lv_obj_create(g_root);
    lv_obj_remove_style_all(base_line);
    lv_obj_set_pos(base_line, 6, kBarBaseY + 2);
    lv_obj_set_size(base_line, 308, 2);
    lv_obj_set_style_bg_color(base_line, lv_color_hex(0x27313B), 0);
    lv_obj_set_style_bg_opa(base_line, LV_OPA_COVER, 0);

    const int slot = kBarAreaWidth / SPECTRUM_BAND_COUNT;
    const int bar_w = std::max(3, slot - kBarGap);
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        g_bars[i] = lv_obj_create(g_root);
        lv_obj_remove_style_all(g_bars[i]);
        lv_obj_set_pos(g_bars[i], kBarX + i * slot, kBarBaseY - 2);
        lv_obj_set_size(g_bars[i], bar_w, 2);
        lv_obj_set_style_bg_color(g_bars[i], band_color(i, 0.0f), 0);
        lv_obj_set_style_bg_opa(g_bars[i], LV_OPA_COVER, 0);
        lv_obj_set_style_radius(g_bars[i], 1, 0);
    }

    create_label(&g_source_label, g_root, "Starting audio input", 8, 148, 304, &lv_font_montserrat_10, 0xB6C3CF);
    create_label(&g_status_label, g_root, "C cal  +/- sens  Enter FX  M mic", 8, 158, 304, &lv_font_montserrat_10, 0x7E8B96);

    g_key_group = lv_group_create();
    lv_group_add_obj(g_key_group, g_root);
    lv_group_focus_obj(g_root);
    if (g_keyboard_indev) {
        lv_indev_set_group(g_keyboard_indev, g_key_group);
    }
}

void render_bars(const SpectrumSnapshot &snapshot)
{
    const int slot = kBarAreaWidth / SPECTRUM_BAND_COUNT;
    const int bar_w = std::max(3, slot - kBarGap);
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[i];
        g_peak_levels[i] = std::max(level, g_peak_levels[i] * 0.965f);
        int h = 3 + static_cast<int>(std::pow(level, 0.72f) * kBarMaxHeight);
        if (h > kBarMaxHeight) h = kBarMaxHeight;
        int peak_lift = static_cast<int>(g_peak_levels[i] * 10.0f);
        lv_obj_set_pos(g_bars[i], kBarX + i * slot, kBarBaseY - h - peak_lift);
        lv_obj_set_size(g_bars[i], bar_w, h + peak_lift);
        style_fill(g_bars[i], band_color(i, level), LV_OPA_COVER, 1);
    }
}

void render_mirror(const SpectrumSnapshot &snapshot)
{
    const int slot = kBarAreaWidth / SPECTRUM_BAND_COUNT;
    const int bar_w = std::max(3, slot - kBarGap);
    constexpr int center_y = 88;
    constexpr int max_half = 56;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[i];
        int h = 3 + static_cast<int>(std::pow(level, 0.66f) * max_half);
        lv_obj_set_pos(g_bars[i], kBarX + i * slot, center_y - h);
        lv_obj_set_size(g_bars[i], bar_w, h * 2);
        style_fill(g_bars[i], band_color(SPECTRUM_BAND_COUNT - 1 - i, level), LV_OPA_COVER, 2);
    }
}

void render_halo(const SpectrumSnapshot &snapshot)
{
    constexpr int cx = 160;
    constexpr int cy = 86;
    constexpr float rx = 108.0f;
    constexpr float ry = 42.0f;
    float bass = band_average(snapshot, 0, 8);
    float mid = band_average(snapshot, 8, 22);
    ++g_visual_tick;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[i];
        float angle = (static_cast<float>(i) / static_cast<float>(SPECTRUM_BAND_COUNT)) * 6.2831853f -
                      1.5707963f + static_cast<float>(g_visual_tick) * 0.006f;
        float pulse = 1.0f + bass * 0.16f + mid * 0.06f;
        int size = 4 + static_cast<int>(std::pow(level, 0.64f) * 16.0f);
        int x = cx + static_cast<int>(std::cos(angle) * rx * pulse) - size / 2;
        int y = cy + static_cast<int>(std::sin(angle) * ry * pulse) - size / 2;
        lv_obj_set_pos(g_bars[i], x, y);
        lv_obj_set_size(g_bars[i], size, size);
        style_fill(g_bars[i], band_color(i, level), opa_from_level(80, level, 175.0f), LV_RADIUS_CIRCLE);
    }
}

void render_pulse(const SpectrumSnapshot &snapshot)
{
    constexpr int cx = 160;
    constexpr int cy = 88;
    float bass = band_average(snapshot, 0, 7);
    float mid = band_average(snapshot, 7, 18);
    float treble = band_average(snapshot, 18, SPECTRUM_BAND_COUNT);
    ++g_visual_tick;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float phase = std::fmod(static_cast<float>(g_visual_tick) * 0.026f + static_cast<float>(i) * 0.092f, 1.0f);
        float level = i < 11 ? bass : (i < 22 ? mid : treble);
        float grow = phase * (64.0f + bass * 34.0f);
        int w = clampi(36 + static_cast<int>(grow) + i * 4, 16, 300);
        int h = clampi(14 + static_cast<int>(grow * 0.42f) + i * 2, 8, 116);
        int opacity = 210 - static_cast<int>(phase * 145.0f) + static_cast<int>(level * 70.0f);
        int border = i % 5 == 0 ? 2 : 1;
        lv_obj_set_pos(g_bars[i], cx - w / 2, cy - h / 2);
        lv_obj_set_size(g_bars[i], w, h);
        style_outline(g_bars[i], band_color((i * 2) % SPECTRUM_BAND_COUNT, level),
                      static_cast<lv_opa_t>(clampi(opacity, 32, 245)), border, LV_RADIUS_CIRCLE);
    }
}

void render_starburst(const SpectrumSnapshot &snapshot)
{
    constexpr int cx = 160;
    constexpr int cy = 88;
    constexpr float inner_rx = 58.0f;
    constexpr float inner_ry = 24.0f;
    ++g_visual_tick;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[i];
        float angle = (static_cast<float>(i) / static_cast<float>(SPECTRUM_BAND_COUNT)) * 6.2831853f +
                      static_cast<float>(g_visual_tick) * 0.014f;
        float lift = 9.0f + std::pow(level, 0.58f) * 48.0f;
        int w = 3 + static_cast<int>(level * 8.0f);
        int h = 5 + static_cast<int>(level * 24.0f);
        int x = cx + static_cast<int>(std::cos(angle) * (inner_rx + lift)) - w / 2;
        int y = cy + static_cast<int>(std::sin(angle) * (inner_ry + lift * 0.43f)) - h / 2;
        lv_obj_set_pos(g_bars[i], x, y);
        lv_obj_set_size(g_bars[i], w, h);
        style_fill(g_bars[i], band_color((i + 10) % SPECTRUM_BAND_COUNT, level),
                   opa_from_level(92, level, 160.0f), LV_RADIUS_CIRCLE);
    }
}

void render_aurora(const SpectrumSnapshot &snapshot)
{
    const int slot = kBarAreaWidth / SPECTRUM_BAND_COUNT;
    constexpr int center_y = 88;
    float bass = band_average(snapshot, 0, 8);
    float treble = band_average(snapshot, 20, SPECTRUM_BAND_COUNT);
    ++g_visual_tick;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[i];
        float wave_a = std::sin((static_cast<float>(i) * 0.52f) + static_cast<float>(g_visual_tick) * 0.065f);
        float wave_b = std::sin((static_cast<float>(i) * 0.21f) - static_cast<float>(g_visual_tick) * 0.038f);
        int h = 6 + static_cast<int>(std::pow(level + treble * 0.18f, 0.64f) * 44.0f);
        int y = center_y - h / 2 + static_cast<int>((wave_a * 13.0f + wave_b * 7.0f) * (0.55f + bass));
        lv_obj_set_pos(g_bars[i], kBarX + i * slot, y);
        lv_obj_set_size(g_bars[i], std::max(5, slot + 2), h);
        style_fill(g_bars[i], band_color((i + 8) % SPECTRUM_BAND_COUNT, level),
                   opa_from_level(118, level, 132.0f), 5);
    }
}

void render_tunnel(const SpectrumSnapshot &snapshot)
{
    constexpr int cx = 160;
    constexpr int cy = 88;
    float bass = band_average(snapshot, 0, 8);
    float mid = band_average(snapshot, 8, 20);
    ++g_visual_tick;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float level = snapshot.bands[(i * 3) % SPECTRUM_BAND_COUNT];
        if (i < 21) {
            float depth = static_cast<float>(i) / 20.0f;
            float pulse = std::fmod(depth + static_cast<float>(g_visual_tick) * 0.018f, 1.0f);
            int w = clampi(28 + static_cast<int>(pulse * 260.0f + bass * 28.0f), 18, 304);
            int h = clampi(10 + static_cast<int>(pulse * 112.0f + mid * 14.0f), 8, 124);
            int opa = 236 - static_cast<int>(pulse * 156.0f) + static_cast<int>(level * 58.0f);
            lv_obj_set_pos(g_bars[i], cx - w / 2, cy - h / 2);
            lv_obj_set_size(g_bars[i], w, h);
            style_outline(g_bars[i], band_color((i + 14) % SPECTRUM_BAND_COUNT, level),
                          static_cast<lv_opa_t>(clampi(opa, 34, 245)), pulse < 0.18f ? 2 : 1, 8);
        } else {
            float angle = static_cast<float>(i) * 1.618f + static_cast<float>(g_visual_tick) * 0.026f;
            int size = 2 + static_cast<int>(level * 9.0f);
            int x = cx + static_cast<int>(std::cos(angle) * (28.0f + bass * 56.0f)) - size / 2;
            int y = cy + static_cast<int>(std::sin(angle) * (13.0f + mid * 28.0f)) - size / 2;
            lv_obj_set_pos(g_bars[i], x, y);
            lv_obj_set_size(g_bars[i], size, size);
            style_fill(g_bars[i], band_color(i, level), opa_from_level(100, level, 140.0f), LV_RADIUS_CIRCLE);
        }
    }
}

void render_effect(const SpectrumSnapshot &snapshot)
{
    switch (g_effect) {
    case VisualEffect::Mirror:
        render_mirror(snapshot);
        break;
    case VisualEffect::Halo:
        render_halo(snapshot);
        break;
    case VisualEffect::Pulse:
        render_pulse(snapshot);
        break;
    case VisualEffect::Starburst:
        render_starburst(snapshot);
        break;
    case VisualEffect::Aurora:
        render_aurora(snapshot);
        break;
    case VisualEffect::Tunnel:
        render_tunnel(snapshot);
        break;
    case VisualEffect::Bars:
    default:
        render_bars(snapshot);
        break;
    }
}

void ui_timer_cb(lv_timer_t *)
{
    SpectrumSnapshot snapshot {};
    spectrum_audio_get_snapshot(&snapshot);

    lv_label_set_text(g_mode_label, snapshot.mode == SpectrumInputMode::Music ? "MUSIC" : "MIC");
    lv_obj_set_style_text_color(
        g_mode_label,
        snapshot.mode == SpectrumInputMode::Music ? lv_color_hex(0xFFB84D) : lv_color_hex(0x2DE2E6),
        0);
    lv_label_set_text(g_effect_label, effect_name(g_effect));

    char meter[40];
    std::snprintf(meter, sizeof(meter), "LVL %2d%%", static_cast<int>(snapshot.rms * 100.0f));
    lv_label_set_text(g_meter_label, meter);
    lv_label_set_text(g_source_label, snapshot.source);
    if (lv_tick_elaps(g_status_hold_until) < 0x7FFFFFFFU) {
        lv_label_set_text(g_status_label, snapshot.status);
    }
    lv_obj_set_style_text_color(
        g_status_label,
        snapshot.live ? lv_color_hex(0x8FA0AD) : lv_color_hex(0xFF8E5A),
        0);

    render_effect(snapshot);
}

#if LV_USE_EVDEV
int evdev_to_lv_key(uint16_t code)
{
    switch (code) {
    case KEY_UP:
        return LV_KEY_UP;
    case KEY_DOWN:
        return LV_KEY_DOWN;
    case KEY_RIGHT:
        return LV_KEY_RIGHT;
    case KEY_LEFT:
        return LV_KEY_LEFT;
    case KEY_ESC:
        return LV_KEY_ESC;
    case KEY_ENTER:
        return LV_KEY_ENTER;
    case KEY_HOME:
        return LV_KEY_HOME;
    case KEY_BACKSPACE:
        return LV_KEY_BACKSPACE;
    default:
        return code;
    }
}

void keypad_read_cb(lv_indev_t *, lv_indev_data_t *data)
{
    data->state = LV_INDEV_STATE_RELEASED;
    data->continue_reading = false;
    pthread_mutex_lock(&keyboard_mutex);
    if (!STAILQ_EMPTY(&keyboard_queue)) {
        key_item *elm = STAILQ_FIRST(&keyboard_queue);
        STAILQ_REMOVE_HEAD(&keyboard_queue, entries);
        if (g_root) {
            lv_obj_send_event(g_root, static_cast<lv_event_code_t>(LV_EVENT_KEYBOARD), elm);
        }
        data->key = evdev_to_lv_key(static_cast<uint16_t>(elm->key_code));
        data->state = static_cast<lv_indev_state_t>(elm->key_state);
        data->continue_reading = !STAILQ_EMPTY(&keyboard_queue);
        std::free(elm);
    }
    pthread_mutex_unlock(&keyboard_mutex);
}

void lv_linux_indev_init()
{
    const char *keyboard_device =
        getenv_default("LV_LINUX_KEYBOARD_DEVICE", "/dev/input/by-path/platform-3f804000.i2c-event");
    pthread_t thread_id;
    pthread_create(&thread_id, nullptr, keyboard_read_thread, const_cast<char *>(keyboard_device));
    pthread_detach(thread_id);
    g_keyboard_indev = lv_indev_create();
    lv_indev_set_type(g_keyboard_indev, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(g_keyboard_indev, keypad_read_cb);
}
#endif

}  // namespace

int main(void)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    lv_init();
    lv_linux_disp_init();
    LV_EVENT_KEYBOARD = lv_event_register_id();
    lv_linux_indev_init();

    build_ui();
    spectrum_audio_start();
    g_ui_timer = lv_timer_create(ui_timer_cb, 33, nullptr);

    std::printf("Entering Audio Spectrum main loop\n");
    while (!g_quit_requested) {
        uint32_t idle_time = lv_timer_handler();
        usleep((idle_time ? idle_time : 1) * 1000);
    }

    if (g_ui_timer) {
        lv_timer_delete(g_ui_timer);
        g_ui_timer = nullptr;
    }
    spectrum_audio_stop();
    return 0;
}
