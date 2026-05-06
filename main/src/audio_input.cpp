#include "audio_input.h"

#include "global_config.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(CONFIG_TINYALSA_COMPONENT_ENABLED) && defined(__linux__)
#include <tinyalsa/asoundlib.h>
#define SPECTRUM_HAS_TINYALSA 1
#else
#define SPECTRUM_HAS_TINYALSA 0
#endif

namespace {

constexpr unsigned kSampleRate = 16000;
constexpr size_t kBlockFrames = 512;
constexpr double kTwoPi = 6.28318530717958647692;

float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

bool env_enabled(const char *name, bool fallback)
{
    const char *text = std::getenv(name);
    if (!text || !text[0]) return fallback;
    return std::strcmp(text, "0") != 0 &&
           std::strcmp(text, "false") != 0 &&
           std::strcmp(text, "False") != 0 &&
           std::strcmp(text, "off") != 0 &&
           std::strcmp(text, "OFF") != 0;
}

bool read_env_uint(const char *name, unsigned int *value)
{
    const char *text = std::getenv(name);
    if (!text || !text[0]) return false;
    char *end = nullptr;
    unsigned long parsed = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0') return false;
    *value = static_cast<unsigned int>(parsed);
    return true;
}

float read_env_float(const char *name, float fallback)
{
    const char *text = std::getenv(name);
    if (!text || !text[0]) return fallback;
    char *end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed)) return fallback;
    return parsed;
}

float input_gain(SpectrumInputMode mode)
{
    float fallback = mode == SpectrumInputMode::Mic ? 96.0f : 2.0f;
    fallback = read_env_float("M5_SPECTRUM_GAIN", fallback);
    return read_env_float(mode == SpectrumInputMode::Mic ? "M5_SPECTRUM_MIC_GAIN" : "M5_SPECTRUM_MUSIC_GAIN",
                          fallback);
}

float automatic_gain(SpectrumInputMode mode, float base_gain, float raw_rms)
{
    if (mode != SpectrumInputMode::Mic) return base_gain;
    if (!env_enabled("M5_SPECTRUM_MIC_AUTO_GAIN", true)) return base_gain;

    float target = read_env_float("M5_SPECTRUM_MIC_AUTO_TARGET", 0.55f);
    target = clampf(target, 0.03f, 0.95f);
    float max_gain = read_env_float("M5_SPECTRUM_MIC_AUTO_MAX_GAIN", 131072.0f);
    max_gain = std::max(max_gain, base_gain);

    if (raw_rms <= 0.0000001f) return base_gain;
    return clampf(target / raw_rms, base_gain, max_gain);
}

const char *mode_name(SpectrumInputMode mode)
{
    return mode == SpectrumInputMode::Music ? "MUSIC" : "MIC";
}

int default_mic_source_value()
{
    const char *text = std::getenv("M5_SPECTRUM_ADC_MUX");
    if (text && (std::strcmp(text, "AMIC") == 0 || std::strcmp(text, "amic") == 0)) return 1;
    return 0;
}

const char *mic_source_name(int source)
{
    return source == 1 ? "AMIC" : "DMIC";
}

void configure_mic_source(int source)
{
    const char *name = mic_source_name(source);
    char command[768];
    std::snprintf(command, sizeof(command),
                  "amixer -q -c 1 set 'ADC MUX' %s >/dev/null 2>&1; "
                  "amixer -q -c 1 set ALC 'ALC OFF' >/dev/null 2>&1; "
                  "amixer -q -c 1 set ADCL 255 >/dev/null 2>&1; "
                  "amixer -q -c 1 set ADCR 255 >/dev/null 2>&1; "
                  "amixer -q -c 1 set 'ADCL PGA' 14 >/dev/null 2>&1; "
                  "amixer -q -c 1 set 'ADCR PGA' 14 >/dev/null 2>&1; "
                  "amixer -q -c 1 set 'ADC OSR' 255 >/dev/null 2>&1; "
                  "tinymix -D 1 set 25 %s >/dev/null 2>&1 || true",
                  name, name);
    std::system(command);
}

std::atomic<int> g_requested_mic_source {default_mic_source_value()};
std::atomic<unsigned int> g_audio_config_generation {1};

struct MicSettingsSnapshot {
    float floor = 1.0202f;
    float span = 24.0f;
    float band_scale = 110.2269f;
    float sensitivity = 8.0f;
    bool calibrating = false;
    float calibration_progress = 0.0f;
};

struct MicSettingsState {
    std::mutex mutex;
    bool loaded = false;
    float floor = 1.0202f;
    float span = 24.0f;
    float band_scale = 110.2269f;
    float sensitivity = 8.0f;
    bool calibrating = false;
    int calibration_blocks = 0;
    int calibration_target_blocks = 64;
    double detector_sum = 0.0;
    double rms_sum = 0.0;
    float detector_max = 0.0f;
    float peak_max = 0.0f;
};

MicSettingsState g_mic_settings;

const char *calibration_file_path()
{
    const char *path = std::getenv("M5_SPECTRUM_CALIBRATION_FILE");
    return path && path[0] ? path : "/usr/share/APPLaunch/config/audio-spectrum.cal";
}

void load_mic_settings_locked()
{
    if (g_mic_settings.loaded) return;
    g_mic_settings.floor = read_env_float("M5_SPECTRUM_MIC_NOISE_FLOOR", 1.0202f);
    g_mic_settings.span = read_env_float("M5_SPECTRUM_MIC_SPEECH_SPAN", 24.0f);
    g_mic_settings.band_scale = read_env_float("M5_SPECTRUM_MIC_BAND_SCALE", 110.2269f);
    g_mic_settings.sensitivity = read_env_float("M5_SPECTRUM_MIC_SENSITIVITY", 8.0f);

    FILE *fp = std::fopen(calibration_file_path(), "r");
    if (fp) {
        char line[128];
        while (std::fgets(line, sizeof(line), fp)) {
            char key[64] = {};
            float value = 0.0f;
            if (std::sscanf(line, "%63[^=]=%f", key, &value) != 2) continue;
            if (std::strcmp(key, "floor") == 0) g_mic_settings.floor = value;
            else if (std::strcmp(key, "span") == 0) g_mic_settings.span = value;
            else if (std::strcmp(key, "band_scale") == 0) g_mic_settings.band_scale = value;
            else if (std::strcmp(key, "sensitivity") == 0) g_mic_settings.sensitivity = value;
        }
        std::fclose(fp);
    }

    g_mic_settings.floor = clampf(g_mic_settings.floor, 0.0f, 10000.0f);
    g_mic_settings.span = clampf(g_mic_settings.span, 8.0f, 10000.0f);
    g_mic_settings.band_scale = clampf(g_mic_settings.band_scale, 2.0f, 240.0f);
    g_mic_settings.sensitivity = clampf(g_mic_settings.sensitivity, 0.10f, 10.0f);
    g_mic_settings.loaded = true;
}

void save_mic_settings_locked()
{
    std::system("mkdir -p /usr/share/APPLaunch/config >/dev/null 2>&1 || true");
    FILE *fp = std::fopen(calibration_file_path(), "w");
    if (!fp) return;
    std::fprintf(fp, "floor=%.4f\n", g_mic_settings.floor);
    std::fprintf(fp, "span=%.4f\n", g_mic_settings.span);
    std::fprintf(fp, "band_scale=%.4f\n", g_mic_settings.band_scale);
    std::fprintf(fp, "sensitivity=%.4f\n", g_mic_settings.sensitivity);
    std::fclose(fp);
}

MicSettingsSnapshot current_mic_settings()
{
    std::lock_guard<std::mutex> lock(g_mic_settings.mutex);
    load_mic_settings_locked();
    MicSettingsSnapshot settings;
    settings.floor = g_mic_settings.floor;
    settings.span = g_mic_settings.span;
    settings.band_scale = g_mic_settings.band_scale;
    settings.sensitivity = g_mic_settings.sensitivity;
    settings.calibrating = g_mic_settings.calibrating;
    settings.calibration_progress = g_mic_settings.calibrating
                                        ? clampf(static_cast<float>(g_mic_settings.calibration_blocks) /
                                                     static_cast<float>(g_mic_settings.calibration_target_blocks),
                                                 0.0f, 1.0f)
                                        : 0.0f;
    return settings;
}

void update_calibration(float raw_rms_counts, float raw_peak_counts)
{
    std::lock_guard<std::mutex> lock(g_mic_settings.mutex);
    load_mic_settings_locked();
    if (!g_mic_settings.calibrating) return;

    float detector = std::max(raw_rms_counts, raw_peak_counts * 0.08f);
    g_mic_settings.detector_sum += detector;
    g_mic_settings.rms_sum += raw_rms_counts;
    g_mic_settings.detector_max = std::max(g_mic_settings.detector_max, detector);
    g_mic_settings.peak_max = std::max(g_mic_settings.peak_max, raw_peak_counts);
    ++g_mic_settings.calibration_blocks;

    if (g_mic_settings.calibration_blocks < g_mic_settings.calibration_target_blocks) return;

    float avg_detector = static_cast<float>(g_mic_settings.detector_sum /
                                            std::max(1, g_mic_settings.calibration_blocks));
    float avg_rms = static_cast<float>(g_mic_settings.rms_sum /
                                       std::max(1, g_mic_settings.calibration_blocks));
    float floor = std::max(0.2f, avg_detector * 1.35f + 0.35f);
    float span = std::max({24.0f, floor * 16.0f, g_mic_settings.detector_max * 12.0f, avg_rms * 24.0f});
    float band_scale = clampf(540.0f / std::sqrt(span), 24.0f, 126.0f);

    g_mic_settings.floor = floor;
    g_mic_settings.span = span;
    g_mic_settings.band_scale = band_scale;
    g_mic_settings.sensitivity = clampf(g_mic_settings.sensitivity, 0.10f, 10.0f);
    g_mic_settings.calibrating = false;
    save_mic_settings_locked();
}

struct PcmReader {
    virtual ~PcmReader() = default;
    virtual bool open(SpectrumInputMode mode, int attempt) = 0;
    virtual int read(int16_t *samples, size_t frames) = 0;
    virtual const char *source() const = 0;
    virtual const char *status() const = 0;
    virtual bool live() const = 0;
};

class SyntheticReader : public PcmReader {
public:
    bool open(SpectrumInputMode mode, int) override
    {
        mode_ = mode;
        phase1_ = 0.0;
        phase2_ = 0.0;
        std::snprintf(source_, sizeof(source_), "%s synthetic", mode_name(mode_));
        std::snprintf(status_, sizeof(status_), "No live capture; synthetic debug signal");
        return true;
    }

    int read(int16_t *samples, size_t frames) override
    {
        const double freq1 = mode_ == SpectrumInputMode::Music ? 164.81 : 220.0;
        const double freq2 = mode_ == SpectrumInputMode::Music ? 523.25 : 880.0;
        const double step1 = kTwoPi * freq1 / static_cast<double>(kSampleRate);
        const double step2 = kTwoPi * freq2 / static_cast<double>(kSampleRate);
        double wobble = std::sin(time_ * 0.031) * 0.4 + 0.6;
        for (size_t i = 0; i < frames; ++i) {
            double v = std::sin(phase1_) * 0.45 + std::sin(phase2_) * 0.25 * wobble;
            if ((i + tick_) % 91 == 0) v += 0.45;
            samples[i] = static_cast<int16_t>(clampf(static_cast<float>(v), -1.0f, 1.0f) * 24000.0f);
            phase1_ += step1;
            phase2_ += step2;
            if (phase1_ > kTwoPi) phase1_ -= kTwoPi;
            if (phase2_ > kTwoPi) phase2_ -= kTwoPi;
            time_ += 1.0;
        }
        tick_ += frames;
        std::this_thread::sleep_for(std::chrono::milliseconds(24));
        return static_cast<int>(frames);
    }

    const char *source() const override { return source_; }
    const char *status() const override { return status_; }
    bool live() const override { return false; }

private:
    SpectrumInputMode mode_ = SpectrumInputMode::Mic;
    double phase1_ = 0.0;
    double phase2_ = 0.0;
    double time_ = 0.0;
    size_t tick_ = 0;
    char source_[96] = {};
    char status_[128] = {};
};

class CommandReader : public PcmReader {
public:
    ~CommandReader() override
    {
        close();
    }

    bool open(SpectrumInputMode mode, int) override
    {
        if (mode == SpectrumInputMode::Mic) {
            if (!env_enabled("M5_SPECTRUM_USE_MIC_CMD", true)) return false;
            int mic_source = g_requested_mic_source.load();
            configure_mic_source(mic_source);
            const char *env_cmd = std::getenv("M5_SPECTRUM_MIC_CMD");
            command_ = env_cmd && env_cmd[0]
                           ? env_cmd
                           : "exec arecord -q -D ${M5_SPECTRUM_MIC_ALSA_DEVICE:-plughw:1,0} -f S16_LE -r 16000 -c 1 -t raw 2>/dev/null";
            std::snprintf(source_, sizeof(source_), "ALSA MIC %s arecord", mic_source_name(mic_source));
            std::snprintf(status_, sizeof(status_), "MIC %s live via plughw", mic_source_name(mic_source));
        } else {
            if (!env_enabled("M5_SPECTRUM_USE_PULSE_MONITOR", true)) return false;
            const char *env_cmd = std::getenv("M5_SPECTRUM_MUSIC_CMD");
            command_ = env_cmd && env_cmd[0]
                           ? env_cmd
                           : "sink=$(pactl get-default-sink 2>/dev/null) && exec parec --raw --format=s16le --rate=16000 --channels=1 --device=\"$sink.monitor\" 2>/dev/null";
            std::snprintf(source_, sizeof(source_), "PipeWire/Pulse monitor");
            std::snprintf(status_, sizeof(status_), "MUSIC from sink monitor");
        }

        pipe_ = popen(command_.c_str(), "r");
        if (!pipe_) return false;
        return true;
    }

    int read(int16_t *samples, size_t frames) override
    {
        if (!pipe_) return -1;
        const size_t want = frames * sizeof(int16_t);
        size_t got = std::fread(samples, 1, want, pipe_);
        if (got == 0) return -1;
        if (got < want) {
            std::memset(reinterpret_cast<char *>(samples) + got, 0, want - got);
        }
        return static_cast<int>(frames);
    }

    const char *source() const override { return source_; }
    const char *status() const override { return status_; }
    bool live() const override { return true; }

private:
    void close()
    {
        if (pipe_) {
            pclose(pipe_);
            pipe_ = nullptr;
        }
    }

    FILE *pipe_ = nullptr;
    std::string command_;
    char source_[96] = {};
    char status_[128] = {};
};

#if SPECTRUM_HAS_TINYALSA
bool parse_capture_device_line(const char *line, unsigned int *card, unsigned int *device)
{
    if (!line || std::strstr(line, "capture") == nullptr) return false;
    return std::sscanf(line, "%u-%u:", card, device) == 2;
}

bool detect_capture_device(bool prefer_monitor, unsigned int *card, unsigned int *device)
{
    FILE *fp = std::fopen("/proc/asound/pcm", "r");
    if (!fp) return false;

    bool have_first = false;
    unsigned int first_card = 0;
    unsigned int first_device = 0;
    char line[256];
    while (std::fgets(line, sizeof(line), fp)) {
        unsigned int c = 0;
        unsigned int d = 0;
        if (!parse_capture_device_line(line, &c, &d)) continue;
        if (!have_first) {
            first_card = c;
            first_device = d;
            have_first = true;
        }
        if (prefer_monitor &&
            (std::strstr(line, "Loopback") || std::strstr(line, "loopback") ||
             std::strstr(line, "Monitor") || std::strstr(line, "monitor") ||
             std::strstr(line, "pulse") || std::strstr(line, "PipeWire"))) {
            *card = c;
            *device = d;
            std::fclose(fp);
            return true;
        }
    }
    std::fclose(fp);

    if (have_first) {
        *card = first_card;
        *device = first_device;
        return true;
    }
    return false;
}

class TinyAlsaReader : public PcmReader {
public:
    ~TinyAlsaReader() override
    {
        close();
    }

    bool open(SpectrumInputMode mode, int) override
    {
        mode_ = mode;
        unsigned int card = 0;
        unsigned int device = 0;

        if (mode == SpectrumInputMode::Music) {
            if (!read_env_uint("M5_SPECTRUM_MUSIC_CARD", &card) ||
                !read_env_uint("M5_SPECTRUM_MUSIC_DEVICE", &device)) {
                if (!detect_capture_device(true, &card, &device)) return false;
            }
        } else {
            if (!read_env_uint("M5_SPECTRUM_MIC_CARD", &card) ||
                !read_env_uint("M5_SPECTRUM_MIC_DEVICE", &device)) {
                if (!detect_capture_device(false, &card, &device)) return false;
            }
            configure_mic_source(g_requested_mic_source.load());
        }

        if (!open_pcm(card, device, 1) && !open_pcm(card, device, 2)) {
            return false;
        }

        std::snprintf(source_, sizeof(source_), "ALSA %s hw:%u,%u %uch", mode_name(mode_), card, device, channels_);
        mix_channels_ = env_enabled("M5_SPECTRUM_MIX_CHANNELS", false);
        std::snprintf(status_, sizeof(status_), "%s live capture%s",
                      mode_name(mode_), channels_ > 1 && !mix_channels_ ? " auto-channel" : "");
        return true;
    }

    int read(int16_t *samples, size_t frames) override
    {
        if (!pcm_) return -1;
        int result = 0;
        if (channels_ <= 1) {
            result = pcm_readi(pcm_, samples, static_cast<unsigned int>(frames));
        } else {
            interleaved_.resize(frames * channels_);
            result = pcm_readi(pcm_, interleaved_.data(), static_cast<unsigned int>(frames));
            if (result > 0) {
                unsigned int best_channel = 0;
                if (!mix_channels_) {
                    std::vector<long long> energy(channels_, 0);
                    for (int frame = 0; frame < result; ++frame) {
                        for (unsigned int ch = 0; ch < channels_; ++ch) {
                            int16_t value = interleaved_[static_cast<size_t>(frame) * channels_ + ch];
                            energy[ch] += std::abs(static_cast<int>(value));
                        }
                    }
                    best_channel = static_cast<unsigned int>(
                        std::max_element(energy.begin(), energy.end()) - energy.begin());
                }
                for (int frame = 0; frame < result; ++frame) {
                    if (mix_channels_) {
                        int mixed = 0;
                        for (unsigned int ch = 0; ch < channels_; ++ch) {
                            mixed += interleaved_[static_cast<size_t>(frame) * channels_ + ch];
                        }
                        samples[frame] = static_cast<int16_t>(mixed / static_cast<int>(channels_));
                    } else {
                        samples[frame] = interleaved_[static_cast<size_t>(frame) * channels_ + best_channel];
                    }
                }
            }
        }
        if (result < 0) {
            std::snprintf(status_, sizeof(status_), "ALSA read failed: %s", pcm_get_error(pcm_));
            close();
            return -1;
        }
        return result;
    }

    const char *source() const override { return source_; }
    const char *status() const override { return status_; }
    bool live() const override { return true; }

private:
    bool open_pcm(unsigned int card, unsigned int device, unsigned int channels)
    {
        close();
        std::memset(&config_, 0, sizeof(config_));
        config_.channels = channels;
        config_.rate = kSampleRate;
        config_.period_size = 256;
        config_.period_count = 4;
        config_.format = PCM_FORMAT_S16_LE;
        config_.start_threshold = 0;
        config_.stop_threshold = 0;
        config_.silence_threshold = 0;

        pcm_ = pcm_open(card, device, PCM_IN, &config_);
        if (!pcm_ || !pcm_is_ready(pcm_)) {
            const char *error = pcm_ ? pcm_get_error(pcm_) : "pcm_open failed";
            std::snprintf(status_, sizeof(status_), "ALSA %uch open failed: %s", channels, error);
            close();
            return false;
        }

        channels_ = channels;
        return true;
    }

    void close()
    {
        if (pcm_) {
            pcm_close(pcm_);
            pcm_ = nullptr;
        }
    }

    SpectrumInputMode mode_ = SpectrumInputMode::Mic;
    struct pcm_config config_ {};
    struct pcm *pcm_ = nullptr;
    unsigned int channels_ = 1;
    bool mix_channels_ = false;
    std::vector<int16_t> interleaved_;
    char source_[96] = {};
    char status_[128] = {};
};
#endif

std::unique_ptr<PcmReader> make_reader(SpectrumInputMode mode, int attempt)
{
    if (attempt == 0) {
        auto reader = std::make_unique<CommandReader>();
        if (reader->open(mode, attempt)) return reader;
    }

#if SPECTRUM_HAS_TINYALSA
    if (attempt <= 1) {
        auto reader = std::make_unique<TinyAlsaReader>();
        if (reader->open(mode, attempt)) return reader;
    }
#endif

    auto reader = std::make_unique<SyntheticReader>();
    reader->open(mode, attempt);
    return reader;
}

float scaled_sample(int16_t sample, float gain)
{
    return clampf(static_cast<float>(sample) / 32768.0f * gain, -1.0f, 1.0f);
}

float scaled_raw_sample(float sample, float gain)
{
    return clampf(sample * gain, -1.0f, 1.0f);
}

float mic_speech_level(float raw_rms_counts, float raw_peak_counts, const MicSettingsSnapshot &settings)
{
    float detector = std::max(raw_rms_counts, raw_peak_counts * 0.08f);
    float signal = std::max(0.0f, detector - settings.floor) * settings.sensitivity;
    float level = std::log1p(signal) / std::log1p(settings.span);
    return clampf(level, 0.0f, 1.0f);
}

void analyze_block(const int16_t *samples, size_t frames, SpectrumInputMode mode, SpectrumSnapshot *snapshot)
{
    const size_t used_frames = std::min(frames, kBlockFrames);
    std::array<float, kBlockFrames> processed {};

    if (used_frames < 4) {
        std::fill(std::begin(snapshot->bands), std::end(snapshot->bands), 0.0f);
        snapshot->rms = 0.0f;
        snapshot->peak = 0.0f;
        return;
    }

    double raw_sum = 0.0;
    for (size_t i = 0; i < used_frames; ++i) {
        processed[i] = static_cast<float>(samples[i]) / 32768.0f;
        raw_sum += processed[i];
    }

    const float raw_mean = used_frames > 0 ? static_cast<float>(raw_sum / static_cast<double>(used_frames)) : 0.0f;
    double raw_sum_sq = 0.0;
    float raw_peak = 0.0f;
    for (size_t i = 0; i < used_frames; ++i) {
        processed[i] -= raw_mean;
        raw_sum_sq += static_cast<double>(processed[i]) * static_cast<double>(processed[i]);
        raw_peak = std::max(raw_peak, std::fabs(processed[i]));
    }

    const float raw_rms = static_cast<float>(std::sqrt(raw_sum_sq / std::max<size_t>(used_frames, 1)));
    const float raw_rms_counts = raw_rms * 32768.0f;
    const float raw_peak_counts = raw_peak * 32768.0f;
    if (mode == SpectrumInputMode::Mic) {
        update_calibration(raw_rms_counts, raw_peak_counts);
    }
    MicSettingsSnapshot mic_settings = current_mic_settings();
    const float speech_level = mode == SpectrumInputMode::Mic
                                   ? mic_speech_level(raw_rms_counts, raw_peak_counts, mic_settings)
                                   : 0.0f;
    const float gain = mode == SpectrumInputMode::Mic ? input_gain(mode) : automatic_gain(mode, input_gain(mode), raw_rms);
    double sum_sq = 0.0;
    float peak = 0.0f;
    for (size_t i = 0; i < used_frames; ++i) {
        processed[i] = scaled_raw_sample(processed[i], gain);
        sum_sq += static_cast<double>(processed[i]) * static_cast<double>(processed[i]);
        peak = std::max(peak, std::fabs(processed[i]));
    }
    snapshot->rms = static_cast<float>(std::sqrt(sum_sq / std::max<size_t>(used_frames, 1)));
    if (mode == SpectrumInputMode::Mic) {
        snapshot->rms = speech_level;
    }
    snapshot->peak = peak;

    for (int band = 0; band < SPECTRUM_BAND_COUNT; ++band) {
        double t = static_cast<double>(band) / static_cast<double>(SPECTRUM_BAND_COUNT - 1);
        double freq = 70.0 * std::pow(6400.0 / 70.0, t);
        int k = static_cast<int>(std::lround(freq * static_cast<double>(used_frames) / kSampleRate));
        if (k < 1) k = 1;
        if (k > static_cast<int>(used_frames / 2 - 1)) k = static_cast<int>(used_frames / 2 - 1);

        double coeff = 2.0 * std::cos(kTwoPi * static_cast<double>(k) / static_cast<double>(used_frames));
        double q0 = 0.0;
        double q1 = 0.0;
        double q2 = 0.0;
        for (size_t i = 0; i < used_frames; ++i) {
            double window = 0.5 - 0.5 * std::cos(kTwoPi * static_cast<double>(i) / static_cast<double>(used_frames - 1));
            double s = static_cast<double>(processed[i]) * window;
            q0 = coeff * q1 - q2 + s;
            q2 = q1;
            q1 = q0;
        }
        double power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
        float magnitude = static_cast<float>(std::sqrt(std::max(power, 0.0)) / static_cast<double>(used_frames));
        float scale = mode == SpectrumInputMode::Mic
                          ? mic_settings.band_scale * mic_settings.sensitivity
                          : 90.0f;
        float level = std::log10(1.0f + magnitude * scale);
        snapshot->bands[band] = clampf(level, 0.0f, 1.0f);
    }

    if (mode == SpectrumInputMode::Mic && snapshot->live) {
        if (mic_settings.calibrating) {
            std::snprintf(snapshot->status, sizeof(snapshot->status),
                          "Cal quiet %2d%% raw %.2f",
                          static_cast<int>(mic_settings.calibration_progress * 100.0f),
                          raw_rms_counts);
        } else {
            std::snprintf(snapshot->status, sizeof(snapshot->status),
                          "MIC raw %.2f pk %.1f lvl %.0f S%.1f",
                          raw_rms_counts, raw_peak_counts, speech_level * 100.0f,
                          mic_settings.sensitivity);
        }
    }
    snapshot->raw_rms = raw_rms_counts;
    snapshot->raw_peak = raw_peak_counts;
    snapshot->level = speech_level;
    snapshot->mic_floor = mic_settings.floor;
    snapshot->mic_span = mic_settings.span;
    snapshot->mic_sensitivity = mic_settings.sensitivity;
    snapshot->calibrating = mic_settings.calibrating;
    snapshot->calibration_progress = mic_settings.calibration_progress;
}

struct AudioState {
    std::atomic<bool> running {false};
    std::atomic<int> requested_mode {static_cast<int>(SpectrumInputMode::Mic)};
    std::thread worker;
    std::mutex mutex;
    SpectrumSnapshot snapshot {};
};

AudioState g_audio;

void publish_snapshot(const SpectrumSnapshot &snapshot)
{
    std::lock_guard<std::mutex> lock(g_audio.mutex);
    float attack = 0.62f;
    float release = 0.18f;
    for (int i = 0; i < SPECTRUM_BAND_COUNT; ++i) {
        float current = g_audio.snapshot.bands[i];
        float next = snapshot.bands[i];
        float mix = next > current ? attack : release;
        g_audio.snapshot.bands[i] = current + (next - current) * mix;
    }
    g_audio.snapshot.rms = snapshot.rms;
    g_audio.snapshot.peak = snapshot.peak;
    g_audio.snapshot.raw_rms = snapshot.raw_rms;
    g_audio.snapshot.raw_peak = snapshot.raw_peak;
    g_audio.snapshot.level = snapshot.level;
    g_audio.snapshot.mic_floor = snapshot.mic_floor;
    g_audio.snapshot.mic_span = snapshot.mic_span;
    g_audio.snapshot.mic_sensitivity = snapshot.mic_sensitivity;
    g_audio.snapshot.calibration_progress = snapshot.calibration_progress;
    g_audio.snapshot.mode = snapshot.mode;
    g_audio.snapshot.live = snapshot.live;
    g_audio.snapshot.calibrating = snapshot.calibrating;
    std::snprintf(g_audio.snapshot.source, sizeof(g_audio.snapshot.source), "%s", snapshot.source);
    std::snprintf(g_audio.snapshot.status, sizeof(g_audio.snapshot.status), "%s", snapshot.status);
}

void worker_loop()
{
    std::array<int16_t, kBlockFrames> samples {};
    SpectrumInputMode active_mode = SpectrumInputMode::Mic;
    int active_mode_value = -1;
    unsigned int active_config_generation = 0;
    int attempt = 0;
    int failures = 0;
    std::unique_ptr<PcmReader> reader;

    while (g_audio.running.load()) {
        int requested = g_audio.requested_mode.load();
        unsigned int requested_config_generation = g_audio_config_generation.load();
        if (!reader || requested != active_mode_value || requested_config_generation != active_config_generation) {
            active_mode = requested == static_cast<int>(SpectrumInputMode::Music)
                              ? SpectrumInputMode::Music
                              : SpectrumInputMode::Mic;
            active_mode_value = requested;
            active_config_generation = requested_config_generation;
            attempt = 0;
            failures = 0;
            reader = make_reader(active_mode, attempt);
            std::printf("Audio Spectrum source: %s (%s)\n", reader->source(), reader->status());
            std::fflush(stdout);
        }

        int got = reader->read(samples.data(), samples.size());
        if (got <= 0) {
            ++failures;
            if (failures >= 2) {
                ++attempt;
                reader = make_reader(active_mode, attempt);
                std::printf("Audio Spectrum source: %s (%s)\n", reader->source(), reader->status());
                std::fflush(stdout);
                failures = 0;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            continue;
        }

        failures = 0;
        SpectrumSnapshot next {};
        next.mode = active_mode;
        next.live = reader->live();
        std::snprintf(next.source, sizeof(next.source), "%s", reader->source());
        std::snprintf(next.status, sizeof(next.status), "%s", reader->status());
        analyze_block(samples.data(), static_cast<size_t>(got), active_mode, &next);
        publish_snapshot(next);
    }
}

}  // namespace

void spectrum_audio_start(void)
{
    if (g_audio.running.exchange(true)) return;
    {
        std::lock_guard<std::mutex> lock(g_audio.mutex);
        g_audio.snapshot.mode = SpectrumInputMode::Mic;
        g_audio.snapshot.live = false;
        std::snprintf(g_audio.snapshot.source, sizeof(g_audio.snapshot.source), "Starting");
        std::snprintf(g_audio.snapshot.status, sizeof(g_audio.snapshot.status), "Opening audio input");
    }
    g_audio.worker = std::thread(worker_loop);
}

void spectrum_audio_stop(void)
{
    if (!g_audio.running.exchange(false)) return;
    if (g_audio.worker.joinable()) {
        g_audio.worker.join();
    }
}

void spectrum_audio_toggle_mode(void)
{
    int current = g_audio.requested_mode.load();
    int next = current == static_cast<int>(SpectrumInputMode::Music)
                   ? static_cast<int>(SpectrumInputMode::Mic)
                   : static_cast<int>(SpectrumInputMode::Music);
    g_audio.requested_mode.store(next);
}

void spectrum_audio_set_mode(SpectrumInputMode mode)
{
    g_audio.requested_mode.store(static_cast<int>(mode));
}

void spectrum_audio_toggle_mic_source(void)
{
    int current = g_requested_mic_source.load();
    g_requested_mic_source.store(current == 1 ? 0 : 1);
    g_audio_config_generation.fetch_add(1);
}

const char *spectrum_audio_mic_source_name(void)
{
    return mic_source_name(g_requested_mic_source.load());
}

void spectrum_audio_start_calibration(void)
{
    std::lock_guard<std::mutex> lock(g_mic_settings.mutex);
    load_mic_settings_locked();
    g_mic_settings.calibrating = true;
    g_mic_settings.calibration_blocks = 0;
    g_mic_settings.detector_sum = 0.0;
    g_mic_settings.rms_sum = 0.0;
    g_mic_settings.detector_max = 0.0f;
    g_mic_settings.peak_max = 0.0f;
}

void spectrum_audio_adjust_sensitivity(float factor)
{
    std::lock_guard<std::mutex> lock(g_mic_settings.mutex);
    load_mic_settings_locked();
    if (!std::isfinite(factor) || factor <= 0.0f) return;
    g_mic_settings.sensitivity = clampf(g_mic_settings.sensitivity * factor, 0.15f, 8.0f);
    save_mic_settings_locked();
}

void spectrum_audio_format_settings(char *buffer, size_t size)
{
    if (!buffer || size == 0) return;
    std::lock_guard<std::mutex> lock(g_mic_settings.mutex);
    load_mic_settings_locked();
    std::snprintf(buffer, size, "S%.2f F%.1f R%.0f B%.0f",
                  g_mic_settings.sensitivity,
                  g_mic_settings.floor,
                  g_mic_settings.span,
                  g_mic_settings.band_scale);
}

void spectrum_audio_get_snapshot(SpectrumSnapshot *snapshot)
{
    if (!snapshot) return;
    std::lock_guard<std::mutex> lock(g_audio.mutex);
    *snapshot = g_audio.snapshot;
}
