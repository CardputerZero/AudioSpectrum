#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <stdbool.h>
#include <stddef.h>

constexpr int SPECTRUM_BAND_COUNT = 32;

enum class SpectrumInputMode {
    Mic = 0,
    Music = 1,
};

struct SpectrumSnapshot {
    float bands[SPECTRUM_BAND_COUNT];
    float rms;
    float peak;
    float raw_rms;
    float raw_peak;
    float level;
    float mic_floor;
    float mic_span;
    float mic_sensitivity;
    float calibration_progress;
    SpectrumInputMode mode;
    bool live;
    bool calibrating;
    char source[96];
    char status[128];
};

void spectrum_audio_start(void);
void spectrum_audio_stop(void);
void spectrum_audio_toggle_mode(void);
void spectrum_audio_set_mode(SpectrumInputMode mode);
void spectrum_audio_toggle_mic_source(void);
const char *spectrum_audio_mic_source_name(void);
void spectrum_audio_start_calibration(void);
void spectrum_audio_adjust_sensitivity(float factor);
void spectrum_audio_format_settings(char *buffer, size_t size);
void spectrum_audio_get_snapshot(SpectrumSnapshot *snapshot);

#endif
