#ifndef VIBRATION_DETECTOR_H
#define VIBRATION_DETECTOR_H

#include <stdint.h>
#include <string.h>
#include <math.h>

/* Raw-count thresholds are provisional; calibrate against real recordings. */
#define VIBRATION_SAMPLE_MS 10U
#define VIBRATION_MAX_GAP_MS 30U
#define VIBRATION_WINDOW 20U
#define VIBRATION_CALIBRATION 100U
#define VIBRATION_P2P_THRESHOLD 2000.0f
#define VIBRATION_RMS_THRESHOLD 700.0f

typedef struct {
    uint32_t p2p;
    uint32_t rms;
    uint8_t abnormal;
    uint8_t valid;
} VibrationResult;

typedef struct {
    float samples[VIBRATION_WINDOW][3];
    float baseline[3];
    float calibration_sum[3];
    uint32_t calibration_count;
    uint32_t index;
    uint32_t count;
    uint32_t last_tick;
    uint8_t has_tick;
    VibrationResult result;
} VibrationDetector;

static inline void Vibration_Init(VibrationDetector *d)
{
    memset(d, 0, sizeof(*d));
}

/* Call only with a fresh, successful measurement. No software FIR/IIR. */
static inline void Vibration_Update(VibrationDetector *d, uint32_t tick,
                                    int16_t ax, int16_t ay, int16_t az)
{
    float raw[3] = {(float)ax, (float)ay, (float)az};
    if (d->has_tick && (uint32_t)(tick - d->last_tick) > VIBRATION_MAX_GAP_MS) {
        d->count = d->index = 0;
        memset(&d->result, 0, sizeof(d->result));
        if (d->calibration_count < VIBRATION_CALIBRATION) {
            d->calibration_count = 0;
            memset(d->calibration_sum, 0, sizeof(d->calibration_sum));
        }
    }
    d->has_tick = 1;
    d->last_tick = tick;

    /* Keep the installed sensor stationary during startup calibration. */
    if (d->calibration_count < VIBRATION_CALIBRATION) {
        for (unsigned axis = 0; axis < 3; ++axis)
            d->calibration_sum[axis] += raw[axis];
        if (++d->calibration_count == VIBRATION_CALIBRATION)
            for (unsigned axis = 0; axis < 3; ++axis)
                d->baseline[axis] = d->calibration_sum[axis] / VIBRATION_CALIBRATION;
        return;
    }

    for (unsigned axis = 0; axis < 3; ++axis)
        d->samples[d->index][axis] = raw[axis] - d->baseline[axis];
    d->index = (d->index + 1U) % VIBRATION_WINDOW;
    if (d->count < VIBRATION_WINDOW)
        ++d->count;
    if (d->count < VIBRATION_WINDOW)
        return;

    float max_p2p = 0.0f, max_rms = 0.0f;
    for (unsigned axis = 0; axis < 3; ++axis) {
        float minimum = d->samples[0][axis];
        float maximum = minimum, squares = 0.0f;
        for (unsigned i = 0; i < VIBRATION_WINDOW; ++i) {
            float value = d->samples[i][axis];
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
            squares += value * value;
        }
        float p2p = maximum - minimum;
        float rms = sqrtf(squares / VIBRATION_WINDOW);
        if (p2p > max_p2p) max_p2p = p2p;
        if (rms > max_rms) max_rms = rms;
    }
    d->result.p2p = (uint32_t)(max_p2p + 0.5f);
    d->result.rms = (uint32_t)(max_rms + 0.5f);
    d->result.abnormal = max_p2p >= VIBRATION_P2P_THRESHOLD ||
                         max_rms >= VIBRATION_RMS_THRESHOLD;
    d->result.valid = 1;
}

#endif
