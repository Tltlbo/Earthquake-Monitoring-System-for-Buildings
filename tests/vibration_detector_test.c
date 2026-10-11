#include <assert.h>
#include <stdio.h>
#include "vibration_detector.h"

static uint32_t tick;

static void feed(VibrationDetector *d, int16_t x, int16_t y, int16_t z)
{
    tick += VIBRATION_SAMPLE_MS;
    Vibration_Update(d, tick, x, y, z);
}

static void calibrate(VibrationDetector *d)
{
    Vibration_Init(d);
    for (unsigned i = 0; i < VIBRATION_CALIBRATION; ++i)
        feed(d, 0, 0, 16384);
    assert(!d->result.valid);
}

int main(void)
{
    VibrationDetector d;
    calibrate(&d);
    for (unsigned i = 0; i < VIBRATION_WINDOW - 1; ++i)
        feed(&d, 0, 0, 16384);
    assert(!d.result.valid);
    feed(&d, 0, 0, 16384);
    assert(d.result.valid && !d.result.abnormal);
    assert(d.result.p2p == 0 && d.result.rms == 0);

    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, i % 2 ? -100 : 100, 0, 16384);
    assert(d.result.p2p == 200 && d.result.rms == 100);
    assert(!d.result.abnormal);

    /* Signed oscillation: zero mean does not hide vibration. */
    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, i % 2 ? -2000 : 2000, 0, 16384);
    assert(d.result.p2p == 4000 && d.result.rms == 2000);
    assert(d.result.abnormal);

    /* Quiet samples eventually replace the entire sliding window. */
    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, 0, 0, 16384);
    assert(!d.result.abnormal);

    /* P2P-only transient: RMS is diluted across the window. */
    feed(&d, 2000, 0, 16384);
    assert(d.result.p2p == 2000 && d.result.rms == 447);
    assert(d.result.abnormal);

    /* RMS-only deviation from the fixed rest baseline. */
    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, 0, 800, 16384);
    assert(d.result.p2p == 0 && d.result.rms == 800);
    assert(d.result.abnormal);

    /* Independent axes: strongest P2P and RMS need not be on the same axis. */
    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, i == 0 ? 2000 : 0, 800, 16384);
    assert(d.result.p2p == 2000 && d.result.rms == 800);

    /* Full-scale signed inputs must not overflow integer arithmetic. */
    for (unsigned i = 0; i < VIBRATION_WINDOW; ++i)
        feed(&d, i % 2 ? INT16_MIN : INT16_MAX, 0, 16384);
    assert(d.result.p2p == 65535 && d.result.rms == 32768);

    /* A missed interval invalidates the window, rather than stretching it. */
    tick += VIBRATION_MAX_GAP_MS;
    feed(&d, 0, 0, 16384);
    assert(!d.result.valid && d.count == 1);

    /* Unsigned tick subtraction supports HAL tick wraparound. */
    d.last_tick = UINT32_MAX - 5U;
    tick = 4U;
    Vibration_Update(&d, tick, 0, 0, 16384);
    assert(d.count == 2);

    /* A gap during calibration restarts the stationary baseline collection. */
    Vibration_Init(&d);
    feed(&d, 0, 0, 16384);
    tick += VIBRATION_MAX_GAP_MS;
    feed(&d, 0, 0, 16384);
    assert(d.calibration_count == 1);
    puts("vibration detector tests passed");
    return 0;
}
