#ifndef VISIONARM_PWM_BENCH_TEST_H
#define VISIONARM_PWM_BENCH_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    PWM_BENCH_PHASE_BOOT_SAFE = 0,
    PWM_BENCH_PHASE_PAN_500_TILT_2500,
    PWM_BENCH_PHASE_BOTH_1500,
    PWM_BENCH_PHASE_PAN_2500_TILT_500,
    PWM_BENCH_PHASE_OUTPUT_DISABLED,
    PWM_BENCH_PHASE_FAULT
} PwmBenchPhase;

typedef struct
{
    PwmBenchPhase phase;
    uint32_t completed_cycles;
    uint32_t failure_count;
} PwmBenchSnapshot;

/* Step-C bench-only task. Keep servo signal wires disconnected while enabled. */
bool PwmBenchTest_Create(void);
void PwmBenchTest_GetSnapshot(PwmBenchSnapshot *snapshot);

#endif /* VISIONARM_PWM_BENCH_TEST_H */
