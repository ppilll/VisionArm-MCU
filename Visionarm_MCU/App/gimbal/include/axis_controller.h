#ifndef VISIONARM_AXIS_CONTROLLER_H
#define VISIONARM_AXIS_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t min_us;
    uint16_t center_us;
    uint16_t max_us;

    int16_t dead_zone_q15;
    uint16_t gain_us_per_full_scale_cycle;
    uint16_t max_delta_us_per_cycle;

    bool invert_error;
} AxisControllerConfig;

typedef struct
{
    int32_t target_us;

    int16_t last_input_q15;
    int32_t signed_error_q15;

    int32_t requested_delta_us;
    int32_t applied_delta_us;

    bool dead_zone_active;
    bool limit_blocked;

    uint32_t update_count;
    uint32_t dead_zone_count;
    uint32_t slew_limit_count;
    uint32_t limit_hit_count;
} AxisController;

typedef struct
{
    uint16_t target_us;

    int32_t signed_error_q15;
    int32_t requested_delta_us;
    int32_t applied_delta_us;

    bool dead_zone_active;
    bool slew_limited;
    bool limit_blocked;
} AxisControllerStepResult;

bool AxisController_Init(AxisController *controller,
                         const AxisControllerConfig *config);

void AxisController_ResetCenter(AxisController *controller,
                                const AxisControllerConfig *config);

bool AxisController_Step(AxisController *controller,
                         const AxisControllerConfig *config,
                         int16_t error_q15,
                         AxisControllerStepResult *result);

bool AxisController_RunSelfTest(void);

#endif /* VISIONARM_AXIS_CONTROLLER_H */
