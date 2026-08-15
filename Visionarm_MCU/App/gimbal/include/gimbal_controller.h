#ifndef VISIONARM_GIMBAL_CONTROLLER_H
#define VISIONARM_GIMBAL_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "actuator_driver.h"
#include "axis_controller.h"

typedef struct
{
    AxisController pan;
    AxisController tilt;
    uint32_t step_count;
} GimbalController;

typedef struct
{
    ActuatorCommand command;
    AxisControllerStepResult pan;
    AxisControllerStepResult tilt;
} GimbalControllerOutput;

bool GimbalController_Init(GimbalController *controller);
void GimbalController_ResetCenter(GimbalController *controller);

bool GimbalController_Step(GimbalController *controller,
                           int16_t error_x_q15,
                           int16_t error_y_q15,
                           GimbalControllerOutput *output);

bool GimbalController_RunSelfTest(void);

#endif /* VISIONARM_GIMBAL_CONTROLLER_H */
