#ifndef VISIONARM_GIMBAL_STUB_H
#define VISIONARM_GIMBAL_STUB_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t pan_q15;
    int16_t tilt_q15;
    bool active;
} GimbalStubSnapshot;

bool GimbalStubTask_Create(void);
void GimbalStub_GetSnapshot(GimbalStubSnapshot *snapshot);

#endif /* VISIONARM_GIMBAL_STUB_H */
