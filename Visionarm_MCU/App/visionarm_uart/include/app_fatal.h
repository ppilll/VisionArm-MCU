#ifndef VISIONARM_APP_FATAL_H
#define VISIONARM_APP_FATAL_H

#include <stdint.h>

typedef enum
{
    APP_FATAL_NONE = 0,
    APP_FATAL_ASSERT,
    APP_FATAL_STACK_OVERFLOW
} AppFatalReason;

typedef struct
{
    volatile AppFatalReason reason;
    const char *volatile assert_file;
    volatile int assert_line;
    volatile uint32_t task_handle;
} AppFatalState;

extern volatile AppFatalState g_app_fatal_state;

void AppFatal_Assert(const char *file, int line);

#endif /* VISIONARM_APP_FATAL_H */
