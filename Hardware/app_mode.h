#ifndef APP_MODE_H
#define APP_MODE_H

#define APP_MODE_LEGACY       0
#define APP_MODE_SERVO        1
#define APP_MODE_STEPPER      2
#define APP_MODE_RACE         3

#ifndef APP_RUN_MODE
#define APP_RUN_MODE APP_MODE_RACE
#endif

#if APP_RUN_MODE < APP_MODE_LEGACY || APP_RUN_MODE > APP_MODE_RACE
#error Invalid APP_RUN_MODE
#endif

#endif
