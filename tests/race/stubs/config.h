/* Route tests always exercise the route, independent of bench-test settings. */
#include "race_config.h"
#undef RACE_PICK_DIRECT_KEY_TEST
#undef RACE_PICK_ENABLE
#define RACE_PICK_DIRECT_KEY_TEST 0U
#define RACE_PICK_ENABLE 0U
