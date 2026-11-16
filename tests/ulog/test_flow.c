/* Reuse the existing simulated board, keys, IMU and actuators. */
#define Servo_IsBusy original_servo_busy
#define AppStepper_IsBusy original_stepper_busy
#define main original_race_test_main
#include "../race/test_app.c"
#undef main
#undef Servo_IsBusy
#undef AppStepper_IsBusy
static uint8_t servo_busy, stepper_busy;
uint8_t Servo_IsBusy(uint8_t id) { (void)id; return servo_busy; }
uint8_t AppStepper_IsBusy(StepperAxis_t axis) { (void)axis; return stepper_busy; }
int main(void)
{
#ifdef TEST_DIRECT
    init(0U);
    tick(100U);
    assert(!AppRace_LogFinished()); /* Boot reset must not finish the log. */
    click();
    assert(!AppRace_LogFinished());
    stepper_busy=1U; tick(100U); assert(!AppRace_LogFinished());
    stepper_busy=0U;
    tick(4000U);
    assert(AppRace_LogFinished()); /* Pick and its post-cycle reset are over. */
    servo_busy=1U; assert(!AppRace_LogFinished()); servo_busy=0U;
    stepper_busy=1U; assert(!AppRace_LogFinished()); stepper_busy=0U;
    tick(1000U);
    assert(AppRace_LogFinished());
    init(0U); tick(100U); click(); s_imu_ok=0U;
    /* Direct test ignores IMU while READY, so exercise watchdog fault. */
    s_now += RACE_IMU_STALE_MS;
    (void)AppRace_ControlTick(); AppRace_Poll();
    assert(AppRace_LogFinished());
#else
    original_race_test_main();
    assert(AppRace_LogFinished());
    init(0U); assert(!AppRace_LogFinished());
    start(); assert(!AppRace_LogFinished());
    click(); assert(AppRace_LogFinished());
#endif
    puts("PASS flow completion / boot reset / abort");
    return 0;
}
