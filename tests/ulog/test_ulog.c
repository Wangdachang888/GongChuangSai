#if defined(APP_PID_TEST_HOST)
#include "board.h"
#include "app_pid_test.h"
#include "app_runtime.h"
#include "bsp_ulog.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

Motor_t motor_left_front, motor_right_front, motor_left_back, motor_right_back;
MecanumChassis_t chassis;
volatile uint8_t g_tune_mode;
static uint32_t now, irq, stops, commands, frames;
static uint8_t key, imu_ok = 1u, bias_ready = 1u, capture_armed;
static unsigned live_count, cache_count, end_count, used_count;
static uint8_t cache_end;
void PidFixture_Init(void);
void PidFixture_Poll(void);
void PidFixture_Snapshot(void);
void PidFixture_Observe(void);

uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return irq; }
void __disable_irq(void) { irq = 1u; }
void __set_PRIMASK(uint32_t value) { irq = value; }
uint32_t app_imu_frames(void) { return frames; }
bool app_imu_ok(void) { return imu_ok; }
bool app_imu_bias_ready(void) { return bias_ready; }
float app_imu_yaw_deg(void) { return 0.0f; }
int HAL_GPIO_ReadPin(void *port, uint16_t pin) { (void)port; (void)pin; return !key; }
float Vbat_GetDriverVoltage(void) { return 12.0f; }
float Vbat_GetBoardVoltage(void) { return 11.9f; }
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *c) { (void)port; (void)c; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *d, uint16_t n, uint32_t t)
{
    char line[800];
    (void)u; (void)t;
    if (n == 36u && d[0] == 0xa5 && d[1] == 0x5a) {
        if (d[3] == 1u) live_count++;
        if (d[3] == 2u) {
            assert(AppPidTest_Status()->state != PID_TEST_RUNNING);
            cache_count++;
        }
    } else {
        assert(n < sizeof(line)); memcpy(line, d, n); line[n] = 0;
        if (strstr(line, "PID_ARMED")) capture_armed = 1;
        if (strstr(line, "PID_USED")) used_count++;
        if (strstr(line, "PID_END")) cache_end = 1;
        if (strstr(line, "#BTLOG,END")) {
#if ULOG_PID_PARKED_UPLOAD_ENABLE != 0
            assert(cache_end);
#endif
            end_count++;
        }
    }
    return HAL_OK;
}
void Mecanum_SetBodySpeed(MecanumChassis_t *c, float vx, float vy, float w)
{ assert(irq); c->vx_target=vx; c->vy_target=vy; c->omega_target=w; c->mode=0; c->body_frame=1; commands++; }
void Mecanum_Stop(MecanumChassis_t *c)
{ c->vx_target=c->vy_target=c->omega_target=0; c->mode=0; }
void Motor_Stop(Motor_t *m)
{ m->target_cmd=m->target_speed=m->output=0; stops++; }

static void reset_module(void)
{
    chassis.motor_lf=&motor_left_front; chassis.motor_rf=&motor_right_front;
    chassis.motor_lb=&motor_left_back; chassis.motor_rb=&motor_right_back;
    chassis.max_speed=.20f; chassis.enable=1;
    Mecanum_Stop(&chassis);
    commands=stops=0;
    AppPidTest_Init(&chassis);
}
static void check_stage(unsigned step)
{
    /* Independent expected rows, covering every block and all three cycles. */
    static const float expected[3][6] = {
        {APP_PID_TEST_LOW_MPS,APP_PID_TEST_MID_MPS,APP_PID_TEST_LOW_MPS,APP_PID_TEST_HIGH_MPS,APP_PID_TEST_MID_MPS,APP_PID_TEST_HIGH_MPS},
        {APP_PID_TEST_MID_MPS,APP_PID_TEST_LOW_MPS,APP_PID_TEST_HIGH_MPS,APP_PID_TEST_LOW_MPS,APP_PID_TEST_MID_MPS,APP_PID_TEST_HIGH_MPS},
        {APP_PID_TEST_HIGH_MPS,APP_PID_TEST_MID_MPS,APP_PID_TEST_LOW_MPS,APP_PID_TEST_MID_MPS,APP_PID_TEST_HIGH_MPS,APP_PID_TEST_LOW_MPS}};
    float v = 0;
    if (APP_PID_TEST_CREEP) {
        if (step > 0 && step < 13)
            v = step % 4 == 1 ? APP_PID_TEST_SPEED_MPS : step % 4 == 3 ? -APP_PID_TEST_SPEED_MPS : 0;
    } else if (step > 0 && step < 55) {
        v = APP_PID_TEST_DIRECTION * expected[((step-1)/6)%3][(step-1)%6];
    }
    assert(fabsf(chassis.vx_target-(APP_PID_TEST_AXIS == 0 ? v : 0)) < 1e-7f);
    assert(fabsf(chassis.vy_target-(APP_PID_TEST_AXIS == 1 ? v : 0)) < 1e-7f);
    assert(chassis.mode == 0 && chassis.body_frame && chassis.omega_target == 0);
}
static void poll_step(uint32_t ms)
{
    for (uint32_t i=0; i<ms; ++i) {
        ULogPidSample p = {0};
        now++; frames++;
        PidFixture_Poll(); PidFixture_Snapshot(); PidFixture_Observe();
        p.timestamp_ms=now; ULog_PidBurstSampleIsr(&p);
        ULog_PidBurstPoll(); ULog_Poll(&chassis);
    }
}
int main(int argc, char **argv)
{
    double displacement = 0;
    uint32_t start, expected = APP_PID_TEST_CREEP ? 32000u : 56000u;
    const char *scenario = argc > 1 ? argv[1] : "unit";
    reset_module();
#if defined(APP_PID_TEST_INVALID)
    assert(AppPidTest_Status()->reason == PID_TEST_REASON_CONFIG && !AppPidTest_Start(0));
    puts("PASS invalid level configuration rejected"); return 0;
#endif
    assert(AppPidTest_Status()->state == PID_TEST_IDLE && !commands && !stops);
    for (now=0; now<10000; now++) AppPidTest_Poll(now, PID_TEST_REASON_NONE);
    assert(AppPidTest_Status()->state == PID_TEST_IDLE && !commands);
    start = now = 0xfffffff0u;
    assert(AppPidTest_Start(now) && !AppPidTest_Start(now));
    for (unsigned step=0; step<(APP_PID_TEST_CREEP ? 14u : 56u); ++step) {
        const unsigned final = APP_PID_TEST_CREEP ? 13u : 55u;
        static const unsigned holds[6]={500,1000,1500,500,1000,1500};
        uint32_t duration = step == 0 || step == final ? 1000 :
            APP_PID_TEST_CREEP ? (step % 2 ? 4000 : 1000) : holds[(step-1)%6];
        check_stage(step);
        displacement += (chassis.vx_target + chassis.vy_target) * duration / 1000.0;
        AppPidTest_Poll(now+duration-1, PID_TEST_REASON_NONE);
        assert(AppPidTest_Status()->step == step && commands == step+1 && !stops);
        now+=duration; AppPidTest_Poll(now, PID_TEST_REASON_NONE);
    }
    if (!APP_PID_TEST_CREEP)
        assert(fabs(displacement - APP_PID_TEST_DIRECTION *
            (19.5 * APP_PID_TEST_LOW_MPS + 15.0 * APP_PID_TEST_MID_MPS + 19.5 * APP_PID_TEST_HIGH_MPS)) < 1e-6);
    assert(now-start == expected && commands == (APP_PID_TEST_CREEP ? 14u : 56u) && stops == 4);
    assert(AppPidTest_Status()->state == PID_TEST_DONE && !AppPidTest_Start(now));
    AppPidTest_Stop(PID_TEST_REASON_MANUAL);
    assert(AppPidTest_Status()->reason == PID_TEST_REASON_COMPLETE);
    for (unsigned reason=PID_TEST_REASON_MANUAL; reason<=PID_TEST_REASON_CAPTURE; ++reason) {
        reset_module(); assert(AppPidTest_Start(now));
        AppPidTest_Poll(now+1000, PID_TEST_REASON_NONE); check_stage(1);
        AppPidTest_Stop((AppPidTestReason)reason);
        assert(stops == 4 && chassis.vx_target == 0 && chassis.vy_target == 0);
        assert(AppPidTest_Status()->reason == (AppPidTestReason)reason && !AppPidTest_Start(now));
    }
    reset_module(); assert(AppPidTest_Start(now));
    AppPidTest_Poll(now+(APP_PID_TEST_CREEP ? 40000 : 70000), PID_TEST_REASON_NONE);
    assert(AppPidTest_Status()->state == PID_TEST_TIMEOUT && stops == 4);
    reset_module(); assert(AppPidTest_Start(now));
    AppPidTest_Poll(now+5000, PID_TEST_REASON_NONE);
    assert(AppPidTest_Status()->step == 1 && commands == 2);
    AppPidTest_Poll(now+5001, PID_TEST_REASON_NONE);
    assert(AppPidTest_Status()->step == 1); /* delayed polls cannot skip holds */
    reset_module(); assert(AppPidTest_Start(now));
    AppPidTest_Poll(now+1, PID_TEST_REASON_IMU);
    assert(AppPidTest_Status()->state == PID_TEST_FAULT && stops == 4);
    chassis.max_speed=.001f; AppPidTest_Init(&chassis);
    assert(AppPidTest_Status()->reason == PID_TEST_REASON_CONFIG && !AppPidTest_Start(now));
    if (!strcmp(scenario,"unit")) { puts("PASS PID sequence: axes/profile/signs/durations/once/rollover/cancel/timeout/fault/config"); return 0; }

    now=0; reset_module(); frames=1;
    if (!strcmp(scenario,"bootheld")) key=1;
    PidFixture_Init(); ULog_SetPidHandler(AppRuntime_PidAccess); ULog_Init();
    poll_step(19000); /* warmup, no button; not auto-started */
    assert(AppPidTest_Status()->state == PID_TEST_IDLE && !commands);
    key=1; poll_step(30); key=0; poll_step(30); /* without capture rejected */
    assert(AppPidTest_Status()->state == PID_TEST_IDLE);
    ULog_TestRx("@PID START\n"); poll_step(30);
    assert(capture_armed && ULog_PidBurstActive());
    key=1; poll_step(30); key=0; poll_step(30);
    assert(AppPidTest_Status()->state == PID_TEST_RUNNING);
    {
        ULogPidRequest req={0}; ULogPidValues val={0}; req.write=1; req.kp=2.5f;
        assert(!strcmp(AppRuntime_PidAccess(&req,&val),"NOT_PARKED_500MS"));
    }
    if (!strcmp(scenario,"flow") || !strcmp(scenario,"bootheld") || !strcmp(scenario,"rerun")) {
        poll_step(expected);
        assert(AppPidTest_Status()->state == PID_TEST_DONE);
        assert(!cache_count); /* less than 500ms true parked */
    } else if (!strcmp(scenario,"cancel")) {
        key=1; poll_step(30);
        assert(AppPidTest_Status()->state == PID_TEST_CANCELLED);
    } else if (!strcmp(scenario,"watchdog")) {
        now+=250; assert(!AppRuntime_ControlTick()); assert(stops==4);
        PidFixture_Poll();
        assert(AppPidTest_Status()->reason == PID_TEST_REASON_LOOP);
        assert(AppRuntime_ControlTick()); /* feedback keeps refreshing after stop */
    } else if (!strcmp(scenario,"imu")) {
        imu_ok=0; poll_step(1);
        assert(AppPidTest_Status()->reason == PID_TEST_REASON_IMU);
    } else if (!strcmp(scenario,"feedback")) {
        motor_left_front.actual_speed=NAN; poll_step(1);
        assert(AppPidTest_Status()->reason == PID_TEST_REASON_FEEDBACK);
        assert(!cache_count); motor_left_front.actual_speed=0;
    } else if (!strcmp(scenario,"capture")) {
        ULog_TestRx("@PID STOP\n"); poll_step(10);
        assert(AppPidTest_Status()->reason == PID_TEST_REASON_CAPTURE);
    } else if (!strcmp(scenario,"stale")) {
        key=1; poll_step(30); key=0; poll_step(30); /* cancel for terminal test */
        assert(AppPidTest_Status()->state == PID_TEST_CANCELLED);
        motor_left_front.actual_speed=100; poll_step(600);
#if ULOG_PID_PARKED_UPLOAD_ENABLE != 0
        assert(!cache_count && !end_count);
#else
        /* No parked transfer remains to gate END; a terminal cancellation
         * can close the live stream immediately even if feedback is stale. */
        assert(cache_count == 0u && end_count == 1u);
#endif
        motor_left_front.actual_speed=0;
    } else assert(0);
    assert(AppPidTest_Status()->state != PID_TEST_RUNNING);
    poll_step(600);
    for (unsigned i=0; i<2200; ++i) ULog_PidBurstPoll();
#if ULOG_PID_PARKED_UPLOAD_ENABLE != 0
    assert(cache_count && cache_count<=2000 && cache_end && end_count==1);
#else
    /* The production path archives the continuous live stream in the
     * browser and must finish without a parked-cache transfer. */
    assert(cache_count == 0u && !cache_end && end_count == 1u);
#endif
    assert(used_count==7 && live_count && !ULog_IsRecording());
    {
        ULogPidRequest req={0}; ULogPidValues val={0};
        assert(!strcmp(AppRuntime_PidAccess(&req,&val),"OK"));
        assert(val.editable == (AppPidTest_Status()->state==PID_TEST_DONE));
    }
    if (!strcmp(scenario,"rerun")) {
        /* A terminal session closes the live stream.  The browser's RESET
         * command must reopen it before the next START, and a stale capture
         * must not make the second START silently disappear. */
        capture_armed = 0u;
        ULog_TestRx("@PID TEST_RESET 41\n");
        poll_step(10);
        assert(AppPidTest_Status()->state == PID_TEST_IDLE);
        assert(ULog_IsRecording());
        ULog_TestRx("@PID START\n");
        poll_step(10);
        assert(ULog_PidBurstActive() && capture_armed);
        ULog_TestRx("@PID TEST 42 0\n");
        poll_step(10);
        assert(AppPidTest_Status()->state == PID_TEST_RUNNING);
        ULog_TestRx("@PID TEST_STOP 43\n");
        poll_step(10);
        assert(AppPidTest_Status()->state == PID_TEST_CANCELLED);
    }
    key=0; poll_step(30); key=1; poll_step(30); key=0; poll_step(30);
    assert(AppPidTest_Status()->state != PID_TEST_RUNNING);
    puts("PASS PID runtime: physical key/capture interlock/real parked/500ms/upload/END/PID_USED/access/watchdog");
    return 0;
}
#else
#include "board.h"
#include "bsp_ulog.h"
#include "app_runtime.h"
#include "app_race.h"
#include "app_square.h"
#include "app_mode.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
Motor_t motor_left_front,motor_right_front,motor_left_back,motor_right_back;
MecanumChassis_t chassis;
volatile uint8_t g_tune_mode;
void ULog_TestObserve(void);
static uint32_t test_irq;
uint32_t __get_PRIMASK(void) { return test_irq; }
void __disable_irq(void) { test_irq=1u; }
void __set_PRIMASK(uint32_t mask) { test_irq=mask; }
static uint32_t now, writes;
static uint8_t finished, square = SQUARE_RUNNING;
static uint8_t terminal_outputs_stopped;
#if APP_RUN_MODE != APP_MODE_RACE
static uint8_t square_seen_running;
#endif
static int tx_error, init_error, format_error;
int ulog_test_snprintf(char *buf, size_t size, const char *format, ...)
{
    int n; va_list args;
    if (format_error) return (int)size;
    va_start(args, format); n=vsnprintf(buf,size,format,args); va_end(args); return n;
}
static RaceFsm race;
static RaceTaskPlan plan;
static char last[800];
static FILE *capture;
static uint32_t live_frames, stored_frames;
#if defined(ULOG_ASYNC_TEST)
static unsigned param_sets;
static const char *async_params(const ULogPidRequest *r,ULogPidValues *v)
{
    if(r->write) {param_sets++;}
    v->revision=param_sets;v->kp=2.5f;v->ki=1.6f;v->editable=1;return "OK";
}
#endif
static uint32_t first_stored_ms, last_stored_ms;
uint32_t HAL_GetTick(void) { return now; }
float app_imu_yaw_deg(void) { return 0; }
uint8_t AppRace_LogFinished(void) { return finished; }
const RaceFsm *AppRace_Status(void) { return &race; }
const RaceTaskPlan *AppRace_TaskPlan(void) { return &plan; }
uint8_t Square_GetState(void) { return square; }
uint8_t Square_GetStep(void) { return 0; }
float Vbat_GetDriverVoltage(void) { return 12.0f; }
float Vbat_GetBoardVoltage(void) { return 11.9f; }
static void sync_snapshot(void)
{
    ULogAppSnapshot snapshot = {0};
    snapshot.imu_yaw_deg = 0.0f;
    snapshot.terminal_outputs_stopped = terminal_outputs_stopped;
#if APP_RUN_MODE == APP_MODE_RACE
    snapshot.status = (uint8_t)race.state;
    snapshot.action_index = race.action_index;
    snapshot.error = (uint8_t)race.error;
    snapshot.plan_index = plan.index;
#if ULOG_ENABLE != 0
    snapshot.plan_done = RaceTaskPlan_Done(&plan);
#endif
    snapshot.finished = finished;
    snapshot.phase = finished ? 4U :
        ((race.state == RACE_RUNNING) ? 1U : 0U);
#else
    snapshot.status = square;
    if (square == SQUARE_RUNNING) square_seen_running = 1U;
    snapshot.finished = (square == SQUARE_DONE || square == SQUARE_TIMEOUT ||
                         (square_seen_running && square == SQUARE_IDLE));
    snapshot.phase = snapshot.finished ? 4U :
        ((square == SQUARE_RUNNING) ? 1U : 0U);
#endif
    ULog_SetAppSnapshot(&snapshot);
}
void HAL_GPIO_Init(void *port, GPIO_InitTypeDef *config) { (void)port; (void)config; }
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *uart) { (void)uart; return init_error; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len, uint32_t timeout)
{
    assert(uart->Init.BaudRate == ULOG_BAUD_RATE && timeout >= 30 && len < sizeof(last));
    memcpy(last, data, len); last[len] = 0; writes++;
    if (capture) {
        assert(fwrite(data,1,len,capture)==len);
        if (len==36 && data[0]==0xa5 && data[1]==0x5a) {
            if (data[3]==1) live_frames++;
            if (data[3]==2) {
                uint32_t ms=(uint32_t)data[6]|(uint32_t)data[7]<<8|
                            (uint32_t)data[8]<<16|(uint32_t)data[9]<<24;
                if (!stored_frames) first_stored_ms=ms;
                else assert(ms==last_stored_ms+10u);
                last_stored_ms=ms; stored_frames++;
            }
        }
    }
    return tx_error;
}
#if defined(ULOG_ASYNC_TEST)
static uint8_t *pending_data;
static uint16_t pending_len;
static uint8_t tx_busy_once;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    (void)uart;
    if (tx_busy_once) { tx_busy_once=0u; return HAL_BUSY; }
    assert(!pending_len);
    pending_data=data; pending_len=len;
    return HAL_OK;
}
static void complete_tx(void)
{
    if (pending_len) {
        assert(fwrite(pending_data,1,pending_len,capture)==pending_len);
        pending_len=0;
        ULog_TxCpltCallback();
    }
}
#endif
#if ULOG_ENABLE && ULOG_PID_BURST_ENABLE
static void command(const char *text)
{
    ULog_TestRx(text);for(unsigned i=0;i<10;i++) ULog_PidBurstPoll();
}
#endif
int main(int argc, char **argv)
{
    Motor_t motors[4] = {{0}};
    MecanumChassis_t c = {0};
    uint32_t count;
    int commas = 0;
    const char *p;
    const char *scenario = argc > 1 ? argv[1] : "normal";
    c.motor_lf=&motors[0]; c.motor_rf=&motors[1]; c.motor_lb=&motors[2]; c.motor_rb=&motors[3];
    race.state=RACE_RUNNING; race.action_count=1;
    if (!strcmp(scenario,"init_error")) init_error=HAL_ERROR;
    ULog_Init();
    sync_snapshot();
#if ULOG_ENABLE == 0
    assert(!ULog_IsRecording()); ULog_Poll(&c); ULog_Printf("x"); assert(!writes); return 0;
#endif
    if (init_error) { ULog_Poll(&c); ULog_Printf("x"); assert(!writes && !ULog_IsRecording()); return 0; }
    assert(ULog_IsRecording());
#if ULOG_ENABLE && ULOG_PID_BURST_ENABLE
    if (!strcmp(scenario,"params")) {
        ULogPidRequest request={0};ULogPidValues v={0};
        Motor_t *m[4]={&motor_left_front,&motor_right_front,&motor_left_back,&motor_right_back};
        PosPID_t *pos[3]={&chassis.pid_x,&chassis.pid_y,&chassis.pid_theta};
        capture=fopen("tmp/pid_params_wire.bin","wb");assert(capture);
        ULog_SetPidHandler(AppRuntime_PidAccess);
        race.state=RACE_READY;square=SQUARE_IDLE;now=2000;
        for(unsigned i=0;i<4;i++){m[i]->kp=2.5f;m[i]->ki=1.6f;m[i]->output_max=1000;m[i]->integral_max=800;}
        for(unsigned i=0;i<3;i++){pos[i]->kp=i==2?3.2f:1.6f;pos[i]->ki=i==2?.08f:.05f;pos[i]->kd=i==2?.18f:.12f;}
        ULog_TestObserve();now+=499;
        command("@PID SET 1 0 FL 3 1.7 0.01\n");assert(strstr(last,"NOT_PARKED_500MS"));assert(m[0]->kp==2.5f);
        now++;ULog_TestObserve();
        command("@PID GET 2 FL\n");assert(strstr(last,"PID_PARAM,2,0,FL,READ,2.5"));
        command("@PID SET 3 0 FL 3 1.7 0.01\n");assert(m[0]->kp==3.0f&&m[1]->kp==2.5f);assert(strstr(last,"PID_PARAM,3,1,FL,SET,3"));
        command("@PID SET 3 0 FL 4 1 0\n");assert(strstr(last,"STALE_REVISION"));assert(m[0]->kp==3.0f);
        command("@PID SET 4 1 FL nan 1 0\n");assert(strstr(last,"MALFORMED"));
        command("@PID SET 5 1 FL 11 1 0\n");assert(strstr(last,"RANGE"));
        command("@PID SET 6 1 FL -1 1 0\n");assert(strstr(last,"RANGE"));
        command("@PID SET 7 1 FL 3 1 0 junk\n");assert(strstr(last,"MALFORMED"));
        command("@PID GET 8 FLjunk\n");assert(strstr(last,"MALFORMED"));
        command("@PID GET 4294967296 FL\n");assert(strstr(last,"MALFORMED"));
        ULog_TestRx("@PID GET 9 ");ULog_PidBurstPoll();command("FR\n");assert(strstr(last,"PID_PARAM,9,1,FR,READ"));
        command("@PID SET 10 1 TH 3.5 0.09 0.2\n");assert(chassis.pid_theta.kp==3.5f);assert(chassis.pid_x.kp==1.6f);
        command("@PID GET 11 TH\n");assert(strstr(last,"PID_PARAM,11,2,TH,READ,3.5"));
        m[0]->target_cmd=1;ULog_TestObserve();command("@PID SET 12 2 FL 4 1 0\n");assert(strstr(last,"NOT_PARKED"));m[0]->target_cmd=0;
        m[0]->output=.1f;ULog_TestObserve();command("@PID SET 13 2 FL 4 1 0\n");assert(strstr(last,"NOT_PARKED"));m[0]->output=0;
        m[0]->actual_speed=NAN;ULog_TestObserve();command("@PID SET 14 2 FL 4 1 0\n");assert(strstr(last,"NOT_PARKED"));m[0]->actual_speed=0;
        race.state=RACE_RUNNING;square=SQUARE_RUNNING;ULog_TestObserve();command("@PID SET 15 2 FL 4 1 0\n");assert(strstr(last,"NOT_PARKED"));
        command("@PID GET 16 FL\n");assert(strstr(last,"PID_PARAM,16,2,FL,READ"));
        race.state=RACE_READY;square=SQUARE_IDLE;ULog_TestObserve();now+=500;ULog_TestObserve();
        ULog_PidBurstSetParked(1);command("@PID START\n");assert(ULog_PidBurstActive());
        for(int i=0;i<300;i++){ULogPidSample sample={0};sample.fl_target=100;sample.fl_actual=90;sample.timestamp_ms=++now;ULog_PidBurstSampleIsr(&sample);ULog_PidBurstPoll();}
        command("@PID SET 17 2 FL 4 1 0\n");assert(strstr(last,"CAPTURE_BUSY"));
        ULog_PidBurstStop();now+=500;for(int i=0;i<50;i++)ULog_PidBurstPoll();
        ULog_Stop();assert(!ULog_IsRecording());
        command("@PID GET 18 FL\n");assert(strstr(last,"PID_PARAM,18,2,FL,READ"));assert(!ULog_IsRecording());
        command("@PID SET 19 2 FL 2.8 1.5 0\n");assert(m[0]->kp==2.8f);assert(strstr(last,"PID_PARAM,19,3,FL,SET"));assert(!ULog_IsRecording());
        command("@PID CONFIG 20 3 FL 5 700 900 120 0.4\n");assert(strstr(last,"PID_PARAM,20,4,FL,SET,"));assert(m[0]->speed_window_ms==5 && m[0]->integral_max==700 && m[0]->output_max==900 && m[0]->deadzone==120 && m[0]->ff_gain==0.4f);
        command("@PID CONFIG 21 4 FL 0 700 900 120 0.4\n");assert(strstr(last,"PID_ERROR,21,RANGE"));
        command("@PID CONFIG 22 4 X 5 700 900 120 0.4\n");assert(strstr(last,"PID_ERROR,22,AXIS"));
        /* Exercise each remaining controller and check atomic IRQ restoration. */
        { const char *names[7]={"FL","FR","RL","RR","X","Y","TH"};
          for(unsigned axis=1;axis<6;axis++){
            char line[96];request.axis=(uint8_t)axis;request.write=0;
            assert(!strcmp(AppRuntime_PidAccess(&request,&v),"OK"));
            snprintf(line,sizeof(line),"@PID SET %u %lu %s 1.9 0.1 0.02\n",20u+axis,(unsigned long)v.revision,names[axis]);command(line);
            request.write=0;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"OK"));assert(v.kp==1.9f);
          }
        }
        request.axis=0;request.write=0;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"OK"));
        request.write=1;request.revision=v.revision;request.kp=2.5f;request.ki=1.6f;request.kd=0;
        chassis.mode=1;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"NOT_PARKED_500MS"));chassis.mode=0;
        g_tune_mode=1;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"NOT_PARKED_500MS"));g_tune_mode=0;
        race.state=RACE_FAULT;square=SQUARE_TIMEOUT;terminal_outputs_stopped=1;sync_snapshot();
        assert(!strcmp(AppRuntime_PidAccess(&request,&v),"NOT_PARKED_500MS"));
        race.state=RACE_READY;square=SQUARE_IDLE;
        /* Overlong/malformed receive lines must not modify parameters. */
        { char line[120];memset(line,'9',sizeof(line));line[119]=0;ULog_TestRx(line);for(int i=0;i<5;i++)ULog_PidBurstPoll();command("\n@PID GET 30 FL\n");assert(strstr(last,"PID_PARAM,30,")); }
        request.axis=7;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"AXIS"));
        request.axis=0;request.write=1;request.revision=3;request.kp=INFINITY;assert(!strcmp(AppRuntime_PidAccess(&request,&v),"RANGE"));
        assert(test_irq==0u);
        fclose(capture);capture=0;
        puts("PASS online PID: actual runtime/FF constants, 7 axes, atomic parked SET, version/replay/range/fragments, capture busy, commands after END");return 0;
    }
#endif
    if (!strcmp(scenario,"burst_fault") || !strcmp(scenario,"burst_abort")) {
        ULogPidSample sample={0};
        capture=fopen("tmp/pid_terminal_wire.bin","wb"); assert(capture);
        now=ULOG_STARTUP_WAIT_MS;
        ULog_PidBurstSetParked(1);
        ULog_PidBurstStart(); assert(ULog_PidBurstActive());
        ULog_PidBurstSetParked(0);
        for (int i=0;i<100;i++) {
            now++; sample.timestamp_ms=now;
            sample.fl_actual=motors[0].actual_speed=300.0f;
            ULog_PidBurstSampleIsr(&sample);
        }
        /* Even a zero-output signal must not bypass the parked gate before
         * the log is finished and its final stop has been requested. */
        ULog_PidBurstStop();
        terminal_outputs_stopped=1; sync_snapshot(); ULog_PidBurstPoll();
        now+=1000; ULog_PidBurstPoll(); assert(!stored_frames);
        race.state=!strcmp(scenario,"burst_fault") ? RACE_FAULT : RACE_ABORTED;
        finished=1; square=SQUARE_DONE;
        terminal_outputs_stopped=0;
        sync_snapshot(); ULog_Poll(&c);
        assert(!ULog_IsRecording() && !ULog_PidBurstActive());
        /* No bypass until the application confirms zero targets/outputs. */
        now+=1000; ULog_PidBurstPoll(); assert(!stored_frames);
        terminal_outputs_stopped=1; sync_snapshot(); ULog_PidBurstPoll();
        now+=499; ULog_PidBurstPoll(); assert(!stored_frames);
        /* A renewed output revokes permission and resets the 500ms wait. */
        terminal_outputs_stopped=0; sync_snapshot(); ULog_PidBurstPoll();
        now+=1000; ULog_PidBurstPoll(); assert(!stored_frames);
        terminal_outputs_stopped=1; sync_snapshot(); ULog_PidBurstPoll();
        now+=499; ULog_PidBurstPoll(); assert(!stored_frames);
        now++;
        for (int i=0;i<20;i++) ULog_PidBurstPoll();
        assert(stored_frames==10);
        assert(strstr(last,"#BTLOG,END"));
        assert(motors[0].actual_speed==300.0f); /* Never fabricate zero speed. */
        ULog_PidBurstStart(); assert(!ULog_PidBurstActive());
        fclose(capture); capture=0;
        printf("PASS frozen speed / terminal outputs / 500ms / final END: %s\n",scenario);
        return 0;
    }
#if defined(ULOG_ASYNC_TEST)
    if (!strcmp(scenario,"params_async")) {
        char fill[1000];memset(fill,' ',sizeof(fill));
        capture=fopen("tmp/pid_params_async.bin","wb");assert(capture);
        ULog_SetPidHandler(async_params);
        for(int i=0;i<4;i++)ULog_WriteRaw((uint8_t *)fill,sizeof(fill));
        /* The first retry after a completed transfer reports HAL_BUSY;
         * the queued parameter reply must remain intact for the next poll. */
        tx_busy_once=1u;
        ULog_TestRx("@PID SET 1 0 FL 2.5 1.6 0\n");for(int i=0;i<5;i++)ULog_PidBurstPoll();assert(param_sets==1);
        for(int i=0;i<40;i++) {ULog_PidBurstPoll();}
        assert(param_sets==1);
        while(pending_len) {complete_tx();}
        for(int i=0;i<10;i++){ULog_PidBurstPoll();while(pending_len)complete_tx();}
        assert(param_sets==1);fclose(capture);capture=0;
        puts("PASS online PID async backpressure: SET applies once, response retained/retried");return 0;
    }
    if (!strcmp(scenario,"async")) {
        ULogPidSample sample={0};
        uint32_t i, before;
        now=ULOG_STARTUP_WAIT_MS;
        capture=fopen("tmp/pid_async_wire.bin","wb"); assert(capture);
        ULog_PidBurstSetParked(1);
        ULog_PidBurstStart();
        before=g_ulog_tx_error_count;
        ULog_PidBurstSetParked(0);
        for(i=1;i<=20000;i++){
            sample.timestamp_ms=++now;
            sample.fl_target=100.0f; sample.fl_actual=-50.0f;
            ULog_PidBurstSampleIsr(&sample);
            ULog_PidBurstPoll(); /* deliberately no TX completion for 20s */
        }
        assert(ULog_PidBurstActive()); /* a full cache must not stop live */
        assert(g_ulog_pid_realtime_dropped>0);
        assert(g_ulog_tx_error_count==before); /* no queue-full error flood */
        ULog_Stop();
        ULog_PidBurstSetParked(1);
        ULog_PidBurstPoll(); now+=500;
        for(i=0;i<5000;i++){ complete_tx(); ULog_PidBurstPoll(); }
        while(pending_len)complete_tx();
        assert(!ULog_IsRecording());
        assert(g_ulog_tx_error_count==before);
        fclose(capture); capture=0;
        puts("PASS async queue saturation: 20s RAM retained, live drops visible, parked upload retries");
        return 0;
    }
#endif
    if (!strncmp(scenario,"burst",5)) {
        ULogPidSample sample={0};
        uint32_t i;
        uint32_t ticks=!strcmp(scenario,"burst_short")?1000u:
                       !strcmp(scenario,"burst_continuous")?360030u:20000u;
        uint8_t live=!strcmp(scenario,"burst_live") || !strcmp(scenario,"burst_continuous");
        now=ULOG_STARTUP_WAIT_MS;
        ULog_PidBurstStart(); assert(!ULog_PidBurstActive()); /* parked gate */
        ULog_PidBurstSetParked(1);
        capture=fopen(ticks>20000u?"tmp/pid_continuous_wire.bin":"tmp/pid_wire_test.bin","wb"); assert(capture);
        ULog_PidBurstStart(); assert(ULog_PidBurstActive());
        ULog_PidBurstStart(); /* rearm must not reset capture */
        ULog_PidBurstSetParked(0);
        sample.fl_target=123.4f; sample.fl_actual=-23.4f; sample.fl_output=1000.0f;
        sample.fr_target=-123.4f; sample.rr_output=-1000.0f;
        for (i=1;i<=ticks;i++) {
            now=ULOG_STARTUP_WAIT_MS+i; sample.timestamp_ms=now;
            ULog_PidBurstSampleIsr(&sample);
            if (live) ULog_PidBurstPoll();
            if (ticks>20000u && i%50u==0u) ULog_Poll(&c);
        }
        assert(ULog_PidBurstActive()); ULog_PidBurstStop();
        assert(!ULog_PidBurstActive());
        assert(!g_ulog_pid_burst_overrun);
        for (i=0;i<20;i++) { now++; ULog_PidBurstPoll(); }
        assert(stored_frames==0); /* no upload while running */
        ULog_Stop(); assert(!ULog_IsRecording());
        ULog_PidBurstSetParked(1);
        ULog_PidBurstPoll();
        now+=499; ULog_PidBurstPoll(); assert(stored_frames==0);
        now++;
        for (i=0;i<2010;i++) ULog_PidBurstPoll();
        assert(stored_frames==(ticks>20000u?2000u:ticks/10u));
        assert(last_stored_ms==ULOG_STARTUP_WAIT_MS+ticks);
        assert(first_stored_ms==last_stored_ms-(stored_frames-1u)*10u);
        assert(strstr(last,"#BTLOG,END"));
        if (live) { assert(live_frames==ticks/5u); assert(!g_ulog_pid_realtime_dropped); }
        else { assert(live_frames==1); assert(g_ulog_pid_realtime_dropped==ticks/5u-1u); }
        fclose(capture); capture=0;
        printf("PASS 20s burst / parked gate / retained upload: %s\n",scenario);
        return 0;
    }
    ULog_Poll(&c); assert(!writes);
    if (!strcmp(scenario,"early")) { finished=1; square=SQUARE_DONE; }
    sync_snapshot();
    if (finished) {
        ULog_Poll(&c); assert(!writes && !ULog_IsRecording()); return 0;
    }
    now=ULOG_STARTUP_WAIT_MS;
    /* Real low-speed manoeuvres must survive the CSV boundary, with signs
     * and the same fractional wheel precision on all four channels. */
    c.vx_actual=0.1234f; c.vy_actual=-0.0678f;
    c.vx_target=-0.0456f; c.vy_target=0.0891f;
    motors[0].target_speed=101.2f; motors[1].target_speed=-102.3f;
    motors[2].target_speed=103.4f; motors[3].target_speed=-104.5f;
    motors[0].actual_speed=91.2f; motors[1].actual_speed=-92.3f;
    motors[2].actual_speed=93.4f; motors[3].actual_speed=-94.5f;
    ULog_Poll(&c); assert(writes == 1);
    for (p=last; *p; p++) if (*p==',') commas++;
    assert(commas == 42 && strstr(last,"\r\n"));
    {
        float fields[43];
        char *end;
        p=last;
        for (int i=0;i<43;i++) {
            fields[i]=strtof(p,&end);
            assert(end!=p);
            if (i<42) assert(*end==',');
            p=end+1;
        }
        assert(fabsf(fields[10]-c.vx_actual)<0.00006f);
        assert(fabsf(fields[11]-c.vy_actual)<0.00006f);
        assert(fabsf(fields[12]-c.vx_target)<0.00006f);
        assert(fabsf(fields[13]-c.vy_target)<0.00006f);
        assert(fabsf(fields[27]-motors[0].target_speed)<0.051f);
        for (int i=0;i<4;i++)
            assert(fabsf(fields[28+i]-motors[i].actual_speed)<0.051f);
        for (int i=1;i<4;i++)
            assert(fabsf(fields[34+i]-motors[i].target_speed)<0.051f);
    }
    now++;
    ULog_Poll(&c); assert(writes == 1);
    finished=1; square=SQUARE_DONE;
    if (!strcmp(scenario,"abort")) square=SQUARE_IDLE;
    if (!strcmp(scenario,"timeout")) square=SQUARE_TIMEOUT;
    if (!strcmp(scenario,"tx_error")) tx_error=HAL_ERROR;
    if (!strcmp(scenario,"format_error")) format_error=1;
    if (!strcmp(scenario,"nonfinite")) c.theta_target=INFINITY;
    if (!strcmp(scenario,"manual")) ULog_Stop();
    sync_snapshot();
    if (!strcmp(scenario,"null")) ULog_Poll(0); else ULog_Poll(&c);
    assert(!ULog_IsRecording());
    if (tx_error) assert(g_ulog_tx_error_count == 1);
    if (!strcmp(scenario,"format_error")) assert(g_ulog_format_error_count == 1);
    if (!strcmp(scenario,"nonfinite")) assert(g_ulog_format_error_count == 1);
    if (!strcmp(scenario,"normal")) {
        assert(writes == 3);
        assert(!strcmp(last, "#BTLOG,END\r\n"));
    }
    count=writes;
    for (int i=0;i<1000;i++) { now+=50; ULog_Poll(&c); }
    ULog_WriteRaw((const uint8_t *)"raw",3); ULog_Printf("text");
    ULog_Init(); ULog_Poll(&c);
    assert(writes == count && !ULog_IsRecording());
    printf("PASS logger: %s\n",scenario);
    return 0;
}
#endif
