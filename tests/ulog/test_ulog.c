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
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    (void)uart;
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
