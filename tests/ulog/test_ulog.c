#include "board.h"
#include "bsp_ulog.h"
#include "app_race.h"
#include "app_square.h"
#include "app_mode.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
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
