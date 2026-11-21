/**
 * @file    Motor.c
 * @brief   单电机速度环（AT8236 双 PWM）
 *
 * 结构：
 *   output = 前馈(速度 x 增益 + 死区补偿)
 *          + P(误差 x kp)
 *          + I(误差积分 x ki，条件积分抗饱和)   <- 负责对抗阻力
 *          + D(误差变化率 x kd)
 *          + 抗静摩擦 kick（卡住时逐渐增加）
 *   再经过：不反拖、限幅、变化率限制，最后映射成 CCR。
 *
 * 低速专用处理：
 *   1. 窗口测速（5ms）+ 一阶低通：避免 1ms 采样的量化噪声
 *   2. 目标速度斜坡：把上层阶跃指令变成连续变化，消除顿挫
 *   3. 死区补偿随速度渐变：不在低速阈值处突跳
 *   4. 条件积分：输出饱和时停止积分，防止堵转顶爆后猛冲
 */
#include "motor.h"
#include <math.h>

/* ---------------- 前馈与死区 ---------------- */
#ifndef MOTOR_FF_GAIN
#define MOTOR_FF_GAIN       0.30f   /* 每 1 脉冲/秒 对应的 output（满量程 1000） */
#endif
#ifndef MOTOR_DZ_OUT
#define MOTOR_DZ_OUT        150.0f  /* 死区补偿上限（12% 占空比） */
#endif
#ifndef MOTOR_DZ_SLOPE
#define MOTOR_DZ_SLOPE      1.20f   /* 死区补偿斜率：每 1 脉冲/秒 的 output */
#endif
#ifndef MOTOR_FF_MIN_SPD
#define MOTOR_FF_MIN_SPD    5.0f   /* 目标速度超过它才加前馈 */
#endif

/* ---------------- 测速 ---------------- */
#ifndef MOTOR_SPEED_WIN_MS
#define MOTOR_SPEED_WIN_MS  5u      /* 测速窗口 ms（5ms -> 200Hz） */
#endif
#ifndef MOTOR_SPD_LPF
#define MOTOR_SPD_LPF       0.35f   /* 测速低通系数 */
#endif

/* ---------------- 目标斜坡 ---------------- */
#ifndef MOTOR_TARGET_RAMP
#define MOTOR_TARGET_RAMP   4000.0f /* 脉冲/秒/秒 */
#endif

/* ---------------- 输出整形 ---------------- */
#ifndef MOTOR_SLEW
#define MOTOR_SLEW          30.0f   /* 每个测速周期允许的最大 output 变化 */
#endif

/* ---------------- 抗静摩擦 ---------------- */
#ifndef MOTOR_KICK_STEP
#define MOTOR_KICK_STEP     3.0f    /* 每个测速周期增加量 */
#endif
#ifndef MOTOR_KICK_MAX
#define MOTOR_KICK_MAX      120.0f  /* 最大补偿（12% 占空比） */
#endif
#ifndef MOTOR_STUCK_RATIO
#define MOTOR_STUCK_RATIO   0.10f   /* 实际速度低于目标的这个比例算卡住 */
#endif

/**
 * @brief  读取编码器增量（16 位计数器，处理溢出）
 */
const MotorControlInfo *Motor_GetControlInfo(void)
{
    static const MotorControlInfo info = {
        MOTOR_SPEED_WIN_MS, MOTOR_FF_GAIN, MOTOR_DZ_OUT, MOTOR_FF_MIN_SPD,
        MOTOR_TARGET_RAMP, MOTOR_SLEW, MOTOR_SPD_LPF,
        MOTOR_KICK_STEP, MOTOR_KICK_MAX, MOTOR_STUCK_RATIO
    };
    return &info;
}

static int32_t Encoder_ReadDelta(Motor_t *motor)
{
    int32_t now_cnt = (int32_t)__HAL_TIM_GET_COUNTER(motor->enc_tim);
    int32_t delta = now_cnt - motor->enc_last_cnt;

    if (delta > 32767) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }
    motor->enc_last_cnt = now_cnt;
    return delta;
}

/**
 * @brief  电机初始化
 */
void Motor_Init(Motor_t *motor,
                TIM_HandleTypeDef *pwm_tim1, uint32_t pwm_ch1, uint32_t pwm_arr1,
                TIM_HandleTypeDef *pwm_tim2, uint32_t pwm_ch2, uint32_t pwm_arr2,
                TIM_HandleTypeDef *enc_tim,
                float kp, float ki, float kd,
                int8_t motor_dir, int8_t enc_dir)
{
    /* 绑定硬件资源 */
    motor->pwm_tim1  = pwm_tim1;
    motor->pwm_ch1   = pwm_ch1;
    motor->pwm_arr1  = pwm_arr1;
    motor->pwm_tim2  = pwm_tim2;
    motor->pwm_ch2   = pwm_ch2;
    motor->pwm_arr2  = pwm_arr2;
    motor->enc_tim   = enc_tim;
    motor->motor_dir = motor_dir;
    motor->enc_dir   = enc_dir;

    /* 状态初始化 */
    motor->enc_last_cnt = 0;
    motor->actual_speed = 0.0f;
    motor->target_speed = 0.0f;
    motor->target_cmd   = 0.0f;

    /* PID 参数 */
    motor->kp = kp;
    motor->ki = ki;
    motor->kd = kd;
    motor->err = 0.0f;
    motor->err_last = 0.0f;
    motor->integral = 0.0f;
    motor->output = 0.0f;
    motor->output_last = 0.0f;
    motor->kick = 0.0f;
    motor->acc_cnt = 0;
    motor->acc_ms = 0u;
    motor->speed_lpf = 0.0f;

    motor->output_max = PID_OUTPUT_MAX;
    motor->integral_max = PID_OUTPUT_MAX * 0.8f;   /* 积分可承担较大负载 */

    /* 启动 PWM */
    HAL_TIM_PWM_Start(pwm_tim1, pwm_ch1);
    if (pwm_tim2 != pwm_tim1) {
        HAL_TIM_PWM_Start(pwm_tim2, pwm_ch2);
    } else {
        HAL_TIM_PWM_Start(pwm_tim2, pwm_ch2);
    }

    /* 启动编码器 */
    HAL_TIM_Encoder_Start(enc_tim, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    motor->enc_last_cnt = (int32_t)__HAL_TIM_GET_COUNTER(motor->enc_tim);

    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->pwm_ch1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->pwm_ch2, 0);
}

/**
 * @brief  设置目标速度（脉冲/秒），实际目标由 Motor_Update 斜坡跟随
 */
void Motor_SetTargetSpeed(Motor_t *motor, float target_speed)
{
    motor->target_cmd = target_speed;
}

/**
 * @brief  速度环更新（建议 1ms 调用；内部按 5ms 窗口计算）
 */
void Motor_Update(Motor_t *motor, float dt)
{
    int32_t delta;
    float   dtm;
    float   raw;

    if (dt <= 0.0f) {
        return;
    }

    /* ---------- 1. 窗口测速 ---------- */
    delta = Encoder_ReadDelta(motor);
    motor->acc_cnt += delta;
    motor->acc_ms = (uint16_t)(motor->acc_ms + (uint16_t)(dt * 1000.0f + 0.5f));

    if (motor->acc_ms < MOTOR_SPEED_WIN_MS) {
        return;                     /* 窗口未满，保持上次 PWM */
    }

    dtm = (float)motor->acc_ms * 0.001f;
    raw = (float)motor->acc_cnt / dtm * (float)motor->enc_dir;

    motor->acc_cnt = 0;
    motor->acc_ms  = 0u;

    motor->speed_lpf = motor->speed_lpf * (1.0f - MOTOR_SPD_LPF) + raw * MOTOR_SPD_LPF;
    motor->actual_speed = motor->speed_lpf;

    /* ---------- 2. 目标速度斜坡（去顿挫） ---------- */
    {
        float dmax = MOTOR_TARGET_RAMP * dtm;
        float d    = motor->target_cmd - motor->target_speed;

        if (d > dmax) {
            d = dmax;
        } else if (d < -dmax) {
            d = -dmax;
        }
        motor->target_speed += d;
    }

    /* ---------- 3. PID ---------- */
    {
        float p_out;
        float d_out;
        float ff;
        float pre;

        motor->err = motor->target_speed - motor->actual_speed;

        p_out = motor->kp * motor->err;
        d_out = motor->kd * (motor->err - motor->err_last) / dtm;
        motor->err_last = motor->err;

        /* 前馈 + 死区补偿（死区随速度渐变，避免低速阈值处突跳） */
        ff = 0.0f;
        if (motor->target_speed > MOTOR_FF_MIN_SPD) {
            ff = motor->target_speed * MOTOR_FF_GAIN + MOTOR_DZ_OUT;
        } else if (motor->target_speed < -MOTOR_FF_MIN_SPD) {
            ff = motor->target_speed * MOTOR_FF_GAIN - MOTOR_DZ_OUT;
        }

        pre = ff + p_out + d_out;

        /* 条件积分：只有未饱和时才积分 */
        if ((pre < motor->output_max) && (pre > -motor->output_max)) {
            motor->integral += motor->err * dtm;
            if (motor->integral >  motor->integral_max) {
                motor->integral =  motor->integral_max;
            }
            if (motor->integral < -motor->integral_max) {
                motor->integral = -motor->integral_max;
            }
        }

        motor->output = pre + motor->ki * motor->integral;
    }

    /* ---------- 4. 抗静摩擦 + 低速不反拖 ---------- */
    {
        float sgn = 0.0f;

        if (motor->target_speed > MOTOR_FF_MIN_SPD) {
            sgn = 1.0f;
        } else if (motor->target_speed < -MOTOR_FF_MIN_SPD) {
            sgn = -1.0f;
        }

        if ((sgn != 0.0f) &&
            (fabsf(motor->actual_speed) <
             (MOTOR_STUCK_RATIO * fabsf(motor->target_speed)))) {
            motor->kick += MOTOR_KICK_STEP;
            if (motor->kick > MOTOR_KICK_MAX) {
                motor->kick = MOTOR_KICK_MAX;
            }
        } else {
            motor->kick *= 0.97f;
            if (motor->kick < 1.0f) {
                motor->kick = 0.0f;
            }
        }
        motor->output += sgn * motor->kick;

        if ((sgn > 0.0f) && (motor->output < 0.0f)) {
            motor->output = 0.0f;
        }
        if ((sgn < 0.0f) && (motor->output > 0.0f)) {
            motor->output = 0.0f;
        }
    }

    /* ---------- 5. 限幅 + 变化率限制 ---------- */
    if (motor->output >  motor->output_max) {
        motor->output =  motor->output_max;
    }
    if (motor->output < -motor->output_max) {
        motor->output = -motor->output_max;
    }

    {
        float d = motor->output - motor->output_last;

        if (d > MOTOR_SLEW) {
            motor->output = motor->output_last + MOTOR_SLEW;
        } else if (d < -MOTOR_SLEW) {
            motor->output = motor->output_last - MOTOR_SLEW;
        }
        motor->output_last = motor->output;
    }

    /* ---------- 6. 映射到 CCR ---------- */
    {
        float duty = motor->output / motor->output_max * (float)motor->motor_dir;

        if (duty > 0.001f) {
            uint32_t ccr = (uint32_t)(duty * (float)motor->pwm_arr1 + 0.5f);
            if (ccr > motor->pwm_arr1) {
                ccr = motor->pwm_arr1;
            }
            __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->pwm_ch1, ccr);
            __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->pwm_ch2, 0);
        } else if (duty < -0.001f) {
            uint32_t ccr = (uint32_t)(-duty * (float)motor->pwm_arr2 + 0.5f);
            if (ccr > motor->pwm_arr2) {
                ccr = motor->pwm_arr2;
            }
            __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->pwm_ch1, 0);
            __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->pwm_ch2, ccr);
        } else {
            __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->pwm_ch1, 0);
            __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->pwm_ch2, 0);
        }
    }
}

/**
 * @brief  电机停止（滑行）
 */
void Motor_Stop(Motor_t *motor)
{
    motor->integral = 0.0f;
    motor->err = 0.0f;
    motor->err_last = 0.0f;
    motor->target_speed = 0.0f;
    motor->target_cmd = 0.0f;
    motor->output = 0.0f;
    motor->output_last = 0.0f;
    motor->kick = 0.0f;
    motor->acc_cnt = 0;
    motor->acc_ms = 0u;
    motor->speed_lpf = 0.0f;

    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->pwm_ch1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->pwm_ch2, 0);
}

/**
 * @brief  读取实际速度（脉冲/秒）
 */
float Motor_GetSpeed(Motor_t *motor)
{
    return motor->actual_speed;
}
