#include "mecanum_chassis.h"
#include <stdlib.h>
#include "nav.h"

/* ��ϵ���ʵ�����������̣���̬���䣩 */
static Nav_t   s_nav;
static uint8_t s_nav_ready = 0;

Nav_t *MecanumChassis_GetNav(void)
{
    return &s_nav;
}

/************************ �ڲ����ߺ��� ************************/
/**
 * @brief  �Ƕȹ�һ���� [-��, ��]
 */
static float Angle_Normalize(float angle)
{
    while (angle >  PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief  λ��PID����
 */
static float PosPID_Calc(PosPID_t *pid, float target, float actual, float dt)
{
    pid->err = target - actual;

    pid->integral += pid->err * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    float derivative = (pid->err - pid->err_last) / dt;
    pid->err_last = pid->err;

    pid->output = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * derivative;
    if (pid->output >  pid->output_max) pid->output =  pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    return pid->output;
}

/* yaw/heading PID: wrap error to (-pi, pi], avoid +/-180 discontinuity */
static float PosPID_CalcAngle(PosPID_t *pid, float target, float actual, float dt)
{
    pid->err = Angle_Normalize(target - actual);

    pid->integral += pid->err * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    float derivative = Angle_Normalize(pid->err - pid->err_last) / dt;
    pid->err_last = pid->err;

    pid->output = pid->kp * pid->err + pid->ki * pid->integral + pid->kd * derivative;
    if (pid->output >  pid->output_max) pid->output =  pid->output_max;
    if (pid->output < -pid->output_max) pid->output = -pid->output_max;

    return pid->output;
}

/************************ �˶�ѧ���� ************************/
/**
 * @brief  ���˶�ѧ�������ٶ� �� ���������ٶ�
 */
static void Mecanum_InverseKinematics(MecanumChassis_t *chassis,
                                      float vx, float vy, float omega,
                                      float *wheel_pulse)
{
    float L = chassis->L;
    float m2p = chassis->meter_to_pulse;

    float v_lf = (vx - vy - L * omega) ;
    float v_rf = (vx + vy + L * omega) ;
    float v_lb = (vx + vy - L * omega) ;
    float v_rb = (vx - vy + L * omega) ;

    wheel_pulse[0] = v_lf * m2p;
    wheel_pulse[1] = v_rf * m2p;
    wheel_pulse[2] = v_lb * m2p;
    wheel_pulse[3] = v_rb * m2p;
}

/**
 * @brief  ���˶�ѧ�����������ٶ� �� �����ٶ�
 */
static void Mecanum_ForwardKinematics(MecanumChassis_t *chassis,
                                      float *wheel_pulse,
                                      float *vx, float *vy, float *omega)
{
    float p2m = chassis->pulse_to_meter;
    float L = chassis->L;

    float v_lf = wheel_pulse[0] * p2m;
    float v_rf = wheel_pulse[1] * p2m;
    float v_lb = wheel_pulse[2] * p2m;
    float v_rb = wheel_pulse[3] * p2m;


	
	*vx = (v_lf + v_rf + v_lb + v_rb) / 4.0f;  // �������ȡ������ĵ��0/1/2/3�ֱ��Ӧ˭
	*vy = (-v_lf + v_rf + v_lb - v_rb) / 4.0f;
	*omega = (-v_lf + v_rf - v_lb + v_rb) / (4.0f * L);
}

/************************ ״̬���� ************************/
/**
* @brief  ���µ���ʵ��״̬���ٶ�+λ�ã��������㣩����������ϵת��Ϊ��������ϵ��
 */
//static void Mecanum_UpdateState(MecanumChassis_t *chassis, float dt)
//{
//    float wheel_pulse[4];

//    wheel_pulse[0] = Motor_GetSpeed(chassis->motor_lf);
//    wheel_pulse[1] = Motor_GetSpeed(chassis->motor_rf);
//    wheel_pulse[2] = Motor_GetSpeed(chassis->motor_lb);
//    wheel_pulse[3] = Motor_GetSpeed(chassis->motor_rb);

//    Mecanum_ForwardKinematics(chassis, wheel_pulse,
//                              &chassis->vx_actual, &chassis->vy_actual, &chassis->omega_actual);

//    chassis->x_pos += chassis->vx_actual * dt;
//    chassis->y_pos += chassis->vy_actual * dt;
//    chassis->theta_pos += chassis->omega_actual * dt;
//    chassis->theta_pos = Angle_Normalize(chassis->theta_pos);
//}

static void Mecanum_UpdateState(MecanumChassis_t *chassis, float dt)
{
    float wheel_pulse[4];

    wheel_pulse[0] = Motor_GetSpeed(chassis->motor_lf);
    wheel_pulse[1] = Motor_GetSpeed(chassis->motor_rf);
    wheel_pulse[2] = Motor_GetSpeed(chassis->motor_lb);
    wheel_pulse[3] = Motor_GetSpeed(chassis->motor_rb);

    /* ���˶�ѧ���õ�������ϵ���ٶ� */
    Mecanum_ForwardKinematics(chassis, wheel_pulse,
                              &chassis->vx_actual, &chassis->vy_actual, &chassis->omega_actual);

    /* ��ϵ�����
     *   ���� = IMU������ + �����ƣ���������ʽ���ٶȵ�Ȩ�ظ���
     *   ƽ�� = �������˶�ѧ�����ںϺ�ĺ���ͶӰ������ϵ
     *   ��ֹ -> ��������������λ�ˣ��� -> ֻ�� IMU ����˥��ƽ��
     */
    if (!s_nav_ready) {
        Nav_Init(&s_nav, chassis->theta_pos);
        s_nav_ready = 1;
    }
    Nav_Update(&s_nav, dt, chassis->vx_actual, chassis->vy_actual,
               chassis->omega_actual, &chassis->imu_snapshot);
    Nav_GetPose(&s_nav, &chassis->x_pos, &chassis->y_pos, &chassis->theta_pos);
}
/************************ λ�û��ջ� ************************/
/**
 * @brief  λ�û�PID����
 */
static void Mecanum_PositionLoop(MecanumChassis_t *chassis, float dt)
{
    float err_x, err_y, err_theta;
    float dist;
    float spd_lim;

    if (chassis->mode != 1) return;

    err_x = chassis->x_target - chassis->x_pos;
    err_y = chassis->y_target - chassis->y_pos;
    err_theta = Angle_Normalize(chassis->theta_target - chassis->theta_pos);

    /* Near target: freeze XY integral to avoid push-through overshoot */
    dist = sqrtf(err_x * err_x + err_y * err_y);
    if (dist < 0.25f) {
        chassis->pid_x.integral = 0.0f;
        chassis->pid_y.integral = 0.0f;
    }
    /* Only clear yaw I inside tol; clearing at 0.20rad killed lock for 5~7deg errors */
    if (fabsf(err_theta) < (chassis->theta_tol * 2.0f)) {
        chassis->pid_theta.integral = 0.0f;
    }

    chassis->vx_target    = PosPID_Calc(&chassis->pid_x, chassis->x_target, chassis->x_pos, dt);
    chassis->vy_target    = PosPID_Calc(&chassis->pid_y, chassis->y_target, chassis->y_pos, dt);
    chassis->omega_target = PosPID_CalcAngle(&chassis->pid_theta, chassis->theta_target, chassis->theta_pos, dt);

    /* Physics brake + yaw-first translate */
    {
        const float a_lin = 0.50f;
        const float a_ang = 1.80f;
        const float creep = 0.05f;
        float brake;
        float w_lim;
        float spd_now;
        float w_now;
        float yaw_scale;
        float ae;

        spd_lim = chassis->max_speed;
        brake = sqrtf(2.0f * a_lin * dist);
        if (brake < spd_lim) {
            spd_lim = brake;
        }
        if ((dist > chassis->position_tol) && (spd_lim < creep)) {
            spd_lim = creep;
        }
        if (dist <= chassis->position_tol) {
            spd_lim = 0.0f;
        }

        /* If heading off >2deg, cut translation to avoid crab-walk */
        ae = fabsf(err_theta);
        if (ae <= 0.035f) {
            yaw_scale = 1.0f;
        } else if (ae >= 0.18f) {
            yaw_scale = 0.20f;
        } else {
            yaw_scale = 1.0f - (ae - 0.035f) * (0.80f / 0.145f);
        }
        spd_lim *= yaw_scale;

        w_lim = chassis->max_omega;
        brake = sqrtf(2.0f * a_ang * fabsf(err_theta));
        if (brake < w_lim) {
            w_lim = brake;
        }
        if ((fabsf(err_theta) > chassis->theta_tol) && (w_lim < 0.25f)) {
            w_lim = 0.25f;
        }
        if (fabsf(err_theta) <= chassis->theta_tol) {
            w_lim = 0.0f;
        }

        if (chassis->vx_target >  spd_lim) chassis->vx_target =  spd_lim;
        if (chassis->vx_target < -spd_lim) chassis->vx_target = -spd_lim;
        if (chassis->vy_target >  spd_lim) chassis->vy_target =  spd_lim;
        if (chassis->vy_target < -spd_lim) chassis->vy_target = -spd_lim;
        if (chassis->omega_target >  w_lim) chassis->omega_target =  w_lim;
        if (chassis->omega_target < -w_lim) chassis->omega_target = -w_lim;

        spd_now = sqrtf(chassis->vx_actual * chassis->vx_actual +
                        chassis->vy_actual * chassis->vy_actual);
        w_now = fabsf(chassis->omega_actual);

        if (fabsf(err_x) < chassis->position_tol &&
            fabsf(err_y) < chassis->position_tol &&
            fabsf(err_theta) < chassis->theta_tol &&
            spd_now < 0.06f &&
            w_now < 0.12f)
        {
            chassis->target_reached = 1;
            chassis->vx_target = 0.0f;
            chassis->vy_target = 0.0f;
            chassis->omega_target = 0.0f;
            chassis->pid_x.integral = 0.0f;
            chassis->pid_y.integral = 0.0f;
            chassis->pid_theta.integral = 0.0f;
        }
        else
        {
            chassis->target_reached = 0;
        }
    }
}



/************************ ����ӿ� ************************/
/**
 * @brief  ���̳�ʼ��
 */
void MecanumChassis_Init(MecanumChassis_t *chassis,
                         Motor_t *lf, Motor_t *rf, Motor_t *lb, Motor_t *rb,
                       float wheel_radius, 
					   float wheel_base_half, 
					   float track_width_half,
                       float gear_ratio,   
					   float enc_ppr_x4)
//wheel_radius			���Ӱ뾶						m			���߲������� 50mm ���� = 0.05f
//wheel_base_half		����ࣨǰ�������ľ��һ�룩	m			����ǰ�������ľ��� �� 2
//track_width_half		���־ࣨ���������ľ��һ�룩	m			�������������ľ��� �� 2
//gear_ratio			���ٱ�							��			�������飬�� 1:30 ������ = 30.0f
//enc_ppr_x4			������ 4 ��Ƶÿת������			��			���������� �� 4���� 500 �� = 2000.0f
							 
{
    /* �󶨵�� */
    chassis->motor_lf = lf;
    chassis->motor_rf = rf;
    chassis->motor_lb = lb;
    chassis->motor_rb = rb;

    /* ��е���� */
    chassis->wheel_radius = wheel_radius;
    chassis->wheel_base_half = wheel_base_half;
    chassis->track_width_half = track_width_half;
    chassis->L = wheel_base_half + track_width_half;
    chassis->gear_ratio = gear_ratio;
    chassis->enc_ppr_x4 = enc_ppr_x4;

    /* ����ת��ϵ�� */
    float wheel_perimeter = 2.0f * PI * wheel_radius;
    float pulse_per_wheel = enc_ppr_x4 * gear_ratio;
    chassis->pulse_to_meter = wheel_perimeter / pulse_per_wheel;
    chassis->meter_to_pulse = pulse_per_wheel / wheel_perimeter;

    /* ״̬���� */
    chassis->x_pos = 0.0f;
    chassis->y_pos = 0.0f;
    chassis->theta_pos = 0.0f;
    chassis->vx_actual = 0.0f;
    chassis->vy_actual = 0.0f;
    chassis->omega_actual = 0.0f;
    chassis->imu_snapshot.yaw_rad = 0.0f;
    chassis->imu_snapshot.valid = 0u;
    chassis->vx_target = 0.0f;
    chassis->vy_target = 0.0f;
    chassis->omega_target = 0.0f;

    /* Ĭ�ϲ��� */
    chassis->max_speed = 1.0f;
    chassis->max_omega = PI;
    chassis->position_tol = 0.01f;
    chassis->theta_tol = 0.05f;

    chassis->enable = 1;
    chassis->mode = 0;
    chassis->body_frame = 0;
    chassis->target_reached = 0;

    Nav_Init(&s_nav, 0.0f);
    s_nav_ready = 1;
}

/**
* @brief  ����������ѭ��������10ms����һ�Σ������³�Ϊ��������ϵ��
 */
//void MecanumChassis_Loop(MecanumChassis_t *chassis, float dt)
//{
//    if (!chassis->enable)
//    {
//        Motor_Stop(chassis->motor_lf);
//        Motor_Stop(chassis->motor_rf);
//        Motor_Stop(chassis->motor_lb);
//        Motor_Stop(chassis->motor_rb);
//        return;
//    }

//    /* 1. ����ʵ��״̬ */
//    Mecanum_UpdateState(chassis, dt);

//    /* 2. λ��ģʽ������λ�û� */
//    if (chassis->mode == 1)
//    {
//        Mecanum_PositionLoop(chassis, dt);
//    }

//    /* 3. ���˶�ѧ���� */
//    float wheel_target[4];
//    Mecanum_InverseKinematics(chassis,
//                              chassis->vx_target, chassis->vy_target, chassis->omega_target,
//                              wheel_target);

//    /* 4. �·�Ŀ���ٶȵ���� */
//    Motor_SetTargetSpeed(chassis->motor_lf, wheel_target[0]);
//    Motor_SetTargetSpeed(chassis->motor_rf, wheel_target[1]);
//    Motor_SetTargetSpeed(chassis->motor_lb, wheel_target[2]);
//    Motor_SetTargetSpeed(chassis->motor_rb, wheel_target[3]);
//}
void MecanumChassis_Loop(MecanumChassis_t *chassis, float dt)
{
    if (!chassis->enable)
    {
        Motor_Stop(chassis->motor_lf);
        Motor_Stop(chassis->motor_rf);
        Motor_Stop(chassis->motor_lb);
        Motor_Stop(chassis->motor_rb);
        return;
    }

    /* 1. ����ʵ��״̬���ڲ���������ϵ������ϵ�� */
    Mecanum_UpdateState(chassis, dt);

    /* 2. λ��ģʽ������λ�û�
     *    PositionLoop ����� vx_target/vy_target �ǡ�����ϵ�� */
    if (chassis->mode == 1)
    {
        Mecanum_PositionLoop(chassis, dt);
    }

    /* �� 3. ����任������ϵ �� ����ϵ������ϵģʽֱ��͸�������������� */
    float vx_body;
    float vy_body;

    if (chassis->body_frame) {
        vx_body = chassis->vx_target;
        vy_body = chassis->vy_target;
    } else {
        float cos_t = cosf(chassis->theta_pos);
        float sin_t = sinf(chassis->theta_pos);

        vx_body =  chassis->vx_target * cos_t + chassis->vy_target * sin_t;
        vy_body = -chassis->vx_target * sin_t + chassis->vy_target * cos_t;
    }

    /* 4. ���˶�ѧ���ó���ϵ�ٶ� */
    float wheel_target[4];
    Mecanum_InverseKinematics(chassis, vx_body, vy_body,
                              chassis->omega_target, wheel_target);

    /* 5. �·�Ŀ���ٶ� */
    Motor_SetTargetSpeed(chassis->motor_lf, wheel_target[0]);
    Motor_SetTargetSpeed(chassis->motor_rf, wheel_target[1]);
    Motor_SetTargetSpeed(chassis->motor_lb, wheel_target[2]);
    Motor_SetTargetSpeed(chassis->motor_rb, wheel_target[3]);
}
/************************ ģʽ�л����˶��ӿ� ************************/
void Mecanum_SetSpeedMode(MecanumChassis_t *chassis, float vx, float vy, float omega)
{
    chassis->mode = 0;
    chassis->body_frame = 0;   /* ����ϵ */
    chassis->target_reached = 0;

    if (vx >  chassis->max_speed) vx =  chassis->max_speed;
    if (vx < -chassis->max_speed) vx = -chassis->max_speed;
    if (vy >  chassis->max_speed) vy =  chassis->max_speed;
    if (vy < -chassis->max_speed) vy = -chassis->max_speed;
    if (omega >  chassis->max_omega) omega =  chassis->max_omega;
    if (omega < -chassis->max_omega) omega = -chassis->max_omega;

    chassis->vx_target = vx;
    chassis->vy_target = vy;
    chassis->omega_target = omega;
}

/**
 * @brief  ����ϵ�ٶ�ģʽ��vx/vy �Ѿ��ǳ��巽��ѭ���ﲻ�ٰ�������ת
 * @note   �ڻ����������� IMU ��������ǰ���ٶȷֽ�ɺ������
 */
void Mecanum_SetBodySpeed(MecanumChassis_t *chassis, float vx_body, float vy_body, float omega)
{
    chassis->mode = 0;
    chassis->body_frame = 1;
    chassis->target_reached = 0;

    if (vx_body >  chassis->max_speed) vx_body =  chassis->max_speed;
    if (vx_body < -chassis->max_speed) vx_body = -chassis->max_speed;
    if (vy_body >  chassis->max_speed) vy_body =  chassis->max_speed;
    if (vy_body < -chassis->max_speed) vy_body = -chassis->max_speed;
    if (omega  >  chassis->max_omega) omega  =  chassis->max_omega;
    if (omega  < -chassis->max_omega) omega  = -chassis->max_omega;

    chassis->vx_target = vx_body;
    chassis->vy_target = vy_body;
    chassis->omega_target = omega;
}

void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float theta)
{
    chassis->mode = 1;
    chassis->target_reached = 0;
    chassis->x_target = x;
    chassis->y_target = y;
    chassis->theta_target = Angle_Normalize(theta);

    /* �л�ģʽ��ջ��֣���ֹ��� */
    chassis->pid_x.err_last = 0.0f;
    chassis->pid_x.integral = 0.0f;
    chassis->pid_y.integral = 0.0f;
    chassis->pid_theta.integral = 0.0f;
}

void Mecanum_MoveForward(MecanumChassis_t *chassis, float speed)
{
    Mecanum_SetSpeedMode(chassis, speed, 0.0f, 0.0f);
}

void Mecanum_MoveBackward(MecanumChassis_t *chassis, float speed)
{
    Mecanum_SetSpeedMode(chassis, -speed, 0.0f, 0.0f);
}

void Mecanum_MoveLeft(MecanumChassis_t *chassis, float speed)
{
    Mecanum_SetSpeedMode(chassis, 0.0f, speed, 0.0f);
}

void Mecanum_MoveRight(MecanumChassis_t *chassis, float speed)
{
    Mecanum_SetSpeedMode(chassis, 0.0f, -speed, 0.0f);
}

void Mecanum_RotateCCW(MecanumChassis_t *chassis, float omega)
{
    Mecanum_SetSpeedMode(chassis, 0.0f, 0.0f, omega);
}

void Mecanum_RotateCW(MecanumChassis_t *chassis, float omega)
{
    Mecanum_SetSpeedMode(chassis, 0.0f, 0.0f, -omega);
}

void Mecanum_Stop(MecanumChassis_t *chassis)
{
    Mecanum_SetSpeedMode(chassis, 0.0f, 0.0f, 0.0f);
}


/* ==================== �����ӿں���ʵ�� ==================== */

/**
 * @brief  ͳһ�˶��ӿڣ��ٶ�ģʽ
 * @param  vx     ǰ���ٶȣ�m/s����=ǰ
 * @param  vy     �����ٶȣ�m/s����=��
 * @param  omega  ��ת���ٶȣ�rad/s����=��ʱ��
 */
void Mecanum_Move(MecanumChassis_t *chassis, float vx, float vy, float omega)
{
    Mecanum_SetSpeedMode(chassis, vx, vy, omega);
}

/**
 * @brief  ��ȡ��ǰλ��
 */
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{
    if (x)     *x     = chassis->x_pos;
    if (y)     *y     = chassis->y_pos;
    if (theta) *theta = chassis->theta_pos;
}

/**
 * @brief  ��ȡ��ǰ�ٶ�
 */
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{
    if (vx)    *vx    = chassis->vx_actual;
    if (vy)    *vy    = chassis->vy_actual;
    if (omega) *omega = chassis->omega_actual;
}

/**
 * @brief  �Ƿ񵽴�Ŀ��λ�ã�λ��ģʽ����Ч��
 */
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis)
{
    return chassis->target_reached;
}

/**
 * @brief  ʹ�� / ʧ�ܵ���
 * @param  enable: 1=ʹ�ܣ�0=ʧ�ܣ�ʧ�ܺ���ֹͣ��
 */
void Mecanum_Enable(MecanumChassis_t *chassis, uint8_t enable)
{
    chassis->enable = enable;
    if (!enable)
    {
        Mecanum_Stop(chassis);
    }
}

/**
 * @brief  ����ǰλ�����㣨��Ϊ�µ�ԭ�㣩
 */
void Mecanum_ResetPosition(MecanumChassis_t *chassis)
{
    chassis->x_pos = 0.0f;
    chassis->y_pos = 0.0f;
    chassis->theta_pos = 0.0f;
    Nav_Reset(&s_nav, 0.0f, 0.0f, 0.0f);
}

/**
 * @brief  ����������ٶȺͽ��ٶ�
 */
void MecanumChassis_SetImuSnapshot(MecanumChassis_t *chassis,
                                   const NavImuSnapshot_t *snapshot)
{
    uint32_t primask;

    if ((chassis == NULL) || (snapshot == NULL)) {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    chassis->imu_snapshot = *snapshot;
    if (primask == 0u) {
        __enable_irq();
    }
}
void Mecanum_SetSpeedLimit(MecanumChassis_t *chassis, float max_speed, float max_omega)
{
    chassis->max_speed = max_speed;
    chassis->max_omega = max_omega;
}

/**
 * @brief  ���õ�λ�ݲ�
 */
void Mecanum_SetTolerance(MecanumChassis_t *chassis, float position_tol, float theta_tol)
{
    chassis->position_tol = position_tol;
    chassis->theta_tol = theta_tol;
}

/**
 * @brief  һ��������λ�û����� PID ����
 */
void Mecanum_SetPositionPID(MecanumChassis_t *chassis,
                            float kp, float ki, float kd,
                            float integral_max, float output_max)
{
    chassis->pid_x.kp = kp;
    chassis->pid_x.ki = ki;
    chassis->pid_x.kd = kd;
    chassis->pid_x.integral_max = integral_max;
    chassis->pid_x.output_max = output_max;

    chassis->pid_y.kp = kp;
    chassis->pid_y.ki = ki;
    chassis->pid_y.kd = kd;
    chassis->pid_y.integral_max = integral_max;
    chassis->pid_y.output_max = output_max;

    chassis->pid_theta.kp = kp;
    chassis->pid_theta.ki = ki;
    chassis->pid_theta.kd = kd;
    chassis->pid_theta.integral_max = integral_max;
    chassis->pid_theta.output_max = output_max;
}
/* �ڲ�������������������Ҫ�ô�Χ��ʾ�����ܸ��������ܳ� 5 λ�� */
