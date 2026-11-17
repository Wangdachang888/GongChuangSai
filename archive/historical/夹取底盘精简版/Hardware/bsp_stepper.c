/**
  ******************************************************************************
  * @file    bsp_stepper.c
  * @brief   D36A �����������������������ʵ��
  ******************************************************************************
  * ���ļ���ֲ�� WHEELTEC ��̨����������֤�� bsp_d36a_stepper.c��
  * ��ϸԭ��˵���� bsp_stepper.h ���ļ�ͷע�͡�
  ******************************************************************************
  */
#include "bsp_stepper.h"
#include "bsp_servo.h"
#include "tim.h"

#define STEPPER_AXIS_COUNT              2U
/* ��С��ת��� 10 ������(=100us)����ֹƵ��̫�ߵ����ж����������� */
#define STEPPER_MIN_TOGGLE_INTERVAL    10U

/* ÿ���������״̬��volatile ����Ϊ��Щ���������ж��ﱻ�޸� */
typedef struct
{
    volatile uint32_t remaining_toggles; /* ʣ�෭ת������1��=2�η�ת�� */
    volatile uint16_t toggle_interval;   /* �������η�ת�ļ������������Ƶ�ʣ� */
    volatile uint8_t  busy;              /* 1=���ڷ����� */
    volatile GPIO_PinState direction;    /* ��ǰ�����ƽ */
    volatile uint8_t enabled;            /* 1=EN high */
    volatile uint8_t profile_enabled;
    volatile uint32_t profile_total_steps;
    volatile uint32_t profile_completed_steps;
    volatile uint16_t profile_accel_steps;
    volatile uint16_t profile_decel_steps;
    volatile uint16_t profile_start_hz;
    volatile uint16_t profile_cruise_hz;
    volatile uint16_t profile_end_hz;
} StepperState_t;

static StepperState_t s_axis[STEPPER_AXIS_COUNT];

/*
 * ����/�˳��ٽ�������ѭ�����ж϶������ s_axis��
 * �޸�ʱ�����ȹ����жϣ���ֹ�ĵ�һ�뱻�жϴ��������ݴ��ҡ�
 */
static uint32_t Stepper_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Stepper_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq(); /* ֻ�н���ǰ�ж��ǿ��ŵĲ����´� */
    }
}

static uint8_t Stepper_IsValidAxis(StepperAxis_t axis)
{
    return ((axis == STEPPER_AXIS_1) ||
            (axis == STEPPER_AXIS_2)) ? 1U : 0U;
}

static uint16_t Stepper_FrequencyToInterval(uint32_t frequency_hz)
{
    uint32_t denominator = frequency_hz * 2U;
    return (uint16_t)((STEPPER_TIMER_TICK_HZ + (denominator / 2U)) /
                      denominator);
}

static void Stepper_UpdateProfileAfterPulse(StepperAxis_t axis)
{
    StepperState_t *state = &s_axis[axis];
    uint32_t next_step;
    uint32_t frequency_hz;
    uint32_t decel_index;

    if (state->profile_enabled == 0U) return;
    state->profile_completed_steps++;
    next_step = state->profile_completed_steps;
    if (next_step >= state->profile_total_steps) return;

    frequency_hz = state->profile_cruise_hz;
    if (state->profile_accel_steps > 1U &&
        next_step < state->profile_accel_steps) {
        frequency_hz = state->profile_start_hz +
            ((uint32_t)(state->profile_cruise_hz - state->profile_start_hz) *
             next_step) / (uint32_t)(state->profile_accel_steps - 1U);
    } else if (state->profile_decel_steps > 1U &&
               next_step >= state->profile_total_steps -
                            state->profile_decel_steps) {
        decel_index = next_step -
            (state->profile_total_steps - state->profile_decel_steps);
        frequency_hz = state->profile_cruise_hz -
            ((uint32_t)(state->profile_cruise_hz - state->profile_end_hz) *
             decel_index) / (uint32_t)(state->profile_decel_steps - 1U);
    }
    state->toggle_interval = Stepper_FrequencyToInterval(frequency_hz);
}

/*
 * ��������Ƚ�ģʽ��
 *   TIM_OCMODE_TOGGLE          -> ����ֵ׷��CCRʱ�����Զ���ת���������ã�
 *   TIM_OCMODE_FORCED_INACTIVE -> ǿ������͵�ƽ��ֹͣʱ�ã���֤STEPͣ�ڵͣ�
 */
static void Stepper_SetOutputMode(StepperAxis_t axis, uint32_t mode)
{
    if (axis == STEPPER_AXIS_1)
    {
        MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC1M, mode);
    }
    else
    {
        MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC2M, mode << 8U);
    }
}

/* �رո���ıȽ��жϲ�����жϱ�־ */
static void Stepper_DisableChannelInterrupt(StepperAxis_t axis)
{
    if (axis == STEPPER_AXIS_1)
    {
        CLEAR_BIT(TIM8->DIER, TIM_DIER_CC1IE);
        TIM8->SR = ~TIM_SR_CC1IF;
    }
    else
    {
        CLEAR_BIT(TIM8->DIER, TIM_DIER_CC2IE);
        TIM8->SR = ~TIM_SR_CC2IF;
    }
}

/* ֹͣһ���ᣨ�ڲ��汾������ǰ���������ٽ����ڣ� */
static void Stepper_StopUnsafe(StepperAxis_t axis)
{
    Stepper_DisableChannelInterrupt(axis);
    Stepper_SetOutputMode(axis, TIM_OCMODE_FORCED_INACTIVE);
    s_axis[axis].remaining_toggles = 0U;
    s_axis[axis].busy = 0U;
    s_axis[axis].profile_enabled = 0U;

    /* �����ᶼͣ�˲Źض�ʱ����ʡ���Ҽ��������ٿ�ת */
    if ((s_axis[STEPPER_AXIS_1].busy == 0U) &&
        (s_axis[STEPPER_AXIS_2].busy == 0U))
    {
        CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);
    }
}

void Stepper_Init(void)
{
    uint32_t primask = Stepper_EnterCritical();

    /* ��ȷ����ʱ��ֹͣ����·�Ƚ��жϹر� */
    CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);
    CLEAR_BIT(TIM8->DIER, TIM_DIER_CC1IE | TIM_DIER_CC2IE);

    /* ��·���ǿ��Ϊ�͵�ƽ��STEP����ʱ�����ǵͣ� */
    Stepper_SetOutputMode(STEPPER_AXIS_1, TIM_OCMODE_FORCED_INACTIVE);
    Stepper_SetOutputMode(STEPPER_AXIS_2, TIM_OCMODE_FORCED_INACTIVE);

    /* ʹ����·�Ƚ������TIM8�Ǹ߼���ʱ������������λ�����ʹ��MOE */
    SET_BIT(TIM8->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);

    /* DIR ��������͵�ƽ��Ϊ��ʼ���� */
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, STEPPER_EN_LEVEL_ON);
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, STEPPER_EN_LEVEL_OFF);

    s_axis[STEPPER_AXIS_1].remaining_toggles = 0U;
    s_axis[STEPPER_AXIS_1].toggle_interval = 0U;
    s_axis[STEPPER_AXIS_1].busy = 0U;
    s_axis[STEPPER_AXIS_1].direction = GPIO_PIN_RESET;
    s_axis[STEPPER_AXIS_1].enabled = 1U;
    s_axis[STEPPER_AXIS_1].profile_enabled = 0U;
    s_axis[STEPPER_AXIS_2].remaining_toggles = 0U;
    s_axis[STEPPER_AXIS_2].toggle_interval = 0U;
    s_axis[STEPPER_AXIS_2].busy = 0U;
    s_axis[STEPPER_AXIS_2].direction = GPIO_PIN_RESET;
    s_axis[STEPPER_AXIS_2].enabled = 0U;
    s_axis[STEPPER_AXIS_2].profile_enabled = 0U;

    TIM8->SR = 0U; /* ��������жϱ�־ */

    /* ʹ�� TIM8 �Ƚ��жϣ����ȼ����һ�㣬��֤�������׼�� */
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

    Stepper_ExitCritical(primask);
}


void Stepper_SetEnable(StepperAxis_t axis, uint8_t enable)
{
    GPIO_PinState level = (enable != 0U) ? STEPPER_EN_LEVEL_ON : STEPPER_EN_LEVEL_OFF;

    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return;
    }

    if (enable == 0U)
    {
        Stepper_Stop(axis);
    }

    if (axis == STEPPER_AXIS_1)
    {
        HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, level);
    }
    else
    {
        HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, level);
    }
    s_axis[axis].enabled = (enable != 0U) ? 1U : 0U;
}

void Stepper_SetEnableAll(uint8_t enable)
{
    Stepper_SetEnable(STEPPER_AXIS_1, enable);
    Stepper_SetEnable(STEPPER_AXIS_2, enable);
}

uint8_t Stepper_GetEnable(StepperAxis_t axis)
{
    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return 0U;
    }
    return s_axis[axis].enabled;
}

HAL_StatusTypeDef Stepper_SetDirection(StepperAxis_t axis,
                                       GPIO_PinState direction)
{
    uint32_t primask;

    if ((Stepper_IsValidAxis(axis) == 0U) ||
        ((direction != GPIO_PIN_RESET) && (direction != GPIO_PIN_SET)))
    {
        return HAL_ERROR;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return HAL_ERROR;
#endif

    primask = Stepper_EnterCritical();

    /* �����в������������巢��һ�뻻�����ɶ��� */
    if (s_axis[axis].busy != 0U)
    {
        Stepper_ExitCritical(primask);
        return HAL_BUSY;
    }

    if (axis == STEPPER_AXIS_1)
    {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, direction);
    }
    else
    {
        HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, direction);
    }

    s_axis[axis].direction = direction;
    Stepper_ExitCritical(primask);
    return HAL_OK;
}

GPIO_PinState Stepper_GetDirection(StepperAxis_t axis)
{
    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return GPIO_PIN_RESET;
    }

    return s_axis[axis].direction;
}

HAL_StatusTypeDef Stepper_Start(StepperAxis_t axis,
                                uint32_t steps,
                                uint32_t frequency_hz)
{
    uint32_t denominator;
    uint32_t interval;
    uint32_t primask;

    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return HAL_ERROR;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return HAL_ERROR;
#endif

    if (s_axis[axis].enabled == 0U)
    {
        return HAL_ERROR; /* EN sleep */
    }

    if (steps == 0U)
    {
        Stepper_Stop(axis);
        return HAL_OK;
    }

    if ((frequency_hz < STEPPER_MIN_FREQ_HZ) ||
        (frequency_hz > STEPPER_MAX_FREQ_HZ) ||
        (steps > (UINT32_MAX / 2U)))
    {
        return HAL_ERROR;
    }

    /*
     * ���㷭ת�����
     * 1������ = �ߵ�ƽ+�͵�ƽ = 2�η�ת�����Է�תƵ�� = 2*����Ƶ�ʣ�
     * ��� = ����Ƶ�� / (2*����Ƶ��)���� denominator/2 ���������롣
     */
    denominator = frequency_hz * 2U;
    interval = (STEPPER_TIMER_TICK_HZ + (denominator / 2U)) / denominator;

    if ((interval < STEPPER_MIN_TOGGLE_INTERVAL) ||
        (interval > UINT16_MAX))
    {
        return HAL_ERROR;
    }

    primask = Stepper_EnterCritical();

    if (s_axis[axis].busy != 0U)
    {
        Stepper_ExitCritical(primask);
        return HAL_BUSY; /* ��һ�����廹û���� */
    }

    Stepper_DisableChannelInterrupt(axis);
    Stepper_SetOutputMode(axis, TIM_OCMODE_FORCED_INACTIVE);

    s_axis[axis].toggle_interval = (uint16_t)interval;
    s_axis[axis].remaining_toggles = steps * 2U;
    s_axis[axis].busy = 1U;
    s_axis[axis].profile_enabled = 0U;

    /*
     * �� CCR �赽"��ǰ����ֵ+һ�����"����Ȼ���е���תģʽ�����жϡ�
     * ����ֵ׷�� CCR ʱ�����һ�η�ת��STEP��ߣ����ж�������ƽ���
     */
    if (axis == STEPPER_AXIS_1)
    {
        TIM8->CCR1 = (uint16_t)(TIM8->CNT + interval);
        TIM8->SR = ~TIM_SR_CC1IF;
        Stepper_SetOutputMode(axis, TIM_OCMODE_TOGGLE);
        SET_BIT(TIM8->CCER, TIM_CCER_CC1E);
        SET_BIT(TIM8->DIER, TIM_DIER_CC1IE);
    }
    else
    {
        TIM8->CCR2 = (uint16_t)(TIM8->CNT + interval);
        TIM8->SR = ~TIM_SR_CC2IF;
        Stepper_SetOutputMode(axis, TIM_OCMODE_TOGGLE);
        SET_BIT(TIM8->CCER, TIM_CCER_CC2E);
        SET_BIT(TIM8->DIER, TIM_DIER_CC2IE);
    }

    SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);
    SET_BIT(TIM8->CR1, TIM_CR1_CEN); /* ������ʱ�� */

    Stepper_ExitCritical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef Stepper_StartRamp(StepperAxis_t axis,
                                    uint32_t steps,
                                    uint32_t start_frequency_hz,
                                    uint32_t cruise_frequency_hz,
                                    uint32_t end_frequency_hz,
                                    uint16_t accel_steps,
                                    uint16_t decel_steps)
{
    HAL_StatusTypeDef status;
    uint32_t primask;

    if (Stepper_IsValidAxis(axis) == 0U || steps == 0U ||
        start_frequency_hz < STEPPER_MIN_FREQ_HZ ||
        cruise_frequency_hz > STEPPER_MAX_FREQ_HZ ||
        end_frequency_hz < STEPPER_MIN_FREQ_HZ ||
        start_frequency_hz > cruise_frequency_hz ||
        end_frequency_hz > cruise_frequency_hz ||
        accel_steps < 2U || decel_steps < 2U ||
        (uint32_t)accel_steps + (uint32_t)decel_steps > steps) {
        return HAL_ERROR;
    }

    primask = Stepper_EnterCritical();
    status = Stepper_Start(axis, steps, start_frequency_hz);
    if (status == HAL_OK) {
        s_axis[axis].profile_total_steps = steps;
        s_axis[axis].profile_completed_steps = 0U;
        s_axis[axis].profile_accel_steps = accel_steps;
        s_axis[axis].profile_decel_steps = decel_steps;
        s_axis[axis].profile_start_hz = (uint16_t)start_frequency_hz;
        s_axis[axis].profile_cruise_hz = (uint16_t)cruise_frequency_hz;
        s_axis[axis].profile_end_hz = (uint16_t)end_frequency_hz;
        s_axis[axis].profile_enabled = 1U;
    }
    Stepper_ExitCritical(primask);
    return status;
}

HAL_StatusTypeDef Stepper_Feed(StepperAxis_t axis,
                               uint32_t steps,
                               uint32_t frequency_hz)
{
    uint32_t denominator;
    uint32_t interval;
    uint32_t primask;

    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return HAL_ERROR;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return HAL_ERROR;
#endif

    if (steps == 0U)
    {
        return HAL_OK;
    }

    if ((frequency_hz < STEPPER_MIN_FREQ_HZ) ||
        (frequency_hz > STEPPER_MAX_FREQ_HZ) ||
        (steps > (UINT32_MAX / 4U)))
    {
        return HAL_ERROR;
    }

    denominator = frequency_hz * 2U;
    interval = (STEPPER_TIMER_TICK_HZ + (denominator / 2U)) / denominator;

    if ((interval < STEPPER_MIN_TOGGLE_INTERVAL) ||
        (interval > UINT16_MAX))
    {
        return HAL_ERROR;
    }

    primask = Stepper_EnterCritical();

    if (s_axis[axis].busy != 0U)
    {
        /*
         * ���ỹ�ڷ����壺ֱ�Ӱѷ�ת��������ȥ��ÿ��2�Σ���ż����
         * ��֤���崮����ʱSTEP��ͣ�ڵ͵�ƽ������Ƶ�ʴ���һ�η�ת��Ч��
         * �����¾���������֮��û���κμ�϶��
         */
        s_axis[axis].remaining_toggles += steps * 2U;
        s_axis[axis].toggle_interval = (uint16_t)interval;
        Stepper_ExitCritical(primask);
        return HAL_OK;
    }

    Stepper_ExitCritical(primask);
    /* ������ͣ������ͨ��ʽ����һ�������� */
    return Stepper_Start(axis, steps, frequency_hz);
}

void Stepper_Stop(StepperAxis_t axis)
{
    uint32_t primask;

    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return;
#endif

    primask = Stepper_EnterCritical();
    Stepper_StopUnsafe(axis);
    Stepper_ExitCritical(primask);
}

void Stepper_StopAll(void)
{
    uint32_t primask = Stepper_EnterCritical();
    Stepper_StopUnsafe(STEPPER_AXIS_1);
#if !STEPPER_AXIS_2_RESERVED_FOR_SERVO
    Stepper_StopUnsafe(STEPPER_AXIS_2);
#endif
    Stepper_ExitCritical(primask);
}

uint8_t Stepper_IsBusy(StepperAxis_t axis)
{
    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return 0U;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return 0U;
#endif

    return s_axis[axis].busy;
}

uint32_t Stepper_GetRemainingSteps(StepperAxis_t axis)
{
    uint32_t toggles;

    if (Stepper_IsValidAxis(axis) == 0U)
    {
        return 0U;
    }

#if STEPPER_AXIS_2_RESERVED_FOR_SERVO
    if (axis == STEPPER_AXIS_2) return 0U;
#endif

    toggles = s_axis[axis].remaining_toggles;
    return (toggles + 1U) / 2U;
}

/**
  * TIM8 �Ƚ��жϴ�����ÿ�����ŷ�ת��������
  * ��������ת������һ -> û����Ͱ�CCR������һ����� -> �����ͣ���ᡣ
  * �������� stm32f4xx_it.c �е� TIM8_CC_IRQHandler() ���á�
  */
void Stepper_TIM8_CC_IRQHandler(void)
{
    uint32_t status = TIM8->SR;
    uint32_t interrupt_enable = TIM8->DIER;

    /* ---------- ��1��ͨ��1�� ---------- */
    if (((status & TIM_SR_CC1IF) != 0U) &&
        ((interrupt_enable & TIM_DIER_CC1IE) != 0U))
    {
        TIM8->SR = ~TIM_SR_CC1IF; /* ���жϱ�־ */

        if (s_axis[STEPPER_AXIS_1].remaining_toggles > 0U)
        {
            s_axis[STEPPER_AXIS_1].remaining_toggles--;
        }

        if (s_axis[STEPPER_AXIS_1].remaining_toggles == 0U)
        {
            Stepper_StopUnsafe(STEPPER_AXIS_1); /* �����ˣ��Զ�ͣ */
        }
        else
        {
            uint16_t next;
            if ((s_axis[STEPPER_AXIS_1].remaining_toggles & 1U) == 0U) {
                Stepper_UpdateProfileAfterPulse(STEPPER_AXIS_1);
            }
            /* ����һ�η�ת��������һ�������16λ��Ȼ���ƣ��������⴦���� */
            next = (uint16_t)(TIM8->CCR1 +
                              s_axis[STEPPER_AXIS_1].toggle_interval);
            if ((int16_t)(next - (uint16_t)TIM8->CNT) <= 0) {
                next = (uint16_t)((uint16_t)TIM8->CNT +
                                 s_axis[STEPPER_AXIS_1].toggle_interval);
            }
            TIM8->CCR1 = next;
        }
    }

    /* ---------- PC7 SERVO_3 timing ---------- */
    if (((status & TIM_SR_CC2IF) != 0U) &&
        ((interrupt_enable & TIM_DIER_CC2IE) != 0U))
    {
        TIM8->SR = ~TIM_SR_CC2IF;
        Servo_PC7_TIM8_CC_IRQHandler();
    }

    /* ���ᶼ����ʱ�ص���ʱ�� */
    if ((s_axis[STEPPER_AXIS_1].busy == 0U) &&
        (s_axis[STEPPER_AXIS_2].busy == 0U) &&
        ((TIM8->DIER & TIM_DIER_CC2IE) == 0U))
    {
        CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);
    }
}
