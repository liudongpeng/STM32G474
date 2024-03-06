/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"

#include "usbd_cdc_if.h"
#include "tim.h"
#include "adc.h"
#include "usart.h"

#include "utils.h"

#include "bsp.h"
#include "motor.h"
#include "button.h"
#include "digital_filter.h"
#include "foc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

#define CH_COUNT 32
struct Frame
{
    float fdata[CH_COUNT];
    unsigned char tail[4];
};

struct Frame vfoaFrame = {
        .tail = {0x00, 0x00, 0x80, 0x7f}
};



uint8_t vofaCmdBuf[1024];



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
};
/* Definitions for gBtnScanTask */
osThreadId_t gBtnScanTaskHandle;
const osThreadAttr_t gBtnScanTask_attributes = {
        .name = "gBtnScanTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


static void bsp_btn_event_callback(void *btn, int event);


/**
 * @brief Control motor work status
 * @param btn
 * @param event
 */
static void bsp_btn_event_callback(void *btn, int event)
{
    if (btn == &btn1)
    {
        motor1.uq += 0.5f;
        if (motor1.uq > motor1.VBus / SQRT3)
            motor1.uq = motor1.VBus / SQRT3;
    }
    else if (btn == &btn2)
    {
        motor1.uq -= 0.5f;
        if (motor1.uq < 0)
            motor1.uq = 0;
    }
    else if (btn == &btn3)
    {
        if (motor1.status != MOTOR_STATUS_RUN)
        {
            motor_align_encoder(&motor1, 0.5f, 0);
//        motor_calib_encoder(&motor1, 0.7f);
        }
    }
    else if (btn == &btn4)
    {
        if (event == ButtonEvent_SingleClick)
        {
            motor1.status = motor1.status == MOTOR_STATUS_RUN ? MOTOR_STATUS_STOP : MOTOR_STATUS_RUN;

            if (motor1.status == MOTOR_STATUS_RUN)
            {
                motor_enable(&motor1);
            }
            else
            {
                motor_disable(&motor1);
            }
        }
    }
    else
    {
    }

    led_toggle(&led2);
}


#define WIN_BUF_SIZE    50
float windowFilterBuf[WIN_BUF_SIZE];

float window_filter(float data, float *buf, int len)
{
    float sum = 0;

    for (int i = 1; i < len; i++)
    {
        buf[i - 1] = buf[i];
    }
    buf[len - 1] = data;

    for (int i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    sum /= len;

    return sum;
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void StartBtnScanTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* bsp and motor init */
    bsp_init();

    /* Install button event callback */
    button_install_event_callback(&btn1, ButtonEvent_SingleClick, bsp_btn_event_callback);
    button_install_event_callback(&btn2, ButtonEvent_SingleClick, bsp_btn_event_callback);
    button_install_event_callback(&btn3, ButtonEvent_SingleClick, bsp_btn_event_callback);
    button_install_event_callback(&btn4, ButtonEvent_SingleClick, bsp_btn_event_callback);

    /* Set motor speed. */
    motor_set_speed(&motor1, 60.0f);

    /* Measure motor phase resistance */
    while (!foc_board.isAdcCalibOvered);
//    motor_meas_phase_resistance(&motor1);
//    motor_meas_phase_resistance2(&motor1);

    /* vofa+ debug */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, vofaCmdBuf, 1024);

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of gBtnScanTask */
    gBtnScanTaskHandle = osThreadNew(StartBtnScanTask, NULL, &gBtnScanTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */

    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    /* Infinite loop */
    for (;;)
    {
        bsp_get_vbus(&foc_board, &motor1.VBus);
        //bsp_get_phase_voltage(&foc_board, &motor1.ua, &motor1.ub, NULL);
        HAL_Delay(1);

        float upLimit = motor1.VBus / SQRT3 * 0.9f;
        pid_ctrl_set_limit(&motor1.idPid, -upLimit, upLimit);
        pid_ctrl_set_limit(&motor1.iqPid, -upLimit, upLimit);

        /* Led blink. */
        static int ledCnt = 0;
        if (++ledCnt > 50)
        {
            ledCnt = 0;
            led_toggle(&led1);
        }

        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBtnScanTask */
/**
* @brief Function implementing the gBtnScanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtnScanTask */
void StartBtnScanTask(void *argument)
{
    /* USER CODE BEGIN StartBtnScanTask */
    /* Infinite loop */
    for (;;)
    {
        button_ticks();
        HAL_Delay(5);

        osDelay(1);
    }
    /* USER CODE END StartBtnScanTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



/**
  * @brief  Injected conversion complete callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* Calc ADC offset */
    static bool isCalcAdcOffsetOvered = false;
    const int measCnt = 20;
    static int measCntCopy = measCnt;
    if (hadc->Instance == ADC1 && !isCalcAdcOffsetOvered)
    {
        foc_board.iaOffset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        foc_board.ibOffset += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        if (--measCntCopy <= 0)
        {
            foc_board.iaOffset /= measCnt;
            foc_board.ibOffset /= measCnt;

            isCalcAdcOffsetOvered = true;
            foc_board.isAdcCalibOvered = true;
        }
    }

    /* Current closed loop */
    if (hadc->Instance == ADC1 && isCalcAdcOffsetOvered)
    {
        HAL_GPIO_WritePin(UserTest_GPIO_Port, UserTest_Pin, GPIO_PIN_SET);

        /* Get phase current. */
        motor1.get_phase_current(&motor1);
//        motor1.ia = lpf_work(&motor1.iaLpFilter, motor1.ia);
//        motor1.ib = lpf_work(&motor1.ibLpFilter, motor1.ib);

        /* Get motor electricity angle. */
        static bool isReadEncoder = true;
        if (isReadEncoder)
        {
            //motor_get_elec_angle(&motor1);
        }
        else
        {
            motor_get_angle_rad(&motor1);

            static bool isAngleInc = true;
            if (isAngleInc)
            {
                motor1.theta += 0.01f;
                if (motor1.theta > M_TWOPI)
                    motor1.theta -= M_TWOPI;
            }
            motor1.sinTheta = arm_sin_f32(motor1.theta);
            motor1.cosTheta = arm_cos_f32(motor1.theta);

            /* 1. clark */
            foc_clark(motor1.ia, motor1.ib, &motor1.ialpha, &motor1.ibeta);
            /* 2. park */
            foc_park(motor1.ialpha, motor1.ibeta, motor1.sinTheta, motor1.cosTheta, &motor1.id, &motor1.iq);
        }

        /* Calc speed. */
        static bool isPllCalcSpeed = true;
        if (isPllCalcSpeed)
        {
            foc_pll_calc(&motor1.speedPll, motor1.encoderRawData);
            motor1.speedRpm = motor1.speedPll.speed * 20000 * 60;
        }
        else
        {
            static int sLastEncoderRaw = 0;

//            motor1.encoderRawData = 16384 - motor1.encoderRawData;
//            int encoderRaw = GetCurrentAbsTotalValue(ENCODER_CPR - motor1.encoderRawData);

            int encoderRaw = GetCurrentAbsTotalValue(motor1.encoderRawData);
            float detRad = (float) (encoderRaw - sLastEncoderRaw) * 20000 * 60.0f / 16384;
            motor1.speedRpm = window_filter(detRad, windowFilterBuf, WIN_BUF_SIZE);
//            motor1.speedRpm = low_pass_filter_work(&motor1.speedLdFilter, detRad);

            sLastEncoderRaw = encoderRaw;
        }

        /* Motor run. */
        if (motor1.status == MOTOR_STATUS_RUN)
        {
            /* Current closed loop. */
            static bool isCurrentLoop = true;
            if (isCurrentLoop)
            {
                motor1.theta = 0;
                motor1.sinTheta = arm_sin_f32(motor1.theta);
                motor1.cosTheta = arm_cos_f32(motor1.theta);

                /* 1. Clark */
                foc_clark(motor1.ia, motor1.ib, &motor1.ialpha, &motor1.ibeta);
                /* 2. Park */
                foc_park(motor1.ialpha, motor1.ibeta, motor1.sinTheta, motor1.cosTheta, &motor1.id, &motor1.iq);

                /* 3. id, iq pid ctrl */
                const float voltage_normalize = 1.5f / motor1.VBus;
                motor1.ud *= voltage_normalize;
                motor1.uq *= voltage_normalize;
                odriver_current_pi_ctrl(&motor1, motor1.idSet, motor1.iqSet);

                /* HFI inj */
                static bool isUseHFI = true;
                static bool isDirChanged = true;
                if (isUseHFI)
                {
                    if (isDirChanged)
                    {
                        motor1.ud += motor1.hfiVoltAmpl * voltage_normalize;
                    }
                    else
                    {
                        motor1.ud -= motor1.hfiVoltAmpl * voltage_normalize;
                    }
                    isDirChanged = !isDirChanged;
                }

                /* 4. Inv park */
                foc_inv_park(motor1.ud, motor1.uq, motor1.sinTheta, motor1.cosTheta, &motor1.ualpha, &motor1.ubeta);

                /* HFI detach */
                motor1.hfiId = 0.5f * (motor1.id + motor1.hfiIdLast);
                motor1.hfiIq = 0.5f * (motor1.iq + motor1.hfiIqLast);
                motor1.hfiIdLast = motor1.id;
                motor1.hfiIqLast = motor1.iq;

                /* 5. SVM */
                int valid = odriver_svm(motor1.ualpha, motor1.ubeta, &motor1.ta, &motor1.tb, &motor1.tc, &motor1.sector);
                if (valid)
                {
                    motor_set_and_apply_pwm_duty(&motor1, motor1.ta, motor1.tb, motor1.tc);

                    /* 6. Recover ud uq */
                    motor1.ud /= voltage_normalize;
                    motor1.uq /= voltage_normalize;
                }
            }
            else
            {
                motor_open_loop_test(&motor1, motor1.ud, motor1.uq);
            }

            /* Speed closed loop. */
            static bool isSpeedLoop = !true;
            static uint32_t speedFbkCnt = 0;
            if (++speedFbkCnt >= 4 && isCurrentLoop && isSpeedLoop)
            {
                speedFbkCnt = 0; // reset counter

                /* Speed ramp */
                if (motor1.isUseSpeedRamp)
                {
                    if (motor1.speedSet > motor1.speedShadow)
                    {
                        motor1.speedShadow += motor1.speedAcc;
                        if (motor1.speedShadow > motor1.speedSet)
                            motor1.speedShadow = motor1.speedSet;
                    }
                    else if (motor1.speedSet < motor1.speedShadow)
                    {
                        motor1.speedShadow -= motor1.speedAcc;
                        if (motor1.speedShadow < motor1.speedSet)
                            motor1.speedShadow = motor1.speedSet;
                    }
                    else
                    {
                    }
                }

                motor_speed_closed_loop(&motor1, motor1.speedShadow, &motor1.iqSet);
            }

            /* Position closed loop. */
            static bool isPositionLoop = !true;
            static uint32_t posFbkCnt = 0;
            if (++posFbkCnt >= 20 && isCurrentLoop && isSpeedLoop && isPositionLoop)
            {
                posFbkCnt = 0; // reset counter

                motor_position_closed_loop(&motor1, motor1.positionSet, &motor1.speedSet);
            }
        }

        /* Vofa debug. */
        int dataIndex = 0;
        vfoaFrame.fdata[dataIndex++] = motor1.ia;
        vfoaFrame.fdata[dataIndex++] = motor1.ib;
        vfoaFrame.fdata[dataIndex++] = motor1.ialpha;
        vfoaFrame.fdata[dataIndex++] = motor1.ibeta;
        vfoaFrame.fdata[dataIndex++] = motor1.VBus;
        vfoaFrame.fdata[dataIndex++] = (float) motor1.angleRadOffset;
        vfoaFrame.fdata[dataIndex++] = motor1.theta;

        vfoaFrame.fdata[dataIndex++] = (float) motor1.ta;
        vfoaFrame.fdata[dataIndex++] = (float) motor1.tb;
        vfoaFrame.fdata[dataIndex++] = (float) motor1.tc;

        vfoaFrame.fdata[dataIndex++] = motor1.iqPid.kp;
        vfoaFrame.fdata[dataIndex++] = motor1.iqPid.ki;
        vfoaFrame.fdata[dataIndex++] = motor1.iqSet;
        vfoaFrame.fdata[dataIndex++] = motor1.iq;

        vfoaFrame.fdata[dataIndex++] = motor1.idPid.kp;
        vfoaFrame.fdata[dataIndex++] = motor1.idPid.ki;
        vfoaFrame.fdata[dataIndex++] = motor1.idSet;
        vfoaFrame.fdata[dataIndex++] = motor1.id;

        vfoaFrame.fdata[dataIndex++] = motor1.speedPid.kp;
        vfoaFrame.fdata[dataIndex++] = motor1.speedPid.ki;
        vfoaFrame.fdata[dataIndex++] = motor1.speedSet;
        vfoaFrame.fdata[dataIndex++] = motor1.speedRpm;

        vfoaFrame.fdata[dataIndex++] = motor1.positionPid.kp;
        vfoaFrame.fdata[dataIndex++] = motor1.positionPid.ki;
        vfoaFrame.fdata[dataIndex++] = motor1.positionSet;
        vfoaFrame.fdata[dataIndex++] = motor1.angle;

        vfoaFrame.fdata[dataIndex++] = motor1.uq;
        vfoaFrame.fdata[dataIndex++] = motor1.ud;
        vfoaFrame.fdata[dataIndex++] = (float) motor1.sector;
        vfoaFrame.fdata[dataIndex++] = motor1.hfiVoltAmpl;

        /* HFI */
        vfoaFrame.fdata[dataIndex++] = motor1.hfiIq;
        vfoaFrame.fdata[dataIndex++] = motor1.hfiId;

        HAL_UART_Transmit_DMA(&huart3, (uint8_t *) (&vfoaFrame), sizeof(vfoaFrame));
//        CDC_Transmit_FS((uint8_t *) (&vfoaFrame), sizeof(vfoaFrame));

        HAL_GPIO_WritePin(UserTest_GPIO_Port, UserTest_Pin, GPIO_PIN_RESET);
    }
}


/**
 * @brief Parse vofa cmd
 * @param cmdBuf
 * @return
 */
static float vofa_cmd_parse(uint8_t *cmdBuf, char *arg)
{
    return atof(cmdBuf + strlen(arg));
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        if (strstr(vofaCmdBuf, "uq"))
        {
            motor1.uq = vofa_cmd_parse(vofaCmdBuf, "uq");
        }
        else if (strstr(vofaCmdBuf, "idkp="))
        {
            float kp = vofa_cmd_parse(vofaCmdBuf, "idkp=");
            motor1.idPid.kp = kp;
        }
        else if (strstr(vofaCmdBuf, "idki="))
        {
            float ki = vofa_cmd_parse(vofaCmdBuf, "idkp=");
            motor1.idPid.ki = ki;
        }
        else if (strstr(vofaCmdBuf, "iqkp="))
        {
            float kp = vofa_cmd_parse(vofaCmdBuf, "iqkp=");
            motor1.iqPid.kp = kp;
            motor1.idPid.kp = kp;

//            motor1.iqPid.ka = 1.5f * motor1.idPid.ki / motor1.iqPid.kp;
//            motor1.idPid.ka = 1.5f * motor1.idPid.ki / motor1.idPid.kp;

            motor1.iqPid.ka = 1.0f / motor1.iqPid.kp;
            motor1.idPid.ka = 1.0f / motor1.idPid.kp;
        }
        else if (strstr(vofaCmdBuf, "iqki="))
        {
            float ki = vofa_cmd_parse(vofaCmdBuf, "iqki=");
            motor1.iqPid.ki = ki;
            motor1.idPid.ki = ki;

//            motor1.iqPid.ka = 1.5f * motor1.idPid.ki / motor1.iqPid.kp;
//            motor1.idPid.ka = 1.5f * motor1.idPid.ki / motor1.idPid.kp;

            motor1.iqPid.ka = 1.0f / motor1.iqPid.kp;
            motor1.idPid.ka = 1.0f / motor1.idPid.kp;
        }
        else if (strstr(vofaCmdBuf, "iqref="))
        {
            float iqref = vofa_cmd_parse(vofaCmdBuf, "iqref=");
            motor1.iqSet = iqref;
        }
        else if (strstr(vofaCmdBuf, "idref="))
        {
            float idref = vofa_cmd_parse(vofaCmdBuf, "idref=");
            motor1.idSet = idref;
        }
        else if (strstr(vofaCmdBuf, "radOffset="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "radOffset=");
            motor1.angleRadOffset = val;
        }
        else if (strstr(vofaCmdBuf, "velref="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "velref=");
            motor1.speedSet = val;
        }
        else if (strstr(vofaCmdBuf, "velkp="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "velkp=");
            motor1.speedPid.kp = val;
        }
        else if (strstr(vofaCmdBuf, "velki="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "velki=");
            motor1.speedPid.ki = val;
        }
        else if (strstr(vofaCmdBuf, "pos_ref="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "pos_ref=");
            motor1.positionSet = val;
        }
        else if (strstr(vofaCmdBuf, "pos_kp="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "pos_kp=");
            motor1.positionPid.kp = val;
        }
        else if (strstr(vofaCmdBuf, "pos_ki="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "pos_ki=");
            motor1.positionPid.ki = val;
        }
        else if (strstr(vofaCmdBuf, "theta_offset="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "theta_offset=");
            motor1.angleRadOffset = val;
        }
        else if (strstr(vofaCmdBuf, "hfi_volt="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "hfi_volt=");
            motor1.hfiVoltAmpl = val;
        }
        else if (strstr(vofaCmdBuf, "motor_en="))
        {
            float val = vofa_cmd_parse(vofaCmdBuf, "motor_en=");

            motor1.status = motor1.status == MOTOR_STATUS_RUN ? MOTOR_STATUS_STOP : MOTOR_STATUS_RUN;

            if (motor1.status == MOTOR_STATUS_RUN)
            {
                motor_enable(&motor1);
            }
            else
            {
                motor_disable(&motor1);
            }
        }
        else
        {
        }

        memset(vofaCmdBuf, 0, 1024);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, vofaCmdBuf, 1024);
    }
}

/* USER CODE END Application */

