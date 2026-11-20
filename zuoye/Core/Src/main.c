/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include"oled.h"
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

/* USER CODE BEGIN PV */
uint16_t led_brightness = 0;//呼吸灯的亮度(0-999)
int16_t led_direction = 1;//呼吸灯亮度变化方向：1=变亮，-1=变暗
uint8_t LED_breath_enable = 1;//呼吸灯是否启用
uint32_t last_led_time = 0;//上一次更新呼吸灯亮度的时间，单位 ms

uint32_t key1_last_edge_time = 0;//上一次按键1的电平变化时间，单位 ms
uint8_t key1_wait_double_click = 0;//按键1是否正在等待第二次短按
uint32_t key1_first_click_time = 0;//按键1第一次短按松开的时间，单位 ms
uint8_t key1_is_pressed = 0;//按键1是否正被按住
uint32_t key1_press_start_time = 0;//按键1按下开始的时间，单位 ms
uint8_t key2_is_pressed = 0;//按键2是否正被按住
uint32_t key2_press_start_time = 0;//按键2按下开始的时间，单位 ms
uint32_t key2_last_edge_time = 0;//上一次按键2的电平变化时间，单位 ms
uint8_t key2_wait_double_click = 0;//按键2是否正在等待第二次短按
uint32_t key2_first_click_time = 0;//按键2第一次短按松开的时间，单位 ms
uint8_t both_keys_active = 0;//两个按键正在同时短按
uint8_t LED2_always_on = 0;//LED2是否常亮
 uint8_t LED1_always_on = 0;//LED1是否常亮
uint8_t led3_flash_enable = 0;//LED3是否启用闪烁
uint8_t computercontrol = 0;//是否由电脑控制（电脑或按键）
uint8_t led1mode = 0;//LED1的模式 （0=呼吸，1=闪烁，2=常亮，3=熄灭）
uint8_t led2mode = 0;//LED2的模式 （0=呼吸，1=闪烁，2=常亮，3=熄灭）
uint8_t led3mode = 0;//LED3的模式 （0=呼吸，1=闪烁，2=常亮，3=熄灭）
uint8_t receivedata[100]; //接收数据缓冲区（DMA接收到的电脑数据）
char keytext[20] = "none";//按键文本
char sendata[250]; //发送数据缓冲区（stm32要发送的数据）
uint16_t sendsize = 0;//本次实际要发送的字符数量
uint32_t last_send_time = 0;//上一次串口发送数据的时间，单位 ms
uint8_t mpuok = 0;//mpu是否正常连接
uint8_t mpuid = 0;//读取MPU6050芯片ID
uint8_t mpu6050_data[14];
uint8_t tiltwarning = 0;//倾斜警告
uint16_t adcvalue = 0;//ADC初始值
uint32_t adcvoltage = 0;//ADC换算后的电压值
uint32_t adclastreadtime = 0;//上一次读取ADC的时间，单位 ms

/* 三轴加速度原始值 */
int16_t accelerationX = 0;
int16_t accelerationY = 0;
int16_t accelerationZ = 0;

/* 三轴角速度原始值 */
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
uint32_t mpu6050_last_read_time = 0;//上一次读取MPU6050数据的时间，单位 ms
uint32_t lastoledtime = 0;//上一次OLED刷新时间，单位 ms
char oledtext[40] ;//OLED显示的文本
uint8_t oledpage = 0;// 0 MPU6050页面 1 LED页面 2 消息页面 3 计时与PWM页面 4 ADC电压
char messageHistory[10][21]; //最近十条消息历史记录
uint8_t messageCount = 0;//消息数量
uint8_t messageIndex = 0;//当前查看第几条消息（索引从0开始）
uint8_t watchstates = 0;//秒表状态：0是暂停，1是正在计时
uint32_t watchstarttime = 0;//秒表开始计时时间，单位 ms
uint32_t watchnowtotallytime = 0;//秒表当前总时间，单位 ms
uint32_t watchsavedtime = 0;//暂停之前已经累计的时间，单位 ms
uint8_t countdownstates = 0;//倒计时状态：0是暂停，1是正在倒计时
uint32_t countdownstarttime = 0;//倒计时开始计时时间，单位 ms
uint32_t countdownremainingtime = 24 * 60 * 60 * 1000;//倒计时当前剩余时间，单位 ms
uint16_t pwmfrequency = 50;//PWM频率
uint16_t pwmcomparetarget = 150;//比较的目标值
uint16_t pwmnowempty = 150;//输出的compare值
uint16_t pwmchangestep = 1;//PWM变化步长
uint32_t pwmlasttime = 0;//上一次PWMcompare值变化的时间，单位 ms
uint16_t pwmemptypersent = 75;//占空比单位为0.1%
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define KEY_DOUDONG 30U
#define KEY_LONG 800U
#define KEY_DOUBLE_CLICK 350U
#define KEY_SONG_TIME 120U//同时按下两个按键的时间窗口，单位 ms
#define MPU6050_ADDRESS 0x68
#define PWM_TIMER_CLOCK 100000U
#define PWM_MAX_FREQUENCY 999U
#define PWM_MIN_FREQUENCY 2U
#define SERVO_FREQUENCY 50
#define TILT_THRESHOLD 5000


static void Restore_Breathing_Mpu_Mode(void)//两按键同时短按：恢复呼吸灯并进入MPU6050页面
{
    key1_wait_double_click = 0;
    key2_wait_double_click = 0;
    computercontrol = 0;
    LED_breath_enable = 1;
    LED1_always_on = 0;
    LED2_always_on = 0;
    led3_flash_enable = 0;
    oledpage = 0;
    strcpy(keytext,"K1_K2_AN");
}
void MPU6050_Write(uint8_t reg, uint8_t data)
{
    uint8_t writedata[2] = {reg, data};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, writedata, 2,100);

}
void MPU6050_Readdata(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS << 1, data, len, 100);
}
void Savemessage(uint16_t Size)
{
    uint8_t i;
    uint8_t messagelength = 0;
    if (messageCount < 10)
    {
        messageIndex = messageCount;
        messageCount++;
    }else
    {
        for (i = 0; i < 9; i++)
        {
            strcpy(messageHistory[i], messageHistory[i + 1]);
        }
        messageIndex = 9;
    }
    while (messagelength < 20 && messagelength + 1 < Size && receivedata[messagelength + 1] != '\n' && receivedata[messagelength + 1] != '\r')
    {
        messageHistory[messageIndex][messagelength] = receivedata[messagelength + 1];
        messagelength++;
    }
    messageHistory[messageIndex][messagelength] = '\0';
    oledpage = 2; //进入消息页面
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//按键外部中断回调函数
{
    if (GPIO_Pin == KEY1_Pin)
    {
        uint32_t current_time = HAL_GetTick();
        uint32_t press_time;

        if (current_time - key1_last_edge_time < KEY_DOUDONG)
        {
            return;
        }
        key1_last_edge_time = current_time;

        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            key1_is_pressed = 1;
            key1_press_start_time = current_time;
            if (key2_is_pressed == 1 && current_time - key2_press_start_time <= KEY_SONG_TIME)
            {
                both_keys_active = 1;
                Restore_Breathing_Mpu_Mode();
            }
            return;
        }

        if (key1_is_pressed == 0)
        {
            return;
        }
        key1_is_pressed = 0;
        if (both_keys_active == 1)
        {
            if (key2_is_pressed == 0)
            {
                both_keys_active = 0;
            }
            return;
        }

        press_time = current_time - key1_press_start_time;
        if (press_time >= KEY_LONG)//K1长按：关闭呼吸灯
        {
            computercontrol = 0;
            LED_breath_enable = 0;
            LED1_always_on = 0;
            LED2_always_on = 0;
            led3_flash_enable = 0;
            key1_wait_double_click = 0;
            strcpy(keytext, "K1_LONG");
        }
        else if (key1_wait_double_click == 1 && current_time - key1_first_click_time <= KEY_DOUBLE_CLICK)//K1双击：LED2常亮/呼吸切换
        {
            computercontrol = 0;
            LED_breath_enable = 1;
            LED2_always_on = !LED2_always_on;
            key1_wait_double_click = 0;
            strcpy(keytext, "K1_DOUBLE_CLICK");
        }
        else
        {
            key1_wait_double_click = 1;
            key1_first_click_time = current_time;
       }
        return;
    }

    if (GPIO_Pin == KEY2_Pin)
    {
        uint32_t current_time = HAL_GetTick();
        uint32_t press_time;

        if (current_time - key2_last_edge_time < KEY_DOUDONG)
        {
            return;
        }
        key2_last_edge_time = current_time;

        if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        {
            key2_is_pressed = 1;
            key2_press_start_time = current_time;
            if (key1_is_pressed == 1 && current_time - key1_press_start_time <= KEY_SONG_TIME)
            {
                both_keys_active = 1;
                Restore_Breathing_Mpu_Mode();
            }
            return;
        }

        if (key2_is_pressed == 0)
        {
            return;
        }
        key2_is_pressed = 0;
        if (both_keys_active == 1)
        {
            if (key1_is_pressed == 0)
            {
                both_keys_active = 0;
            }
            return;
        }

        press_time = current_time - key2_press_start_time;
        if (press_time >= KEY_LONG)//K2长按：打开呼吸灯并进入消息页面
        {
            computercontrol = 0;
            key2_wait_double_click = 0;
            LED_breath_enable = 1;
            LED1_always_on = 0;
            LED2_always_on = 0;
            led3_flash_enable = 0;
            if (messageCount > 0)
            {
                messageIndex = messageCount - 1;
            }
            oledpage = 2;
            strcpy(keytext, "MESSAGE");
        }
        else if (key2_wait_double_click == 1 && current_time - key2_first_click_time <= KEY_DOUBLE_CLICK)//K2双击：MPU和LED页面切换
        {
            computercontrol = 0;
            key2_wait_double_click = 0;
            oledpage = (oledpage == 0) ? 1 : 0;
            strcpy(keytext, "K2_DOUBLE_CLICK");
        }
        else
        {
            key2_wait_double_click = 1;
            key2_first_click_time = current_time;
        }
    }
}
uint8_t qu_san_ge_shu(uint8_t*text,uint16_t*value)
{
    if (text[0] < '0' || text[0] > '9' || text[1] < '0' || text[1] > '9' || text[2] < '0' || text[2] > '9')
    {
        return 0;
    }
    *value = (text[0] - '0') * 100 + (text[1] - '0') * 10 + (text[2] - '0');
    return 1;
}
void PWM_setempty(uint16_t empty)
{
    uint32_t periodcount;
    if (empty > 999)
    {
        return;
    }
    periodcount = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;
    pwmemptypersent = empty;
    pwmcomparetarget = (uint16_t)(periodcount * empty / 1000U);
   
}
void PWM_setfrequency(uint16_t frequency)
{
    uint32_t periodcount;
   if (frequency < PWM_MIN_FREQUENCY || frequency > PWM_MAX_FREQUENCY)
   {
       return;
   }
    periodcount = PWM_TIMER_CLOCK / frequency;
    __HAL_TIM_SET_AUTORELOAD(&htim3,periodcount - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    pwmfrequency = frequency;
    pwmcomparetarget = (uint16_t)(periodcount * pwmemptypersent / 1000U);
    if (pwmnowempty > periodcount)
    {
        pwmnowempty = (uint16_t)periodcount;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmnowempty);
}
 void servo_setangle(uint16_t angle)
{
    uint16_t empty;
    if (angle > 180)
    {
        return;
    }
    if (pwmfrequency != SERVO_FREQUENCY)
    {
        PWM_setfrequency(SERVO_FREQUENCY);
    }
    /* 普通 180° 舵机使用 1.0 ms～2.0 ms 脉宽，避免两端顶住后抖动。 */
    empty = (uint16_t)(50U + (((uint32_t)angle * 50U + 90U) / 180U));
    PWM_setempty(empty);
}
void PWM_Update(uint32_t current_time)
{
    uint16_t chazhi;
    if (current_time - pwmlasttime < 20)
    {
        return;
    }
    pwmlasttime = current_time;
    if (pwmnowempty < pwmcomparetarget)
    {
        chazhi = pwmcomparetarget - pwmnowempty;
      if (chazhi <= pwmchangestep)
      {
        pwmnowempty = pwmcomparetarget;
      }else
      {
        pwmnowempty = pwmnowempty + pwmchangestep;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmnowempty);
    }
    else if (pwmnowempty > pwmcomparetarget)
    {
        chazhi = pwmnowempty - pwmcomparetarget;
      if (chazhi <= pwmchangestep)
      {
        pwmnowempty = pwmcomparetarget;
      }else
      {
        pwmnowempty = pwmnowempty - pwmchangestep;
      } 
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmnowempty);
    }

    
      }
 void countdown_update(uint32_t current_time)
{
    uint32_t timeleft;
    if (countdownstates == 0)
    {
        return;
    }
    timeleft = current_time - countdownstarttime;
    if (timeleft >= countdownremainingtime)
    {
        countdownstates = 0;
        countdownremainingtime = 0;
        strcpy(keytext, "COUNTDOWN_END");
    }else
    {
        countdownremainingtime = countdownremainingtime - timeleft;
        countdownstarttime = current_time;
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)//串口接收回调函数(串口空闲时调用)
{
    uint16_t value;
    if (huart == &huart2)
    {
        if (Size >= 2)
        {
            if (receivedata[0] == '0' && receivedata[1] == '0')
            {
                computercontrol = 0;
            }
            else if (receivedata[0] == '1')
            {
                computercontrol = 1;
               
                if (receivedata[1] == '1') led1mode = 0;
                if (receivedata[1] == '2') led1mode = 1;
                if (receivedata[1] == '3') led1mode = 2;
                if (receivedata[1] == '4') led1mode = 3;
            }
            else if (receivedata[0] == '2')
            {
                computercontrol = 1;
               
                if (receivedata[1] == '1') led2mode = 0;
                if (receivedata[1] == '2') led2mode = 1;
                if (receivedata[1] == '3') led2mode = 2;
                if (receivedata[1] == '4') led2mode = 3;
            }
            else if (receivedata[0] == '3')
            {
                computercontrol = 1;
                
                if (receivedata[1] == '1') led3mode = 0;
                if (receivedata[1] == '2') led3mode = 1;
                if (receivedata[1] == '3') led3mode = 2;
                if (receivedata[1] == '4') led3mode = 3;
            }else if (receivedata[0] == '4')
            {
                if (receivedata[1] == '1') oledpage = 0;
                if (receivedata[1] == '2') oledpage = 1;
                if (receivedata[1] == '3') oledpage = 2;
                if (receivedata[1] == '4') oledpage = 3;
                if (receivedata[1] == '5') oledpage = 4;
            }else if (receivedata[0] == '5')
            {
                oledpage = 3;
                if (receivedata[1] == '1')
                {
                    if (watchstates == 0)
                    {
                        watchstates = 1;
                        watchstarttime = HAL_GetTick();
                        strcpy(keytext, "WATCH_START");
                    }
                }else if (receivedata[1] == '2')
                {
                    if (watchstates == 1)
                    {
                        watchstates = 0;
                        watchsavedtime += HAL_GetTick() - watchstarttime;
                       
                        strcpy(keytext, "WATCH_PAUSE");
                    }
                }else if (receivedata[1] == '3')
                {
                    watchstates = 0;
                    watchsavedtime = 0;
                    watchstarttime = 0;
                    watchnowtotallytime = 0;
                    strcpy(keytext, "WATCH_RESET");
                }
            }else if (receivedata[0] == '6')
            {
                oledpage = 3;
                if (receivedata[1] == '1')
                {
                    if (countdownremainingtime > 0)
                    {
                        countdownstates = 1;
                        countdownstarttime = HAL_GetTick();
                        strcpy(keytext, "COUNTDOWN_START");
                    }
                }else if (receivedata[1] == '2')
                {
                    if (countdownstates == 1)
                    {
                        countdown_update(HAL_GetTick());
                    }
                    countdownstates = 0;
                    strcpy(keytext, "COUNTDOWN_PAUSE");
                }else if (receivedata[1] == '3')
                {
                    countdownstates = 0;
                    countdownremainingtime = 24U * 60U * 60U * 1000U;
                    countdownstarttime = 0;
                    strcpy(keytext, "COUNTDOWN_RESET");
                }
            }else if (receivedata[0] == '7')
            {
                oledpage = 3;
                if (receivedata[1] == '1')
                {
                    servo_setangle(0);
                    strcpy(keytext, "SERVO_0");
                }else if (receivedata[1] == '2')
                {
                    servo_setangle(90);
                    strcpy(keytext, "SERVO_90");
                }else if (receivedata[1] == '3')
                {
                    servo_setangle(180);
                    strcpy(keytext, "SERVO_180");
                }else if (receivedata[1] == '4')
                {
                    pwmchangestep = 1;
                    strcpy(keytext, "PWM_SLOW");
                }else if (receivedata[1] == '5')
                {
                    pwmchangestep = 2;
                    strcpy(keytext, "PWM_MID");
                }else if (receivedata[1] == '6')
                {
                    pwmchangestep = 4;
                    strcpy(keytext, "PWM_FAST");
                }
            }
        else if (receivedata[0] == 'A' && Size >= 4)
        {
            oledpage = 3;
            if (qu_san_ge_shu(&receivedata[1], &value) && (value >= 0 && value <= 180))
            {
                servo_setangle(value);
                strcpy(keytext, "PWM_ANGLE");
            }
        }
        else if (receivedata[0] == 'D' && Size >= 4)
        {
            oledpage = 3;
            if (qu_san_ge_shu(&receivedata[1], &value) && value <= 999)
            {
                PWM_setempty(value);
                strcpy(keytext, "PWM_EMPTY");

            }
        }else if (receivedata[0] == 'R' && Size >= 4)
        {
            oledpage = 3;
            if (qu_san_ge_shu(&receivedata[1], &value) && value >= 1 && value <= 500)
            {
             pwmchangestep = value;
             PWM_Update(HAL_GetTick());
                strcpy(keytext, "PWM_RATE");
            }
        }
        else if (receivedata[0] == 'F' && Size >= 4)
        {
              oledpage = 3;
              if (qu_san_ge_shu(&receivedata[1], &value))
              {
                PWM_setfrequency(value);
                strcpy(keytext, "PWM_FREQ");
              }
        }
    }
}
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receivedata, sizeof(receivedata));
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
   HAL_ADCEx_Calibration_Start(&hadc1);
 
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    PWM_setfrequency(pwmfrequency);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
   
    HAL_Delay(20);
    OLED_Init();
    MPU6050_Write(0x6B, 0x00);//唤醒MPU6050
    MPU6050_Readdata(0x75, &mpuid, 1);//读取MPU6050芯片ID
    if (mpuid == 0x68)
{
    mpuok = 1;
    strcpy(keytext, "MPU_OK");
}
else
{
    mpuok = 0;
    strcpy(keytext, "MPU_ERROR");
}
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receivedata, sizeof(receivedata));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_time = HAL_GetTick();
    uint16_t flashbrightness;
    if (watchstates == 1)
    {
        watchnowtotallytime = watchsavedtime + current_time - watchstarttime;
    }else 
    {
    watchnowtotallytime = watchsavedtime;
    }
    countdown_update(current_time);
    PWM_Update(current_time);
    if (current_time - adclastreadtime >= 200)
    {
        adclastreadtime = current_time;
        if (HAL_ADC_Start(&hadc1) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
        {
            adcvalue = HAL_ADC_GetValue(&hadc1);
            adcvoltage = (uint32_t)adcvalue * 3300 / 4096;
        }
        HAL_ADC_Stop(&hadc1);
        }
        
    }

    if (key1_wait_double_click == 1 && current_time - key1_first_click_time > KEY_DOUBLE_CLICK)//K1短按：切换呼吸灯开关
    {
        key1_wait_double_click = 0;
        computercontrol = 0;
        LED_breath_enable = !LED_breath_enable;
        LED2_always_on = 0;
        strcpy(keytext, "K1_SHORT");
    }
    if (key2_wait_double_click == 1 && current_time - key2_first_click_time > KEY_DOUBLE_CLICK)//K2短按：其他页面切换LED3闪烁，消息页面查看上一条
    {
        key2_wait_double_click = 0;
        computercontrol = 0;
        if (oledpage == 2 && messageCount > 0)
        {
            if (messageIndex == 0)
            {
                messageIndex = messageCount - 1;
            }
            else
            {
                messageIndex--;
            }
            strcpy(keytext, "MSG_UP");
        }
        else
        {
            led3_flash_enable = !led3_flash_enable;
            strcpy(keytext, "K2_SHORT");
        }
    }
    if (mpuok == 1 && current_time - mpu6050_last_read_time >= 100)
    {
        mpu6050_last_read_time = current_time;
        MPU6050_Readdata(0x3B, mpu6050_data, 14);
        accelerationX = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data[1]);
        accelerationY = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data[3]);
        accelerationZ = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data[5]);
        gyroX = (int16_t)((mpu6050_data[8] << 8) | mpu6050_data[9]);
        gyroY = (int16_t)((mpu6050_data[10] << 8) | mpu6050_data[11]);
        gyroZ = (int16_t)((mpu6050_data[12] << 8) | mpu6050_data[13]);
        if (accelerationX > TILT_THRESHOLD || accelerationX < -TILT_THRESHOLD || accelerationY > TILT_THRESHOLD || accelerationY < -TILT_THRESHOLD)
        {
           tiltwarning = 1;
        }else {
        tiltwarning = 0;
        }
    }
    
    if (LED_breath_enable == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }else
    {
        if (current_time - last_led_time >=2)
    {
        last_led_time = current_time;
       
        if (led_brightness >= 999)
        {
            led_brightness = 999;
            led_direction = -1;
        }
        else if (led_brightness == 0)
        {
            led_direction = 1;
        }
    }
     led_brightness = (uint16_t)((int16_t)led_brightness + led_direction);
        if (LED1_always_on == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_brightness);
        }
        if (LED2_always_on == 1)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_brightness);
        }
        if (led3_flash_enable == 1)
        {
            if ((current_time / 100 )% 2 == 0)
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
            }
            else
            {
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            }
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_brightness);
        }
    }

    flashbrightness = ((current_time / 100U) % 2U == 0U) ? 999U : 0U;
if (computercontrol == 1)
{
   
   
    
    if (led1mode == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_brightness);
    }
    else if (led1mode == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, flashbrightness);
    }
    else if (led1mode == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
    }
    else if (led1mode == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
    if (led2mode == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_brightness);
    }
    else if (led2mode == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, flashbrightness);
    }
    else if (led2mode == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
    }
    else if (led2mode == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    if (led3mode == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_brightness);
    }
    else if (led3mode == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, flashbrightness);
    }
    else if (led3mode == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
    }
    else if (led3mode == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }
}
if (tiltwarning == 1)
{
    if ((current_time / 100U) % 2U == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }
}
if (current_time - lastoledtime >= 200)
{
    lastoledtime = current_time;
    OLED_NewFrame();
    if (oledpage == 0)
    {
        OLED_PrintASCIIString(0, 0, "MPU6050", &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "AX:%d AY:%d", accelerationX, accelerationY);
        OLED_PrintASCIIString(0, 10, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "AZ:%d", accelerationZ);
        OLED_PrintASCIIString(0, 20, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "GX:%d GY:%d", gyroX, gyroY);
        OLED_PrintASCIIString(0, 30, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "GZ:%d", gyroZ);
        OLED_PrintASCIIString(0, 40, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        if (tiltwarning == 1)
        {
            OLED_PrintASCIIString(0, 50, "TILT WARNING", &afont8x6, OLED_COLOR_NORMAL);
        }
        else {
            OLED_PrintASCIIString(0, 50, "NO TILT WARNING", &afont8x6, OLED_COLOR_NORMAL);
        }
    }
    else if (oledpage == 1)
    {
        OLED_PrintASCIIString(0, 0, "LED CONTROL", &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "LED1:%u", (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1));
        OLED_PrintASCIIString(0, 12, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "LED2:%u", (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2));
        OLED_PrintASCIIString(0, 24, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "LED3:%u", (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3));
        OLED_PrintASCIIString(0, 36, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "KEY:%s", keytext);
        OLED_PrintASCIIString(0, 48, oledtext, &afont8x6, OLED_COLOR_NORMAL);
    }
     else if (oledpage == 2)
    {
        OLED_PrintASCIIString(0, 0, "MESSAGE", &afont8x6, OLED_COLOR_NORMAL);
        if (messageCount == 0)
        {
            OLED_PrintASCIIString(0, 16, "NO MESSAGE", &afont8x6, OLED_COLOR_NORMAL);
        }
        else
        {
            snprintf(oledtext, sizeof(oledtext), "MSG %u/%u", (unsigned int)(messageIndex + 1), (unsigned int)messageCount);
            OLED_PrintASCIIString(0, 16, oledtext, &afont8x6, OLED_COLOR_NORMAL);
            OLED_PrintASCIIString(0, 32, messageHistory[messageIndex], &afont8x6, OLED_COLOR_NORMAL);
        }
        
    }
    else if (oledpage == 3)
    {
        uint32_t swseconds = watchnowtotallytime / 1000;
        uint32_t swdecisecond = (watchnowtotallytime / 100U) % 10U;
        uint32_t cdseconds = countdownremainingtime / 1000;
        uint32_t pwmperiod = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
        uint32_t pwmduty = (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) * 1000U) / pwmperiod;
        OLED_PrintASCIIString(0, 0, "TIMER & PWM", &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "SW:%02lu:%02lu:%02lu.%lu %s", (unsigned long)(swseconds / 3600), (unsigned long)((swseconds % 3600) / 60), (unsigned long)(swseconds % 60), (unsigned long)swdecisecond, watchstates ? "RUN" : "PAUSE");
        OLED_PrintASCIIString(0, 12, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext, sizeof(oledtext), "CD:%02lu:%02lu:%02lu %s", (unsigned long)(cdseconds / 3600), (unsigned long)((cdseconds % 3600) / 60), (unsigned long)(cdseconds % 60), countdownstates ? "RUN" : "PAUSE");
        OLED_PrintASCIIString(0, 24, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext,sizeof(oledtext),"F:%uHz D%lu.%lu%%", (unsigned int)pwmfrequency, (unsigned long)(pwmduty / 10U), (unsigned long)(pwmduty % 10U));
        OLED_PrintASCIIString(0, 36, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext,sizeof(oledtext),"CCR:%u R:%u",(unsigned int)pwmnowempty,(unsigned int)pwmchangestep);
        OLED_PrintASCIIString(0, 48, oledtext, &afont8x6, OLED_COLOR_NORMAL);
    }
    else if (oledpage == 4)
    {
        OLED_PrintASCIIString(0, 0, "ADC VOLAGE", &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext,sizeof(oledtext),"RAW:%u", (unsigned int)adcvalue);
        OLED_PrintASCIIString(0, 16, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        snprintf(oledtext,sizeof(oledtext),"VOLAGE:%lu.%03luV", (unsigned long)(adcvoltage / 1000), (unsigned long)(adcvoltage % 1000));
        OLED_PrintASCIIString(0, 32, oledtext, &afont8x6, OLED_COLOR_NORMAL);
        OLED_PrintASCIIString(0, 48, "input:PAO", &afont8x6, OLED_COLOR_NORMAL);
    }
   

    OLED_ShowFrame();
}
if (current_time - last_send_time >= 200)
{
    int textlength;
    uint32_t swseconds = watchnowtotallytime / 1000;
    uint32_t cdseconds = countdownremainingtime / 1000;
    uint32_t pwmperiod = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
    uint32_t pwmduty = (__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1) * 1000U) / pwmperiod;
    
    textlength = snprintf(sendata, sizeof(sendata), "LED1=%u,LED2=%u,LED3=%u,KEY=%s,"
    "sw=%02lu:%02lu:%02lu,"
    "cd=%02lu:%02lu:%02lu,"
    "pwm=%uHz,empty=%u.%u%%,rate=%u,"
    "AX=%d, AY=%d, AZ=%d, GX=%d, GY=%d, GZ=%d,"
    "ADC = %u,V = %lu.%03luV,"
    "ccr=%u,target=%u\r\n", (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1), (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2), (unsigned int)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3), keytext,(unsigned long)(swseconds / 3600), (unsigned long)((swseconds % 3600) / 60), (unsigned long)(swseconds % 60),
    (unsigned long)(cdseconds / 3600), (unsigned long)((cdseconds % 3600) / 60), (unsigned long)(cdseconds % 60),
    (unsigned int)pwmfrequency, (unsigned int)(pwmduty / 10U), (unsigned int)(pwmduty % 10U),
    (unsigned int)pwmchangestep,accelerationX, accelerationY, accelerationZ,gyroX, gyroY, gyroZ,(unsigned int)adcvalue, (unsigned long)(adcvoltage / 1000), (unsigned long)(adcvoltage % 1000),__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1), (unsigned int)pwmcomparetarget );

    if(textlength > 0 && textlength < sizeof(sendata))
    {
    sendsize = (uint16_t)textlength;
    if (HAL_UART_Transmit_DMA(&huart2,(uint8_t*)sendata, sendsize) == HAL_OK)
    {
        last_send_time = current_time;
    }

    }
}
  }

    



    

    

        


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
