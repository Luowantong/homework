







1.  PA8/PA9/PA10  = TIM1 三路 LED PWM
  PB12/PB14     = K1/K2 外部中断按键（上拉、按下为低）
  PA2/PA3       = USART2 TX/RX，115200
  PB6/PB7       = I2C1 SCL/SDA，MPU6050 和 OLED
  PA6           = TIM3_CH1，舵机 PWM
  PA0           = ADC1_IN0，0~3.3 V 电压采样


2. main.c 顶部：
------------------------------------------------------------


2.1 LED 状态
  led_brightness       当前呼吸亮度数值，设计范围 0~999
  led_direction        呼吸方向：1 变亮，-1 变暗
  LED_breath_enable    三灯总呼吸是否启用
  LED1_always_on       LED1 是否强制常亮
  LED2_always_on       LED2 是否强制常亮
  led3_flash_enable    LED3 是否使用闪烁效果



2.3 上位机控制状态
  computercontrol      0=本地按键/自动逻辑控制，1=串口控制三灯模式
  led1mode/led2mode/led3mode
                       0=呼吸，1=闪烁，2=常亮，3=熄灭
  receivedata[100]     DMA 接收到的串口数据
  sendata[250]         STM32 准备经 DMA 发送给上位机的状态字符串
  keytext              最近一次按键或命令事件的文字说明

2.4 MPU6050 与倾斜报警状态
  mpuok                是否读到了正确的 MPU6050 芯片 ID（0x68）
  mpu6050_data[14]     从寄存器 0x3B 一次读取的 14 字节缓存
  accelerationX/Y/Z    三轴加速度“原始 16 位数值”
  gyroX/Y/Z            三轴陀螺仪“原始 16 位数值”
  tiltwarning          X 或 Y 原始加速度超过阈值时置 1

2.5 OLED 和消息状态
  oledpage             0=MPU，1=LED，2=消息，3=计时/PWM，4=ADC
  oledtext[40]         OLED 每行临时文字缓存
  messageHistory       最多十条、每条最多 20 个字符的历史消息
  messageCount         当前保存的消息数量
  messageIndex         当前正在显示哪一条消息

2.6 秒表、倒计时、舵机/普通 PWM
  watchstates/watchstarttime/watchsavedtime/watchnowtotallytime
                       秒表是否运行、起点、已累计时间、当前显示时间
  countdownstates/countdownstarttime/countdownremainingtime
                       倒计时是否运行、上次计算时刻、剩余时间
  pwmfrequency         TIM3 当前频率，默认 50 Hz
  pwmcomparetarget     想要到达的 TIM3 比较值（目标 CCR）
  pwmnowempty          当前已经输出的 TIM3 比较值（当前 CCR）
  pwmchangestep        每次平滑变化的步长
  pwmemptypersent      占空比，单位为 0.1%，例如 75 表示 7.5%



3. 宏定义：
------------------------------------------------------------
KEY_DOUDONG       30 ms   按键消抖时间
KEY_LONG           800 ms  长按门槛
KEY_DOUBLE_CLICK   350 ms  双击等待窗口
KEY_SONG_TIME      120 ms  两键近同时按下的时间窗口
MPU6050_ADDRESS    0x68    MPU6050 I2C 地址
PWM_TIMER_CLOCK    100000  TIM3 经过预分频后的计数频率
SERVO_FREQUENCY    50 Hz   舵机使用的频率
TILT_THRESHOLD     5000    X/Y 加速度原始值的倾斜报警阈值


4. 自定义函数：按 main.c 出现的先后顺序解释
------------------------------------------------------------

4.1 Restore_Breathing_Mpu_Mode(void)
------------------------------------------------------------
用途：处理“两键近同时按下”的一键恢复动作。

它会做以下事：
  1. 取消 K1/K2 的双击等待；
  2. computercontrol = 0，退出电脑控制；
  3. LED_breath_enable = 1，恢复呼吸；
  4. 清除 LED1/LED2 常亮和 LED3 闪烁状态；
  5. oledpage = 0，返回 MPU6050 页面；
  6. keytext 写为 K1_K2_AN，供 OLED/串口显示。

HAL_GPIO_EXTI_Callback() 中检测到两个按键在 120 ms 内先后按下时。


4.2 MPU6050_Write(uint8_t reg, uint8_t data)
------------------------------------------------------------
用途：给 MPU6050 的一个寄存器写一个字节。

做法：将 {reg, data} 组成两个字节，通过 HAL_I2C_Master_Transmit() 发给
I2C 地址 0x68。

启动时执行 MPU6050_Write(0x6B, 0x00)，退出休眠模式。


4.3 MPU6050_Readdata(uint8_t reg, uint8_t *data, uint8_t len)
------------------------------------------------------------
用途：从 MPU6050 的 reg 开始连续读取 len 个字节。

做法：先发送寄存器地址，再调用 HAL_I2C_Master_Receive() 接收数据。

典型调用：每 100 ms 从 0x3B 开始读取 14 字节，放入 mpu6050_data[]。


4.4 Savemessage(uint16_t Size)
------------------------------------------------------------
用途：将 receivedata[] 中的一条文本存到消息历史数组。

行为：
  1. 未满十条时，在末尾加入一条；
  2. 已满十条时，整体前移，丢掉最旧的一条；
  3. 从 receivedata[1] 开始复制，最多复制 20 字符；
  4. 遇到 \r 或 \n 停止；
  5. 自动切换 OLED 到消息页（oledpage = 2）。



4.5 HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
------------------------------------------------------------
用途：GPIO 外部中断的回调函数。K1 和 K2 的按下、松开都会进入这里。

输入参数：GPIO_Pin 表示是哪一个引脚触发了中断。

通用处理方式：
  1. 读 HAL_GetTick() 得到当前时间；
  2. 如果与上次电平变化间隔小于 30 ms，认为是抖动，直接返回；
  3. 按键是上拉接法：读取到 GPIO_PIN_RESET 表示“刚按下”；
  4. 按下时记录开始时间，松开时计算 press_time；
  5. 根据 press_time、双击状态和另一颗按键状态，决定长按、双击或待确认短按。

K1 分支（GPIO_Pin == KEY1_Pin）：
  - 按下：标记 key1_is_pressed，并尝试判断是否和 K2 同时按下；
  - 长按 >= 800 ms：关闭呼吸和 LED 特殊效果；
  - 第二次短按落在 350 ms 窗口内：切换 LED2 常亮/呼吸；
  - 第一次短按：只写 key1_wait_double_click = 1，真正的单击动作延后到主循环。

K2 分支（GPIO_Pin == KEY2_Pin）：
  - 按下：标记 key2_is_pressed，并尝试判断是否和 K1 同时按下；
  - 长按 >= 800 ms：进入消息页；
  - 双击：在 MPU 页(0)和 LED 页(1)间切换；
  - 第一次短按：只进入等待双击状态；实际单击动作同样由主循环完成。




4.6 qu_san_ge_shu(uint8_t *text, uint16_t *value)
------------------------------------------------------------
用途：把连续三个 ASCII 数字转换为一个 0~999 的整数。

例如 text 指向 {'0','9','0'} 时，*value 得到 90。
如果任何一个字符不是 '0'~'9'，返回 0；成功返回 1。

它为 Axxx、Dxxx、Rxxx、Fxxx 等三位参数命令准备。


4.7 PWM_setempty(uint16_t empty)
------------------------------------------------------------
用途：设置 TIM3 的“目标占空比”，但不立即跳到目标值。

输入：empty，范围 0~999，单位为 0.1%。

做法：
  1. 读取 TIM3 当前周期计数值；
  2. 保存 pwmemptypersent；
  3. 计算 pwmcomparetarget = periodcount * empty / 1000。

真正修改 TIM3 CCR 的工作交给后面的 PWM_Update()，因此可实现平滑变化。


4.8 PWM_setfrequency(uint16_t frequency)
------------------------------------------------------------
用途：改变 TIM3 的 PWM 频率。

输入范围：2~999 Hz，超出范围直接返回。

核心计算：
  periodcount = PWM_TIMER_CLOCK / frequency
  ARR = periodcount - 1

然后根据已有的 pwmemptypersent 重新计算目标 CCR，这样改频率后会尽量保持
相同占空比。


4.9 servo_setangle(uint16_t angle)
------------------------------------------------------------
用途：将舵机角度转换为 PWM 目标占空比。

处理步骤：
  1. angle 大于 180 则直接返回；
  2. 若 TIM3 当前频率不是 50 Hz，先调用 PWM_setfrequency(50)；
  3. 0~180 度映射为 50~100（即 5.0%~10.0% 占空比）；
  4. 调用 PWM_setempty() 设定目标。

在 50 Hz（周期 20 ms）下：
  5.0% 约为 1.0 ms 脉宽，对应 0 度；
  7.5% 约为 1.5 ms 脉宽，对应 90 度；
 10.0% 约为 2.0 ms 脉宽，对应 180 度。


4.10 PWM_Update(uint32_t current_time)
------------------------------------------------------------
用途：使 TIM3 的当前 CCR 以较慢速度向目标 CCR 靠近。

调用位置：主循环每次都调用；函数内部保证至少间隔 20 ms 才更新一次。

行为：
  - pwmnowempty < pwmcomparetarget：按 pwmchangestep 增加；
  - pwmnowempty > pwmcomparetarget：按 pwmchangestep 减少；
  - 到达目标后停止改变；
  - 每次改变后用 __HAL_TIM_SET_COMPARE() 写入 TIM3_CH1。




4.11 countdown_update(uint32_t current_time)
------------------------------------------------------------
用途：更新倒计时剩余毫秒数。

处理步骤：
  1. 倒计时未运行时立即返回；
  2. timeleft = current_time - countdownstarttime；
  3. 如果耗时已超过剩余时间，停止并将剩余时间置零；
  4. 否则从剩余时间中减去 timeleft，并把 countdownstarttime 更新为当前时刻。



4.12 HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
------------------------------------------------------------
用途：USART2 使用 DMA Receive-to-Idle 接收时，出现串口空闲事件后进入的回调。

输入参数：
  huart：触发回调的串口句柄；
  Size：本次 DMA 实际接收的字节数。

处理顺序：
  1. 先确认 huart == &huart2；
  2. 按 receivedata[0] 和 receivedata[1] 解析命令；
  3. 修改 LED、OLED、秒表、倒计时、舵机等“状态变量”；
  4. 重新调用 HAL_UARTEx_ReceiveToIdle_DMA()，准备接收下一帧；
  5. 关闭 DMA 半传输中断，避免半个缓存到达时多一次无用回调。

两字符命令表：
  00            退出电脑控制
  11/12/13/14   LED1：呼吸/闪烁/常亮/熄灭
  21/22/23/24   LED2：呼吸/闪烁/常亮/熄灭
  31/32/33/34   LED3：呼吸/闪烁/常亮/熄灭
  41/42/43/44/45  切换 OLED 第 0/1/2/3/4 页
  51/52/53     秒表：开始/暂停/复位
  61/62/63     倒计时：开始/暂停/复位为 24 小时
  71/72/73     舵机：0 度/90 度/180 度
  74/75/76     PWM 平滑速度：1/2/4

代码还设计了以下带三位参数的命令：
  Axxx          舵机角度，例如 A090
  Dxxx          TIM3 占空比，例如 D075 表示 7.5%
  Rxxx          平滑变化步长，例如 R010
  Fxxx          TIM3 频率，例如 F050




4.13 SystemClock_Config(void)
------------------------------------------------------------
用途：配置系统时钟和 ADC 时钟。它由 CubeMX 生成，main() 上电时只调用一次。

当前设置：
  HSE 外部 8 MHz 晶振
      -> PLL x9
      -> SYSCLK/HCLK 72 MHz
      -> APB1 36 MHz（APB1 上的 TIM3 实际定时器时钟仍为 72 MHz）
      -> APB2 72 MHz（TIM1 所在总线）
      -> ADC 时钟为 PCLK2/6 = 12 MHz

若晶振、时钟树或 ADC 分频配置错误，UART 波特率、PWM 频率、HAL_GetTick()
相关的定时效果都会受影响。


4.14 HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
------------------------------------------------------------
用途：启动定时器，并使能中断。

输入参数：定时器句柄。

中断回调函数：HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  - htim == &htim3 时，进入 TIM3_IRQHandler()，再进入 TIM3_UP_TIM10_IRQHandler()；
  - htim == &htim10 时，进入 TIM10_IRQHandler()。





5. main()：上电后按顺序发生什么
------------------------------------------------------------

5.1 HAL_Init()
初始化 HAL 库、Flash 接口和 SysTick。HAL_GetTick() 的毫秒计时从这里开始可用。

5.2 SystemClock_Config()
使用 HSE 外部 8 MHz 晶振，经 PLL x9 得到 72 MHz 系统时钟。
APB1 为 36 MHz，但 APB1 定时器时钟为 72 MHz；ADC 时钟为 12 MHz。

5.3 MX_GPIO_Init() 与 MX_DMA_Init()
配置 GPIO、K1/K2 的 EXTI 中断、DMA 通道及相关 NVIC 中断。

5.4 MX_TIM1_Init()
配置 PA8/PA9/PA10 为 TIM1 三路 PWM。它们用于 LED 亮度控制。

5.5 MX_USART2_UART_Init()
配置 PA2/PA3 为 USART2，115200 bps；同时关联 DMA1 Channel6（RX）和
DMA1 Channel7（TX）。

5.6 MX_I2C1_Init()
配置 PB6/PB7 为 100 kHz I2C。OLED 与 MPU6050 都走这条总线。

5.7 MX_TIM3_Init()
配置 PA6 为 TIM3_CH1。TIM3 预分频后计数频率为 100 kHz，方便按微秒级
计算舵机脉宽。

5.8 MX_ADC1_Init() 与 HAL_ADCEx_Calibration_Start()
配置 PA0 为 ADC1_IN0，并在开机时校准 ADC，减少芯片内部偏差。

5.9 启动 PWM
程序启动 TIM1 的 CH1/CH2/CH3，LED PWM 才会真的从引脚输出。
随后设置 TIM3 默认 50 Hz，并启动 TIM3_CH1，给舵机输出初始 PWM。

5.10 初始化 OLED 和 MPU6050
先延时 20 ms 给 OLED 上电稳定时间，再 OLED_Init()。
接着唤醒 MPU6050，读取 WHO_AM_I 判断 mpuok。结果会写入 keytext，
后续 OLED/串口可显示 MPU_OK 或 MPU_ERROR。

5.11 开启 UART DMA 接收
调用 HAL_UARTEx_ReceiveToIdle_DMA() 后，串口才能异步接收上位机命令。


6. while(1)：主循环按真正执行顺序解释
------------------------------------------------------------
下面是理解整个项目最重要的一节。每轮 while(1) 都按以下顺序执行。

步骤 1：读取当前时间
  uint32_t current_time = HAL_GetTick();
  后续所有“每隔多久做一次”的判断都用它完成。

步骤 2：更新秒表显示时间
  - watchstates == 1 时：当前时间 = 已保存时间 + (当前时刻 - 本次开始时刻)；
  - 暂停时：当前时间固定为 watchsavedtime。

步骤 3：调用 countdown_update(current_time)
  如果倒计时正在运行，减少 countdownremainingtime；否则无动作。

步骤 4：调用 PWM_Update(current_time)
  让舵机的当前 CCR 慢慢接近目标 CCR。因为函数内部限制 20 ms 才更新一次，
  所以不会在每轮 while 中猛烈改变。

步骤 5：每约 200 ms 读取一次 ADC
  条件：current_time - adclastreadtime >= 200。
  依次 Start -> PollForConversion(最多等 5 ms) -> GetValue -> 换算毫伏 -> Stop。
  adcvalue 保存原始值；adcvoltage 保存按 3.3 V 参考计算的毫伏值。

步骤 6：处理“已经排除双击”的 K1 短按
  K1 松手后不是马上执行；超过 350 ms 仍未出现第二次点击，才在此处：
  - 切换 LED_breath_enable；
  - 清除 LED2 常亮；
  - 更新 keytext 为 K1_SHORT。

步骤 7：处理“已经排除双击”的 K2 短按
  同样等待 350 ms 后：
  - 如果当前是消息页并且 messageCount > 0，messageIndex 跳到上一条；
  - 否则切换 led3_flash_enable。

步骤 8：每约 100 ms 读取 MPU6050
  仅当 mpuok == 1 时读取 0x3B 开始的 14 字节。
  程序将每两个字节拼成一个 int16_t，分别放入 AX/AY/AZ/GX/GY/GZ。
  然后判断 |AX| 或 |AY| 是否超过 5000，决定 tiltwarning。

步骤 9：写入“基础 LED 效果”
  如果 LED_breath_enable == 0，三个 TIM1 CCR 都写为 0。
  否则按 led_brightness 给三路写入呼吸值；LED1/LED2 可被常亮标志替代，
  LED3 可被闪烁标志替代。

  当前代码阅读注意：设计意图是每隔约 2 ms 更新一次 led_brightness；但
  led_brightness = led_brightness + led_direction 这一句目前位于时间判断块外，
  因此实际会在每轮 while 都执行。若想严格按 2 ms 步进，应将它放进
  if (current_time - last_led_time >= 2) 的花括号内。

步骤 10：上位机 LED 模式覆盖基础 LED 效果
  如果 computercontrol == 1，按 led1mode/led2mode/led3mode 再写一遍 TIM1
  三路 CCR。由于它出现在基础效果之后，电脑命令会覆盖前面的结果。

步骤 11：倾斜报警最终覆盖 LED3
  如果 tiltwarning == 1，LED3 每 100 ms 在 999 与 0 之间翻转。
  这段位于电脑控制之后，所以报警优先级最高。

步骤 12：每约 200 ms 构建并显示一帧 OLED
  条件：current_time - lastoledtime >= 200。
  先 OLED_NewFrame()，根据 oledpage 写入对应页面文本，再 OLED_ShowFrame()。

  页 0：MPU6050 原始数据与 TILT WARNING/NO TILT WARNING
  页 1：LED 三路 CCR 和 keytext
  页 2：消息历史；空时显示 NO MESSAGE
  页 3：秒表、倒计时、TIM3 PWM 信息
  页 4：ADC 原始值和电压

步骤 13：每约 200 ms 通过 UART DMA 上报状态
  通过 snprintf() 将 LED CCR、KEY、计时、PWM、MPU 原始数据、ADC 数据等
  拼到 sendata[]，再调用 HAL_UART_Transmit_DMA() 发送。
  只有 DMA 启动成功时，last_send_time 才会更新。



7. OLED 模块
------------------------------------------------------------
OLED 使用“先画到显存，再一次显示”的方式：

  OLED_NewFrame()
      -> OLED_PrintASCIIString() / snprintf()
      -> OLED_ShowFrame()






