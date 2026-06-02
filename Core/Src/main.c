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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "pid.h"
#include "math.h"

PIDController pid_speed1;
PIDController pid_speed2;

uint32_t en1_count = 0;
uint32_t num_byte1 = 0;
uint8_t data_rx_1;
uint8_t rx_buff[9];
bool rx_flag1 = false;
volatile int speed1=0,speed2=0,dir1=0,dir2=0;
float dt = 0.01f;
int debug_dat = 0;
uint8_t tx_buff[5];

#define encoder_ppr 333
#define sample_time 10.0f
#define abs(x) (x>0) ? x : -x

volatile int32_t pre_c1= 0;
volatile float m1_speed = 0.0f;
volatile int32_t encoder1_c = 0;

volatile int32_t pre_c2= 0;
volatile float m2_speed = 0.0f;
volatile int32_t encoder2_c = 0;

#define speed_size 5
float speed_buffer1[speed_size];
uint8_t speed_index1 = 0;

float speed_buffer2[speed_size];
uint8_t speed_index2 = 0;
volatile int err_line = 0;
uint8_t check = 0;
uint8_t out_speed1;
int8_t active_sensors = 0;
uint8_t sensor[5] = {0,0,0,0,0};

//int pid_speed2(uint16_t speed_target ,float speed_motor);
uint8_t line_value = 0;

bool pid1_FLAG = false, pid2_FLAG = false;

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
//	if(htim -> Instance == htim2.Instance ){
//		en1_count = __HAL_TIM_GET_COUNTER(htim);
//	}
//	
//	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart -> Instance == huart1.Instance){
//		if(data_rx_1 == 0x03){
//			rx_flag1 = true;
//		}
//		if(rx_flag1){
//			rx_buff[num_byte1] = data_rx_1;
//			num_byte1++;
//			if(num_byte1 > 8){
//				num_byte1 = 0;
//				rx_flag1 = false;
//			}
//		}
//	}
//	HAL_UART_Receive_IT(&huart1,&data_rx_1,1);
}
void Calculate_Motor_Speed_1(void){
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    int32_t delta = current_count - pre_c1;

    if (delta > 32768)       delta -= 65536;
    else if (delta < -32768) delta += 65536;

    pre_c1 = current_count;

    encoder1_c += delta;


    float counts_per_sec = (float)delta * (1000.0f / sample_time);
    float rev_per_sec    = counts_per_sec / (encoder_ppr * 4.0f);
    float rpm            = rev_per_sec * 60.0f;

    speed_buffer1[speed_index1] = rpm;
    speed_index1 = (speed_index1 + 1) % speed_size;

    float sum = 0.0f;
    for (uint8_t i = 0; i < speed_size; i++)
        sum += speed_buffer1[i];

    m1_speed = sum / speed_size;
}

void Calculate_Motor_Speed_2(void){
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
	
    int32_t delta = current_count - pre_c2;

    if (delta > 32768)       delta -= 65536;
    else if (delta < -32768) delta += 65536;

    pre_c2 = current_count;

    encoder2_c += delta;


    float counts_per_sec = (float)delta * (1000.0f / sample_time);
    float rev_per_sec    = counts_per_sec / (encoder_ppr * 4.0f);
    float rpm            = rev_per_sec * 60.0f;

    speed_buffer2[speed_index2] = rpm;
    speed_index2 = (speed_index2 + 1) % speed_size;

    float sum = 0.0f;
    for (uint8_t i = 0; i < speed_size; i++)
        sum += speed_buffer2[i];

    m2_speed = sum / speed_size;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim1){
      Calculate_Motor_Speed_1();
			Calculate_Motor_Speed_2();
			pid1_FLAG = true;
			pid2_FLAG = true;
    }
}
float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void led(uint16_t time){
	static uint32_t time_sw = 0;
	static uint8_t num_led = 0;
	
	if(HAL_GetTick()-time_sw >= time){
		num_led += 1;
		if(num_led > 4) num_led = 0;
		time_sw = HAL_GetTick();
	}
	
	switch (num_led){
		case 0:
			HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET);		
			HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_RESET);			
			HAL_GPIO_WritePin(led4_GPIO_Port,led4_Pin,GPIO_PIN_RESET);
		  break;
		case 1:
			HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_SET);
		  break;
		case 2:
			HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET);
		  break;
		case 3:
			HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_SET);
		  break;
    case 4:
			HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(led4_GPIO_Port,led4_Pin,GPIO_PIN_SET);
		  break;      			
	}
	
}

/*tinh gia tri cam bien*/

/*gui gia tri cam bien*/
void send_line_value(){
	static uint32_t time = 0;
	if(HAL_GetTick()-time > 10){
		//line_value = line_cal();
		if(err_line>=0){
			tx_buff[1] = 0xFF;
		}else{
			tx_buff[1] = 0x00;
		}
		tx_buff[0] = 0x04;
		tx_buff[2] = (err_line>0?err_line:-err_line);
		tx_buff[3] = active_sensors;		
		tx_buff[4] = 0xA3;
		HAL_UART_Transmit(&huart1,tx_buff,5,100);
		time = HAL_GetTick();
	}
}


void PID_Setup(void)
{
    pid_speed1.Kp = 3.5f;  // 1.95
    pid_speed1.Ki = 15.0f;  //  0.8
    pid_speed1.Kd = 0.2f; // 0.35

    pid_speed1.tau = 0.02f;  

    pid_speed1.limMin = -255.0f;
    pid_speed1.limMax = 255.0f;

    pid_speed1.limMinInt = -255.0f;
    pid_speed1.limMaxInt = 255.0f;

    pid_speed1.T = 0.01f;     

    PIDController_Init(&pid_speed1);
	  pid_speed2.Kp = 3.5f;  // 1.95
    pid_speed2.Ki = 15.0f;  //  0.7
    pid_speed2.Kd = 0.2f; // 0.35

    pid_speed2.tau = 0.02f;  

    pid_speed2.limMin = -255.0f;
    pid_speed2.limMax = 255.0f;

    pid_speed2.limMinInt = -255.0f;
    pid_speed2.limMaxInt = 255.0f;

    pid_speed2.T = 0.01f;     

    PIDController_Init(&pid_speed2);
}


uint32_t lastTick = 0;
float Ts = 10; // ms

void loop_PID_1(int speed){
    if(pid1_FLAG){
			if(dir1==0){
				speed = speed;
			}else{
				speed = -speed;
			}
			
			int output = PIDController_Update(&pid_speed1,speed, m1_speed);
			
			if (output > 0) {
					output = output * (255 - 80) / 255 + 80;
			}else if (output < 0) {
					output = output * (255 - 80) / 255 - 80;
			}
			
			//int pwm = abs(output);
			
			if(output < 80 || speed == 0) __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
			
			debug_dat = output;

			if(dir1==0){
				HAL_GPIO_WritePin(dir1_GPIO_Port,dir1_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dir2_GPIO_Port,dir2_Pin,GPIO_PIN_SET);					
			}else{
				HAL_GPIO_WritePin(dir1_GPIO_Port,dir1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dir2_GPIO_Port,dir2_Pin,GPIO_PIN_RESET);					
			}
			if(speed!=0){
				int pwm = (dir1==0)?(255+output):(255-output);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,constrain(pwm,0,255));
			}
			pid1_FLAG = false;	
    }
}

void loop_PID_2(int speed){
    if(pid2_FLAG){
			
			if(dir2==0){
				speed = speed;
			}else{
				speed = -speed;
			}
			
      float output = PIDController_Update(&pid_speed2, speed, m2_speed);
			if (output > 0) {
					output = output * (255 - 80) / 255 + 80;
			} else if (output < 0) {
					output = output * (255 - 80) / 255 - 80;
			}
			
			if(output < 80 || speed == 0 ) __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);
			

			if(dir2==0){
				HAL_GPIO_WritePin(dir3_GPIO_Port,dir3_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(dir4_GPIO_Port,dir4_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(dir3_GPIO_Port,dir3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(dir4_GPIO_Port,dir4_Pin,GPIO_PIN_RESET);
			}
			if(speed!=0){
				int pwm = (dir2==0)?(255+output):(255-output);
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,constrain(pwm,0,255));				
			} 
			pid2_FLAG = false;	
    }
}
void cal_err_line() {
    sensor[0] = !HAL_GPIO_ReadPin(in1_GPIO_Port, in1_Pin);
    sensor[1] = !HAL_GPIO_ReadPin(in2_GPIO_Port, in2_Pin);
    sensor[2] = !HAL_GPIO_ReadPin(in3_GPIO_Port, in3_Pin);
    sensor[3] = !HAL_GPIO_ReadPin(in4_GPIO_Port, in4_Pin);
    sensor[4] = !HAL_GPIO_ReadPin(in5_GPIO_Port, in5_Pin);

    int32_t sum = 0;
	  active_sensors = 0;

    if (sensor[0]) { sum += -20; active_sensors++; }
    if (sensor[1]) { sum += -10; active_sensors++; }
    if (sensor[2]) { sum += 0;   active_sensors++; }
    if (sensor[3]) { sum += 10;  active_sensors++; }
    if (sensor[4]) { sum += 20;  active_sensors++; }

    if (active_sensors > 0) {
        err_line = (float)sum / active_sensors;
    }else{
			
    }
}

void pid_line_speed(int base_speed_t){
	static double Kp,Ki,Kd;
	static int32_t error,last_err = 0,output;
	Kp = 1.5; Ki = 0.25; Kd = 0.15;// 5.25 0.3 1.5
	cal_err_line();
	error = 0 - err_line;
	output = Kp*(error) + Ki*(err_line+last_err) + Kd*(err_line-last_err);
	last_err = error;
	output = constrain(output,-10,10);
	
	loop_PID_1(base_speed_t+output);
	loop_PID_2(base_speed_t-output);
	//check = output;
}
void data_process(){
	if(rx_buff[0] == 0x03 && rx_buff[7] == 0x3A && rx_buff[8] == 0xFF){
		dir1 = rx_buff[1];
		dir2 = rx_buff[2];
		speed1 = (rx_buff[3] << 8) | rx_buff[4] ;
		speed2 = (rx_buff[5] << 8) | rx_buff[6] ;	
	}
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);
	HAL_UART_Receive_DMA(&huart1,rx_buff,9);
	HAL_TIM_Base_Start_IT(&htim1);
	PID_Setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		data_process();
		led(speed1);
		cal_err_line();
//		dir1 = 0;
//		dir2 = 0;
//		speed1 = 20;
//		speed2 = 20;
//		
		loop_PID_1(speed1);
		loop_PID_2(speed2);
		send_line_value();
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2499;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 124;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, dir4_Pin|dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led1_Pin|led2_Pin|led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, led4_Pin|dir1_Pin|dir3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dir4_Pin dir2_Pin */
  GPIO_InitStruct.Pin = dir4_Pin|dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : in1_Pin in2_Pin in3_Pin in4_Pin
                           in5_Pin */
  GPIO_InitStruct.Pin = in1_Pin|in2_Pin|in3_Pin|in4_Pin
                          |in5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led1_Pin led2_Pin led3_Pin */
  GPIO_InitStruct.Pin = led1_Pin|led2_Pin|led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led4_Pin dir1_Pin dir3_Pin */
  GPIO_InitStruct.Pin = led4_Pin|dir1_Pin|dir3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
