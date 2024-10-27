/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "st7735.h"
#include "logo.h"
#include "psu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum EncStates {NO_TRN, INC_TRN_SLOW, INC_TRN_NORM,INC_TRN_FAST,
						DEC_TRN_SLOW, DEC_TRN_NORM,DEC_TRN_FAST};
enum BtnStates {NO_PRESS, FIRST_EDGE, PRESS_NORM, PRESS_LONG};
enum MenuSelected {ON_OFF,MODE, MEM, BL, U, I};
enum dsMenuSelected {U_DISP,I_DISP, U_SETP, I_SETP, EXIT, SAVE_EXIT};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DAC_IO_TEST //to test DAC correct soldering
#define TRN_SLOW_NORM  4   //4
#define TRN_NORM_FAST  10 //10
#define NORM_PRESS_TIME  1 // *100 in mS
#define LONG_PRESS_TIME  5 // *100 in mS
#define SCALE_U_MIN 8.0
#define SCALE_U_MAX 11.0
#define SCALE_I_MIN 0.5
#define SCALE_I_MAX 2.0
#define SCALE_U_SP_MIN 27.0
#define SCALE_U_SP_MAX 38.0
#define SCALE_I_SP_MIN 205.0
#define SCALE_I_SP_MAX 230.0
#define I2C_SLAVE_ADDRESS 0x2D
#define I2C_BUFF_LEN 16 //multiple of 4 number
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
volatile uint16_t adc_RAW[3]; //0-ADC-V,1-ADC-I,2-Temp.ADC
float vdd = 3.302; //On board regulated voltage
float scaleU=9.46 , scaleI=1.43; // scaling constants for U,I~9,1/1,48
float scaleUsp=33.40, scaleIsp=218.00; //scaling const. for set points
float constU, constI; // pre-calculated constant to speed up U&I calculation
float outU, outI, temp_MCU;
char float_for_LCD[CHAR_BUFF_SIZE+1]; // +1 is used for "stop string" generation
enum EncStates enc = NO_TRN;
enum BtnStates btn = NO_PRESS;
enum MenuSelected mnu_sel = BL;
enum dsMenuSelected ds_mnu_sel = U_DISP;
int16_t enc_cnt = 0;
uint8_t btn_cnt = 0;
char* ptr; // Point to converted to char floats for LCD displaying
bool on_off = false; // disable/enable power at output 0/1
bool mod_sel_MS = false; // device is slave/device is master 0/1
int8_t mem_sel = 0; // selected memory set from 0 to 9
int16_t lcd_pwm_bl = 200; //max value
float uSP = 5.0; // set point for output voltage
float iSP = 1.0; // set point for output current
char onTd100 = '0', onTd10 = '0', onTd1 = '0',
	 onTh10 = '0', onTh1 = '0', onTm10 = '0', onTm1 = '0',
	 onTs10 = '0', onTs1 = '0';// On time
uint16_t uint_spU=0, uint_spI=0; // values sent to DACs
uint8_t masterTxBuf[I2C_BUFF_LEN];//0-On/Off,1-4 uSP,5-8 iSP, 12-15-CRC
uint8_t masterRxBuf[I2C_BUFF_LEN];//0-On/Off,1-4 outU,5-8 outI, 12-15-CRC
uint8_t slaveTxBuf[I2C_BUFF_LEN];//0-On/Off,1-4 outU,5-8 outI, 12-15-CRC
uint8_t slaveRxBuf[I2C_BUFF_LEN];//0-On/Off,1-4 uSP,5-8 iSP, 12-15-CRC
uint32_t tempCRC;
bool on_off_slave_state = false; // state of slave controlled device
bool i2cError = false; // help flag to indicate I2C CRC Error
bool redraw_ST = false; // to redraw device state in slave mode
bool redraw_uSP = false; // to redraw U set point in slave mode
bool redraw_iSP = false; // to redraw I set point in slave mode
//uint8_t tmpCRC;// temporary value for CRC calculation
//uint32_t masterTxCounter = 0;
//uint32_t masterRxCounter = 0;
//uint32_t slaveTxCounter = 0;
//uint32_t slaveRxCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void menu_handler(void);
void device_settings(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
	  	  	  	  	  	  	  	  	  	  	  	  	  Error_Handler();

  /* Start ADC group regular conversion by DMA*/
  if (HAL_ADC_Start_DMA(&hadc,(uint32_t *)adc_RAW,3) != HAL_OK)
	  	  	  	  	  	  	  	  	  	  	  	  	  Error_Handler();

  /* Start TIM3 as encoder counter */
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
	  	  	  	  	  	  	  	  	  	  	  	  	  Error_Handler();

  /*Run TIM6 to do 100uS interrupts for button and encoder services */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)		  Error_Handler();

  /*Start TIM1 to generate PWM output for LCD Back Light */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
	  	  	  	  	  	  	  	  	  	  	  Error_Handler();

  /* Start Back Light at max Level=200 */
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, lcd_pwm_bl);

//  LCD_Init( lcd_ScanDir ); // LCD initialization
  ST7735_Init();
  ST7735_SetRotation(1);
  if(!HAL_GPIO_ReadPin(SW_T_GPIO_Port,SW_T_Pin)) device_settings();//setup screen
  btn = PRESS_NORM;// to activate V menu on start

  ST7735_DrawImage(0, 0, 160, 128, (uint16_t*) logo_160x128);//draw welcome screen
  HAL_Delay(2000);
  ST7735_DrawString(0,0,"So Long, and Thanks...",Font_7x10,BRRED,BLACK);
  HAL_Delay(2000);
  ST7735_DrawString(25,118,"...for all the Fish",Font_7x10,BRRED,BLACK);
  HAL_Delay(2000);

  draw_main_st(BLACK, WHITE); // Main static screen
  if(*( uint32_t *)(SCALE_U_ADDR) != 0xFFFFFFFF)
	  scaleU=*( float *)(SCALE_U_ADDR); // load user set scale constant
  if(*( uint32_t *)(SCALE_I_ADDR) != 0xFFFFFFFF)
	  scaleI=*( float *)(SCALE_I_ADDR); // load user set scale constant
  if(*( uint32_t *)(SCALE_U_SP_ADDR) != 0xFFFFFFFF)
	  scaleUsp=*( float *)(SCALE_U_SP_ADDR); // load user set scale constant
  if(*( uint32_t *)(SCALE_I_SP_ADDR) != 0xFFFFFFFF)
	  scaleIsp=*( float *)(SCALE_I_SP_ADDR); // load user set scale constant
  if(*( uint32_t *)(MEM_SEL_ARR_ADDR) != 0xFFFFFFFF)
	  uSP=*( float *)(MEM_SEL_ARR_ADDR); // load user set for U@M0
  if(*( uint32_t *)(MEM_SEL_ARR_ADDR+4) != 0xFFFFFFFF)
	  iSP=*( float *)(MEM_SEL_ARR_ADDR+4); // load user set for I@M0
  /*BEGIN: Draw on LCD set points taken from FLASH Memory if they are valid */

	ptr = float_to_char(uSP, float_for_LCD);
	if(uSP<1)
	{
		ST7735_DrawString(5,84," 0",Font_11x18,WHITE,BLACK);
		ST7735_DrawString(27,84,ptr,Font_11x18,WHITE,BLACK);
	}
	else
	{
		if(uSP<10)
		{
			ST7735_DrawString(5,84," ",Font_11x18,WHITE,BLACK);
			ST7735_DrawString(16,84,ptr,Font_11x18,WHITE,BLACK);
		}
		else ST7735_DrawString(5,84,ptr,Font_11x18,WHITE,BLACK);
	}
	ST7735_DrawString( 60,84,"0V",Font_11x18,WHITE,BLACK);

	ptr = float_to_char(iSP, float_for_LCD);
	if(iSP<1)
	{
		ST7735_DrawString(89,84,"0",Font_11x18,WHITE,BLACK);
		ST7735_DrawString(100,84,ptr,Font_11x18,WHITE,BLACK);
	}
	else ST7735_DrawString(89,84,ptr,Font_11x18,WHITE,BLACK);
	ST7735_DrawString(144,84,"A",Font_11x18,WHITE,BLACK);
  /*END: Draw on LCD set points taken from FLASH Memory if they are valid */
  constU = (vdd*scaleU)/4095; //calculated only once to avoid...
  constI = (vdd*scaleI)/4095; //....circular repeated calculation
  //Enable by DEFINE to tests only
#ifdef DAC_IO_TEST
  uint_spI=0;
  uint_spU=0;
  while(1)
  {
	  i_DAC10_Set(uint_spI);
	  v_DAC10_Set(uint_spU);
	  uint_spI++;
	  uint_spU++;
	  if(uint_spI > 0x3ff) uint_spI =  0;
	  if(uint_spU > 0x3ff) uint_spU = 0;
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  get_adcs(adc_RAW, &temp_MCU, &outU, &outI, constU, constI, vdd);
	  if((enc != NO_TRN)||(btn != NO_PRESS)) menu_handler();
 	  get_time(hrtc, &onTd100, &onTd10, &onTd1 , &onTh10, &onTh1, &onTm10, &onTm1,
 			  &onTs10, &onTs1, on_off);
 	  draw_main_dy(ptr, float_for_LCD, on_off, outU, outI, onTd100, onTd10, onTd1, onTh10,
			  onTh1, onTm10, onTm1, onTs10, onTs1, temp_MCU);
 	  if(temp_MCU < 85.0) //no over hating
	  {
 			  uint_spI = (uint16_t)(scaleIsp   * iSP);
 			  i_DAC10_Set(uint_spI);
 			  if(on_off) // output is POWERED
 			  {
 				  uint_spU = (uint16_t)(scaleUsp * uSP);
 				   v_DAC10_Set(uint_spU);

 			  }
 			  else // output is UNPOWERED
 			  {
 				  v_DAC10_Set(0);
 				  //  i_DAC10_Set(0);
 			  }
	  }
 	  else //over heating, output UNPOWERED
 	  {
 		  v_DAC10_Set(0);
 		  i_DAC10_Set(0);
 		  on_off = false;
 	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*) &slaveRxBuf,I2C_BUFF_LEN);
	  if(mod_sel_MS)// device is in master mode
	  {
		  masterTxBuf[0]=(uint8_t)on_off;
		  memcpy(&masterTxBuf[1], &uSP, sizeof(uSP));
		  memcpy(&masterTxBuf[5], &iSP, sizeof(iSP));
		  tempCRC = HAL_CRC_Calculate(&hcrc,(uint32_t *)masterTxBuf, (I2C_BUFF_LEN/4)-1);
		  memcpy(&masterTxBuf[I2C_BUFF_LEN-4],&tempCRC,sizeof(tempCRC));
		  HAL_I2C_Master_Transmit(&hi2c1, (I2C_SLAVE_ADDRESS << 1),
				  	  	  	  	  	  	  	 (uint8_t*) &masterTxBuf, I2C_BUFF_LEN, 500);
		  HAL_Delay(5);//Give some time to slave to compute received
		  // Dummy i2c receiving for computability with PC mode interface
		  HAL_I2C_Master_Receive(&hi2c1, (I2C_SLAVE_ADDRESS << 1),
				  	  	  	  	  	  	  	  (uint8_t*) &masterRxBuf, I2C_BUFF_LEN, 500);
	  }
	  if(i2cError)
	  {
 		  v_DAC10_Set(0);
 		  i_DAC10_Set(0);
 		  on_off = false;
 		  ST7735_DrawString(0,52,"i2c communication error",Font_11x18,WHITE, RED);
 		  while(1);//Lock device after CRC Error
	  }
	  if(redraw_ST)
	  {
		  if(on_off) ST7735_DrawString(124,2," ON",Font_11x18,WHITE,GREEN);
		  else ST7735_DrawString(124,2,"OFF",Font_11x18,WHITE,RED);
		  redraw_ST = false;
	  }
	  if(redraw_uSP)
	  {
		  ptr = float_to_char(uSP, float_for_LCD);
		  if(uSP<1)
		  {
			  ST7735_DrawString(5,84," 0",Font_11x18,WHITE,BLACK);
			  ST7735_DrawString(27,84,ptr,Font_11x18,WHITE,BLACK);
		  }
		  else
		  {
			  if(uSP<10)
			  {
				  ST7735_DrawString(5,84," ",Font_11x18,WHITE,BLACK);
				  ST7735_DrawString(16,84,ptr,Font_11x18,WHITE,BLACK);
			  }
			  else ST7735_DrawString(5,84,ptr,Font_11x18,WHITE,BLACK);
		  }
		  ST7735_DrawString( 60,84,"0V",Font_11x18,WHITE,BLACK);
		  redraw_uSP = false;
	  }
	  if(redraw_iSP)
	  {
		  ptr = float_to_char(iSP, float_for_LCD);
		  if(iSP<1)
		  {
			  ST7735_DrawString(89,84,"0",Font_11x18,WHITE,BLACK);
			  ST7735_DrawString(100,84,ptr,Font_11x18,WHITE,BLACK);
		  }
		  else ST7735_DrawString(89,84,ptr,Font_11x18,WHITE,BLACK);
		  ST7735_DrawString(144,84,"A",Font_11x18,WHITE,BLACK);
		  redraw_iSP = false;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 90;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, I_DAC7_Pin|I_DAC8_Pin|I_DAC9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, I_DAC3_Pin|I_DAC5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, I_DAC4_Pin|LCD_RST_Pin|LCD_DC_Pin|LCD_CS_Pin
                          |V_DAC4_Pin|V_DAC3_Pin|V_DAC2_Pin|V_DAC1_Pin
                          |I_DAC0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, V_DAC0_Pin|V_DAC6_Pin|V_DAC7_Pin|V_DAC8_Pin
                          |V_DAC9_Pin|V_DAC5_Pin|I_DAC1_Pin|I_DAC2_Pin
                          |I_DAC6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : I_DAC7_Pin I_DAC8_Pin I_DAC9_Pin */
  GPIO_InitStruct.Pin = I_DAC7_Pin|I_DAC8_Pin|I_DAC9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I_DAC3_Pin I_DAC5_Pin */
  GPIO_InitStruct.Pin = I_DAC3_Pin|I_DAC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I_DAC4_Pin LCD_RST_Pin LCD_DC_Pin LCD_CS_Pin
                           V_DAC4_Pin V_DAC3_Pin V_DAC2_Pin V_DAC1_Pin
                           I_DAC0_Pin */
  GPIO_InitStruct.Pin = I_DAC4_Pin|LCD_RST_Pin|LCD_DC_Pin|LCD_CS_Pin
                          |V_DAC4_Pin|V_DAC3_Pin|V_DAC2_Pin|V_DAC1_Pin
                          |I_DAC0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_T_Pin */
  GPIO_InitStruct.Pin = SW_T_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_T_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : V_DAC0_Pin V_DAC6_Pin V_DAC7_Pin V_DAC8_Pin
                           V_DAC9_Pin V_DAC5_Pin I_DAC1_Pin I_DAC2_Pin
                           I_DAC6_Pin */
  GPIO_InitStruct.Pin = V_DAC0_Pin|V_DAC6_Pin|V_DAC7_Pin|V_DAC8_Pin
                          |V_DAC9_Pin|V_DAC5_Pin|I_DAC1_Pin|I_DAC2_Pin
                          |I_DAC6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //every 100mS
{
    if (htim->Instance==TIM6) //check if the interrupt comes from TIM6
        {
    		enc_cnt = TIM3->CNT; TIM3->CNT = 0;
    		if (enc_cnt >0)
    		{
    			if(enc_cnt <= TRN_SLOW_NORM) enc = INC_TRN_SLOW;
    			else enc = INC_TRN_NORM;
    			if(enc_cnt > TRN_NORM_FAST) enc = INC_TRN_FAST;
    		}
    		if (enc_cnt < 0)
    		{
    			if(enc_cnt >= (-TRN_SLOW_NORM)) enc = DEC_TRN_SLOW;
    			else enc = DEC_TRN_NORM;
    			if(enc_cnt < (-TRN_NORM_FAST)) enc = DEC_TRN_FAST;
    		}

    		if (!HAL_GPIO_ReadPin(SW_T_GPIO_Port,SW_T_Pin)) // Button is pressed
    		{
    			if(btn == NO_PRESS)
    				{
    					btn = FIRST_EDGE;
    					btn_cnt =0;
    				}
    			else ++btn_cnt;
    		}
    		else // Button is released
    		{
    			if (btn == FIRST_EDGE)
    			{
    				if(btn_cnt < 5) btn = PRESS_NORM; // less than 500mS press
    				else btn = PRESS_LONG;
    			}
    		}
        }
}

void menu_handler(void)
{
	if(enc != NO_TRN)
	{
		switch(mnu_sel)
		{
		case ON_OFF:
		{
			if(enc == INC_TRN_SLOW)
			{
				ST7735_DrawString(124,2," ON",Font_11x18,WHITE,GREEN);
				on_off = 1;
			}
			if(enc == DEC_TRN_SLOW)
			{
				ST7735_DrawString(124,2,"OFF",Font_11x18,WHITE,RED);
				on_off = 0;
			}
			break;
		}
		case MODE:
		{
			if(enc == INC_TRN_SLOW)
			{
				ST7735_DrawString(124,21," SL",Font_11x18,BLACK,GBLUE);
				mod_sel_MS = false;
			}
			if(enc == DEC_TRN_SLOW)
			{
				ST7735_DrawString(124,21," MS",Font_11x18,BLACK,BRRED);
				mod_sel_MS = true ;
			}
			break;
		}
		case MEM:
		{
				HAL_Delay(200);
				if(enc == INC_TRN_SLOW)
				{
					mem_sel=mem_sel+1;
					if (mem_sel > 9) mem_sel=9;
				}
				if(enc == DEC_TRN_SLOW)
				{
					mem_sel=mem_sel-1;
					if (mem_sel < 0) mem_sel=0;
				}
				float_for_LCD[0]=0x30+mem_sel;
				float_for_LCD[1]=0;
				ST7735_DrawString(124,40," M",Font_11x18,BLACK,GRAY);
				ST7735_DrawString(146 ,40,float_for_LCD,Font_11x18,BLACK,GRAY);
			break;
		}
		case BL:
		{
			if(enc == INC_TRN_SLOW) lcd_pwm_bl = lcd_pwm_bl+1;
			if(enc == INC_TRN_NORM) lcd_pwm_bl = lcd_pwm_bl+10;
			if(enc == INC_TRN_FAST) lcd_pwm_bl = lcd_pwm_bl+50;
			if(enc == DEC_TRN_SLOW) lcd_pwm_bl = lcd_pwm_bl-1;
			if(enc == DEC_TRN_NORM) lcd_pwm_bl = lcd_pwm_bl-10;
			if(enc == DEC_TRN_FAST) lcd_pwm_bl = lcd_pwm_bl-50;
			if (lcd_pwm_bl < 0) lcd_pwm_bl = 0;
			if (lcd_pwm_bl > 200) lcd_pwm_bl = 200;
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, lcd_pwm_bl);
			if (lcd_pwm_bl != 200)
				ST7735_DrawString(124,59,"LGT",Font_11x18,WHITE,MAGENTA);
			else
				ST7735_DrawString(124,59,"MAX",Font_11x18,WHITE,MAGENTA);
			break;
		}
		case U:
		{
			if(enc == INC_TRN_SLOW) uSP=uSP+0.01; //0.03
			if(enc == INC_TRN_NORM) uSP=uSP+0.3;
			if(enc == INC_TRN_FAST) uSP=uSP+3;
			if(enc == DEC_TRN_SLOW) uSP=uSP-0.01; //0.03
			if(enc == DEC_TRN_NORM) uSP=uSP-0.3;
			if(enc == DEC_TRN_FAST) uSP=uSP-3;
			if(uSP>30) uSP=30; // no more 30V
			if(uSP<0) uSP=0; // no less 0V
			ptr = float_to_char(uSP, float_for_LCD);
			if(uSP<1)
			{
				ST7735_DrawString(5,84," 0",Font_11x18,WHITE,BLACK);
				ST7735_DrawString(27,84,ptr,Font_11x18,WHITE,BLACK);
			}
			else
			{
				if(uSP<10)
				{
					ST7735_DrawString(5,84," ",Font_11x18,WHITE,BLACK);
					ST7735_DrawString(16,84,ptr,Font_11x18,WHITE,BLACK);
				}
				else ST7735_DrawString(5,84,ptr,Font_11x18,WHITE,BLACK);
			}
			ST7735_DrawString( 60,84,"0V",Font_11x18,WHITE,BLACK);
			break;
		}
		case I:
		{
			if(enc == INC_TRN_SLOW) iSP=iSP+0.001; //0.005
			if(enc == INC_TRN_NORM) iSP=iSP+0.05;
			if(enc == INC_TRN_FAST) iSP=iSP+0.5;
			if(enc == DEC_TRN_SLOW) iSP=iSP-0.001; //0.005
			if(enc == DEC_TRN_NORM) iSP=iSP-0.05;
			if(enc == DEC_TRN_FAST) iSP=iSP-0.5;
			if(iSP>4.6) iSP=4.6; //no more 4.6A
			if(iSP<0) iSP=0; // no less 0A
			ptr = float_to_char(iSP, float_for_LCD);
			if(iSP<1)
			{
				ST7735_DrawString(89,84,"0",Font_11x18,WHITE,BLACK);
				ST7735_DrawString(100,84,ptr,Font_11x18,WHITE,BLACK);
			}
			else ST7735_DrawString(89,84,ptr,Font_11x18,WHITE,BLACK);
			ST7735_DrawString(144,84,"A",Font_11x18,WHITE,BLACK);
			break;
		}
		}
		enc=NO_TRN;
	}

	if(btn != (NO_PRESS || FIRST_EDGE))
	{
		if(btn == PRESS_NORM)
		{
			switch(mnu_sel)
			{
			case ON_OFF: //Power on, off
			{
				ST7735_DrawRect
					(123, 1, 35, 20, BLACK);
				ST7735_DrawRect
					(123,20, 35, 20, WHITE);
				mnu_sel = MODE;
				break;
			}
			case MODE:  //Constant U or constant I
			{
				ST7735_DrawRect
					(123,20, 35, 20, BLACK);
				ST7735_DrawRect
					(123,39, 35, 20, WHITE);
				mnu_sel = MEM;
				break;
			}
			case MEM: //Memory settings
			{
				ST7735_DrawRect
					(123,39, 35, 20, BLACK);
				ST7735_DrawRect
					(123,58, 35, 20, WHITE);
				mnu_sel = BL;
				break;
			}
			case BL: //Back Light setup
			{
				ST7735_DrawRect
					(123,58, 35, 20, BLACK);
				ST7735_DrawRect
					(4,83, 79, 20, WHITE);
				mnu_sel = U;
				break;
			}
			case U: //Voltage set point
			{
				ST7735_DrawRect
					(4,83, 79, 20, BLACK);
				ST7735_DrawRect
					(87,83, 69, 20, WHITE);
				mnu_sel = I;
				break;
			}
			case I: //Current set point
			{
				ST7735_DrawRect
					(87,83, 69, 20, BLACK);
				ST7735_DrawRect
					(123, 1, 35, 20, WHITE);
				mnu_sel = ON_OFF;
				break;
			}
			}
		}
		if(btn == PRESS_LONG)
		{
			if(mnu_sel != MEM)
			{
				if(on_off)
				{
					ST7735_DrawString(124,2,"OFF",Font_11x18,WHITE,RED);
					on_off = false;
				}
				else
				{
					ST7735_DrawString(124,2," ON",Font_11x18,WHITE,GREEN);
					on_off = true;
				}
			}
			else // process memory pre-seting selection
			{
				if(on_off) //write SP as M to FLASH
				{
					save_settings(scaleU, scaleI, scaleUsp, scaleIsp, mem_sel, uSP, iSP);
				}
				else //get M values from FLASH
				{
					if(*( uint32_t *)(MEM_SEL_ARR_ADDR+(8*mem_sel)) != 0xFFFFFFFF)
						uSP=*( float *)(MEM_SEL_ARR_ADDR+(8*mem_sel));
					ptr = float_to_char(uSP, float_for_LCD);
					if(uSP<1)
					{
						ST7735_DrawString(5,84," 0",Font_11x18,WHITE,BLACK);
						ST7735_DrawString(27,84,ptr,Font_11x18,WHITE,BLACK);
					}
					else
					{
						if(uSP<10)
						{
							ST7735_DrawString(5,84," ",Font_11x18,WHITE,BLACK);
							ST7735_DrawString(16,84,ptr,Font_11x18,WHITE,BLACK);
						}
						else ST7735_DrawString(5,84,ptr,Font_11x18,WHITE,BLACK);
					}
					ST7735_DrawString( 60,84,"0V",Font_11x18,WHITE,BLACK);

					if(*( uint32_t *)(MEM_SEL_ARR_ADDR+(8*mem_sel)+4) != 0xFFFFFFFF)
						iSP=*( float *)(MEM_SEL_ARR_ADDR+(8*mem_sel)+4);
					ptr = float_to_char(iSP, float_for_LCD);
					if(iSP<1)
					{
						ST7735_DrawString(89,84,"0",Font_11x18,WHITE,BLACK);
						ST7735_DrawString(100,84,ptr,Font_11x18,WHITE,BLACK);
					}
					else ST7735_DrawString(89,84,ptr,Font_11x18,WHITE,BLACK);
					ST7735_DrawString(144,84,"A",Font_11x18,WHITE,BLACK);
				}
			}
		}
		btn=NO_PRESS;
	}
}

void device_settings(void)
{
	while(!HAL_GPIO_ReadPin(SW_T_GPIO_Port,SW_T_Pin)); // wait Button is released
    HAL_Delay(200);
	btn=NO_PRESS;
	ST7735_FillScreen(BLACK);

	float scaleU_tmp = scaleU, scaleI_tmp = scaleI,
		scaleUsp_tmp = scaleUsp, scaleIsp_tmp = scaleIsp;

	ST7735_DrawString(1,2,"C>Udsp:",Font_11x18,WHITE, BLACK);
	ptr = float_to_char(scaleU, float_for_LCD);
	ST7735_DrawString(81,2,ptr,Font_11x18,WHITE, BLACK);

	ST7735_DrawString(1,22,"C>Idsp:",Font_11x18,WHITE, BLACK);
	ptr = float_to_char(scaleI, float_for_LCD);
	ST7735_DrawString(81,22,ptr,Font_11x18,WHITE, BLACK);

	ST7735_DrawString(1,42,"C>Ustp:",Font_11x18,WHITE, BLACK);
	ptr = float_to_char(scaleUsp, float_for_LCD);
	ST7735_DrawString(81,42,ptr,Font_11x18,WHITE, BLACK);

	ST7735_DrawString(1,62,"C>Istp:",Font_11x18,WHITE, BLACK);
	ptr = float_to_char(scaleIsp, float_for_LCD);
	ST7735_DrawString(81,62,ptr,Font_11x18,WHITE, BLACK);

	ST7735_DrawString(3,107,"EXIT SAVE&EXIT",Font_11x18,WHITE, BLACK);

	ST7735_DrawRect(79, 0, 80, 21, WHITE);


	while ((btn != PRESS_LONG) || ((ds_mnu_sel != EXIT)&&(ds_mnu_sel != SAVE_EXIT)))
	{
		if(btn == PRESS_NORM)
		{
			 switch(ds_mnu_sel)
			{
			case U_DISP:
			{
				ST7735_DrawRect(79, 0, 80, 21, BLACK);
				ST7735_DrawRect(79, 20, 80, 21, WHITE);
				ds_mnu_sel = I_DISP;
				break;
			}
			case I_DISP:
			{
				ST7735_DrawRect(79, 20, 80, 21, BLACK);
				ST7735_DrawRect(79, 40, 80, 21, WHITE);
				ds_mnu_sel = U_SETP;
				break;
			}
			case U_SETP:
			{
				ST7735_DrawRect(79, 40, 80, 21, BLACK);
				ST7735_DrawRect(79, 60, 80, 21, WHITE);
				ds_mnu_sel = I_SETP;
				break;
			}
			case I_SETP:
			{
				ST7735_DrawRect(79, 60, 80, 21, BLACK);
				ST7735_DrawRect(1, 105, 49, 21, WHITE);
				ds_mnu_sel = EXIT;
				break;
			}
			case EXIT:
			{
				ST7735_DrawRect(1, 105, 49, 21, BLACK);
				ST7735_DrawRect(55, 105, 105 , 21, WHITE);
				ds_mnu_sel = SAVE_EXIT;
				break;
			}
			case SAVE_EXIT:
			{
				ST7735_DrawRect(55, 105, 105 , 21, BLACK);
				ST7735_DrawRect(79, 0, 80, 21, WHITE);
				ds_mnu_sel = U_DISP;
				break;
			}
			}
			btn=NO_PRESS;
		}
		if((btn == PRESS_LONG)&&
				((ds_mnu_sel == U_DISP)||(ds_mnu_sel == I_DISP)||
						(ds_mnu_sel == U_SETP)||(ds_mnu_sel == I_SETP)))
																btn=NO_PRESS;
		if(enc != NO_TRN)
		{
			float_for_LCD[CHAR_BUFF_SIZE]= 0;//hard stop of converted string
			if(ds_mnu_sel == U_DISP)
			{
				if(enc == INC_TRN_SLOW) scaleU = scaleU+0.001;
				if((enc == INC_TRN_NORM)||(enc == INC_TRN_NORM)) scaleU = scaleU+0.1;
				if(enc == DEC_TRN_SLOW) scaleU = scaleU-0.001;
				if((enc == DEC_TRN_NORM)||(enc == DEC_TRN_NORM)) scaleU = scaleU-0.1;
				if (scaleU > SCALE_U_MAX) scaleU = SCALE_U_MAX;
				if (scaleU < SCALE_U_MIN) scaleU = SCALE_U_MIN;
				ptr = float_to_char(scaleU, float_for_LCD);
				ST7735_DrawString(81,2,ptr,Font_11x18,WHITE, BLACK);
				if (scaleU < 10.0)
					ST7735_DrawString(136,2," ",Font_11x18,WHITE, BLACK);

			}
			if(ds_mnu_sel == I_DISP)
			{
				if(enc == INC_TRN_SLOW) scaleI = scaleI+0.001;
				if((enc == INC_TRN_NORM)||(enc == INC_TRN_NORM)) scaleI = scaleI+0.1;
				if(enc == DEC_TRN_SLOW) scaleI = scaleI-0.001;
				if((enc == DEC_TRN_NORM)||(enc == DEC_TRN_NORM)) scaleI = scaleI-0.1;
				if (scaleI > SCALE_I_MAX) scaleI = SCALE_I_MAX;
				if (scaleI < SCALE_I_MIN) scaleI = SCALE_I_MIN;
				ptr = float_to_char(scaleI, float_for_LCD);
				if (scaleI < 1.0)
				{
					ptr--;
					*ptr = '0';
				}
				ST7735_DrawString(81,22,ptr,Font_11x18,WHITE, BLACK);
			}
			if(ds_mnu_sel == U_SETP)
			{
				if(enc == INC_TRN_SLOW) scaleUsp = scaleUsp+0.001;
				if((enc == INC_TRN_NORM)||(enc == INC_TRN_NORM)) scaleUsp = scaleUsp+0.1;
				if(enc == DEC_TRN_SLOW) scaleUsp = scaleUsp-0.001;
				if((enc == DEC_TRN_NORM)||(enc == DEC_TRN_NORM)) scaleUsp = scaleUsp-0.1;
				if (scaleUsp > SCALE_U_SP_MAX) scaleUsp = SCALE_U_SP_MAX;
				if (scaleUsp < SCALE_U_SP_MIN) scaleUsp = SCALE_U_SP_MIN;
				ptr = float_to_char(scaleUsp, float_for_LCD);
				ST7735_DrawString(81,42,ptr,Font_11x18,WHITE, BLACK);
			}
			if(ds_mnu_sel == I_SETP)
			{
				if(enc == INC_TRN_SLOW) scaleIsp = scaleIsp+0.001;
				if((enc == INC_TRN_NORM)||(enc == INC_TRN_NORM)) scaleIsp = scaleIsp+0.1;
				if(enc == DEC_TRN_SLOW) scaleIsp = scaleIsp-0.001;
				if((enc == DEC_TRN_NORM)||(enc == DEC_TRN_NORM)) scaleIsp = scaleIsp-0.1;
				if (scaleIsp > SCALE_I_SP_MAX) scaleIsp = SCALE_I_SP_MAX;
				if (scaleIsp < SCALE_I_SP_MIN) scaleIsp = SCALE_I_SP_MIN;
				ptr = float_to_char(scaleIsp, float_for_LCD);
				ST7735_DrawString(81,62,ptr,Font_11x18,WHITE, BLACK);
			}
			enc=NO_TRN;
		}
	}
	if (ds_mnu_sel == SAVE_EXIT)
		save_settings(scaleU, scaleI, scaleUsp, scaleIsp, 10,0,0);
	else
	{
		scaleU = scaleU_tmp; scaleI = scaleI_tmp;
		scaleUsp = scaleUsp_tmp; scaleIsp = scaleIsp_tmp;
	}
	enc=NO_TRN;
	btn=NO_PRESS;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  bool temp_b;
  float temp_f;
  if (hi2c == &hi2c2)
  {
 	  slaveTxBuf[0]=(uint8_t)on_off;
	  memcpy(&slaveTxBuf[1], &outU, sizeof(outU));
	  memcpy(&slaveTxBuf[5], &outI, sizeof(outI));
	  tempCRC = HAL_CRC_Calculate(&hcrc,(uint32_t *)slaveTxBuf, (I2C_BUFF_LEN/4)-1);
	  memcpy(&slaveTxBuf[I2C_BUFF_LEN-4],&tempCRC,sizeof(tempCRC));
	  HAL_I2C_Slave_Transmit_IT(&hi2c2, (uint8_t*) &slaveTxBuf, I2C_BUFF_LEN);
	  memcpy(&tempCRC, &slaveRxBuf[I2C_BUFF_LEN-4], sizeof(tempCRC));
	  if(tempCRC == HAL_CRC_Calculate(&hcrc,(uint32_t *)slaveRxBuf, (I2C_BUFF_LEN/4)-1))
	  {
		  temp_b = (slaveRxBuf[0] != 0) ? true : false;
		  if (temp_b != on_off) // if received value differ from current update LCD
		  {
			  on_off = temp_b;
			  redraw_ST = true;
		  }
		  memcpy(&temp_f, &slaveRxBuf[1], sizeof(temp_f));
		  if (temp_f != uSP) // if received value differ from current update LCD
		  {
			  uSP = temp_f;
			  redraw_uSP = true;
		  }
		  memcpy(&temp_f, &slaveRxBuf[5], sizeof(temp_f));
		  if (temp_f != iSP)
		  {
			  iSP = temp_f;
			  redraw_iSP = true;
		  }
	  }
	  else i2cError = true;
  }
}

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

#ifdef  USE_FULL_ASSERT
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
