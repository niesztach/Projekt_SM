/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "bmp2_config.h"
#include "pid_regulator.h"
#include "lcd_i2c.h"
#include <stdbool.h>
#include "fatfs_sd.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//float duty=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//odczyt z karty microsd
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char bufffer[100];
int time_counter=-1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double temp = 0.0f;     // [degC]
unsigned int temp_int;	// [mdegC]
double press = 0.0f;    // [hPa]
unsigned int press_int; // [Pa]
char word[]="000";
unsigned int zadane=0;
unsigned int zadane_rezystora=0;
unsigned int zadane_wiatraka=0;
float current_temp=0;
float fanControlTest=-1.0;  //tymczasowa zmienna pomocnicza do weryfikacji poprawnosci dzialania PID
float fanControlTestDriven=-1.0;
unsigned int zadane_obiektu = 27000;
unsigned int powitanie = 1;
uint16_t readValue_potencjometr=0;
// wyświetlacz
struct lcd_disp disp;
unsigned int test = 0;
volatile uint8_t sd_card_operation_flag = 0;
int value;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
pid_t2 pid1={.param.Kp=1.2,.param.Ki=0.002, .param.Kd=0,.param.dt=1.0, .previous_error=0, .previous_integral=0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static unsigned int cnt = 0;
	    if(htim == &htim2)
	    {
	        cnt++;
	        BMP2_ReadData(&bmp2dev, NULL, &temp); //odczyt temperatury z czujnika

	        temp_int = 1000*temp;

	        if(cnt == 4)
	        {
	            cnt = 0;
	            uint8_t tx_buffer[128];
	            int tx_msg_len = sprintf((char*)tx_buffer, "%d\r\n",
	                                     temp_int / 1000);

	            HAL_UART_Transmit(&huart3, tx_buffer, tx_msg_len, HAL_MAX_DELAY);

	            sd_card_operation_flag = 1;	        }
  }
  //pid
  if(htim == &htim7){

  		 current_temp = temp_int;
  	}else{
  		float pwm_duty_f= (999.0*calculate_discrete_pid(&pid1,zadane_obiektu,current_temp));
  		fanControlTest=pwm_duty_f;
  		uint16_t pwm_duty=0;
  		//saturacja
  		if(pwm_duty_f<0)pwm_duty=0;else
  		if(pwm_duty_f>999.0)pwm_duty=999;else
  			pwm_duty=(uint16_t)pwm_duty_f;
  		fanControlTestDriven = pwm_duty;

  		if(temp_int -600 > zadane_obiektu) __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 700);
  		if(temp_int -200 < zadane_obiektu)__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0);


  		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm_duty);

  	}
}

void wypelnienie1(uint16_t duty)
{
	duty = duty*10;
	zadane_rezystora=duty;
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
}

void wypelnienie2(uint16_t duty)
{
	duty = duty*10;
	zadane_wiatraka=duty;
	//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t fill=0;

	if(word[0] == 'r')
	{
		fill = ((word[1] - '0') * 10 + (word[2] - '0'));
		wypelnienie1(fill);
	}

	if(word[0] == 'w')
	{
		fill = ((word[1] - '0') * 10 + (word[2] - '0'));
		wypelnienie2(fill);
	}
	HAL_UART_Receive_IT(&huart3, word, 3);
}

void display_function()
{
	if(powitanie)
	{
		sprintf((char*)disp.f_line, "Dzie%c dobry %c", '\x4', '\x5');
		sprintf((char*)disp.s_line, "smacznej kawusi");
		lcd_display(&disp);
		HAL_Delay(1000);
		powitanie = 0;
	}
	else
	{
		sprintf((char*)disp.f_line, "T. akt.:%d.%02d%cC", temp_int / 1000, temp_int % 1000, '\x7');
		sprintf((char*)disp.s_line, "T. zad.:%d.%02d%cC", zadane_obiektu / 1000, zadane_obiektu % 1000, '\x8');
		lcd_display(&disp);
	}

	HAL_Delay(2000);

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
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  BMP2_Init(&bmp2dev);// inicjalizacja czujnika
  HAL_TIM_Base_Start_IT(&htim2);// uruchomienie timerow
  HAL_TIM_Base_Start_IT(&htim7);// uruchomienie timerow
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  HAL_UART_Receive_IT(&huart3, word, 3);
//wyswietlacz
  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);
  lcd_prog(&disp);
  //potencjometr
  //HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start(&hadc1);
	  display_function();
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  readValue_potencjometr = HAL_ADC_GetValue(&hadc1);
	  zadane_obiektu = readValue_potencjometr*5.36 +18000.0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (sd_card_operation_flag) {
	         sd_card_operation_flag = 0; // Zeruj flagę
	         time_counter++; //czas probki

	         if (f_mount(&fs, "", 0) == FR_OK) {
	             if (f_open(&fil, "log-file.csv", FA_OPEN_APPEND | FA_WRITE) == FR_OK) { //dziala na istniejacym pliku
//	        	 if (f_open(&fil, "log-file.csv", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) { //czysci plik przed zadzialaniem
	                 char log_buffer[128];
	                 int log_length = sprintf(log_buffer, "%d,Temperature:,%u.%03u,DegC\r\n",
	                                          time_counter, temp_int / 1000, temp_int % 1000);


	                 UINT bytes_written;
	                 FRESULT write_result = f_write(&fil, log_buffer, log_length, &bytes_written);

	                 if (write_result == FR_OK && bytes_written == log_length) {
	                     // Dane zostały pomyślnie zapisane do pliku
	                     // Opcjonalnie, możesz dodać tutaj dodatkowy kod, np. logowanie sukcesu
	                	 value ++;
	                 } else {
	                     // Wystąpił błąd podczas zapisu
	                     // Opcjonalnie, możesz dodać tutaj kod obsługi błędów
	                	value = value *10;
	                 }

	                 f_close(&fil);
	             }
	             f_mount(NULL, "", 0);
	         }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
