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
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tusb.h"
#include "tusb_config.h"
#include "board_api.h"
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
int octave_num = 2;
bool flag1 = false;
bool flag2 = false;
uint32_t ADC1_VAL[ 8 ]; // one element for each ADC channel (one device)
uint32_t ADC2_VAL[ 8 ]; // one element for each ADC channel (one device)
uint16_t KEYPRESS = 0;
uint16_t KEYPRESSED = 0;
// for test case (from tinyusb example)
uint32_t note_pos = 0;

// Store example melody as an array of note values
uint8_t note_sequence[] =
{
  74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
  74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
  56,61,64,68,74,78,81,86,90,93,98,102
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Separate into individual file eventually??
void READ_KEYPRESS(uint32_t adc1_val[], uint32_t adc2_val[]);
uint8_t HALL_TO_DAC(uint32_t adc1_val[], uint32_t adc2_val[], int octave_num);
uint8_t DAC_TO_MIDI(uint8_t val);
void MIDI_TASK(int octave_num);
void midi_task(void);

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
  /* Disable System Clock Init - Use Custom Function */
  //SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  board_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize tinyUSB */
  tusb_init();
  tud_task(); //necessary?

  Init_ADC();
  Reset_DAC();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Periodically call tinyUSB task */
    tud_task();

    // poll to see if octave up button pressed
    /*
    if (!HAL_GPIO_ReadPin(GPIOC, OCTAVE_UP_Pin) && !flag1){
      octave_num = octave_num + 1;
      if(octave_num > 4) octave_num = 4;
      flag1 = true;
    }
    if (HAL_GPIO_ReadPin(GPIOC, OCTAVE_UP_Pin)) flag1 = false;
    */

    // poll to see if octave down switch pressed
    /*
    if (!HAL_GPIO_ReadPin(GPIOC, OCTAVE_DOWN_Pin) && !flag2){
      octave_num = octave_num - 1;
      if(octave_num < 0) octave_num = 0;
      flag2 = true;
    }
    if (HAL_GPIO_ReadPin(GPIOC, OCTAVE_DOWN_Pin)) flag2 = false;
    */
    // Read values from all channels of ADC_1
    /*
    for (int i = 0; i < 8; i++) {
      ADC1_VAL[i] = Read_ADC(0,i); // CS = 0, CH = i
    }
    // Read values from all channels of ADC_2
    for (int i = 0; i < 8; i++) {
      ADC2_VAL[i] = Read_ADC(1,i); // CS = 1, CH = i
    }
    */
    /* CHECK OUTPUT SWITCH, SEND 0x00 from DAC if in MIDI mode*/

    // corresponds to DAC (analog) mode
    // verify the configuration of mode select pin
    /*
    if (!HAL_GPIO_ReadPin(GPIOC, MODE_SWITCH_Pin)){
      Set_DAC(HALL_TO_DAC(ADC1_VAL,ADC2_VAL,octave_num)); // data byte, corresponds to each channel of one 8 channel DAC (eventually need 2 DACs)
    }
    */
    /* EVENTUALLY should send DAC = 0 (SET GATE also eventually) AND midi signal */
    /*
    else{
      Set_DAC(0x0);
      READ_KEYPRESS(ADC1_VAL,ADC2_VAL);
      MIDI_TASK(octave_num);
    }
    */

    midi_task();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void MIDI_TASK(int octave_num){
  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);

  // delay necessary?
  //HAL_Delay(20);
  static int octave_history;

  uint8_t note_on[3];
  uint8_t note_off[3];

  //note_on[0] = 0x90 | channel;
  //note_on[1] = (42);
  //note_on[2] = 127;
  //tud_midi_stream_write(cable_num, note_on, 3);

  
  for(int i=0;i<12;i++){
    if ((((KEYPRESS >> i) & 0x1) == 1) && ((KEYPRESSED >> i) == 0)){
      KEYPRESSED = KEYPRESSED | (0x1 << i);
      note_on[0] = 0x90 | channel;
      note_on[1] = (12*octave_num + 24 + i);
      note_on[2] = 127;
      tud_midi_stream_write(cable_num, note_on, 3);
    }
    else if((((KEYPRESS >> i) & 0x1) == 1) && ((KEYPRESSED >> i) == 1)){
      if(octave_history == octave_num) continue;
      else {
        note_off[0] = 0x80 | channel;
        note_off[1] = (12*octave_history + 24 + i);
        note_off[2] = 0;
        tud_midi_stream_write(cable_num, note_off, 3);

        KEYPRESSED = KEYPRESSED | (0x1 << i);
        note_on[0] = 0x90 | channel;
        note_on[1] = (12*octave_num + 24 + i);
        note_on[2] = 127;
        tud_midi_stream_write(cable_num, note_on, 3);
      }
    }
    else if((((KEYPRESS >> i) & 0x1) == 0) && ((KEYPRESSED >> i) == 1)){
      KEYPRESSED = KEYPRESSED & (0x0 << i);
      note_off[0] = 0x80 | channel;
      note_off[1] = (12*octave_num + 24 + i);
      note_off[2] = 0;
      tud_midi_stream_write(cable_num, note_off, 3);
    }
    else {
      if(octave_history == octave_num) continue;
      else {
        note_off[0] = 0x80 | channel;
        note_off[1] = (12*octave_history + 24 + i);
        note_off[2] = 0;
        tud_midi_stream_write(cable_num, note_off, 3);
      }
    }
  }
  octave_history = octave_num;  
}

/* READ ALL CURRENTLY PRESSED KEYS */
void READ_KEYPRESS(uint32_t adc1_val[], uint32_t adc2_val[]) {
   KEYPRESS = 0x0;
   for (int i = 0; i < 12; i++) {
        if (i < 6) {
            if (adc1_val[i] > 600) KEYPRESS |= 0b1 << i;
        } else {
            if (adc2_val[i - 6] > 600) KEYPRESS |= 0b1 << i;
        }
    }
}

/* HALL EFFECT TO DAC OUTPUT CONVERSION */

uint8_t HALL_TO_DAC(uint32_t adc1_val[], uint32_t adc2_val[], int octave_num) {
    int channel_num = 12;
    
   for (int i = 0; i < 12; i++) {
        if (i < 6) {
            if (adc1_val[i] > 600) {
                channel_num = i;
                break;
            }
        } else {
            if (adc2_val[i - 6] > 600) {
                channel_num = i;
                break;
            }
        }
    }
    
    if (octave_num == 0) { // 0-50
        if (channel_num == 0) {
            return 3;
        } else if (channel_num == 1) {
            return 7;
        } else if (channel_num == 2) {
            return 11;
        } else if (channel_num == 3) {
            return 15;
        } else if (channel_num == 4) {
            return 19;
        } else if (channel_num == 5) {
            return 23;
        } else if (channel_num == 6) {
            return 27;
        } else if (channel_num == 7) {
            return 31;
        } else if (channel_num == 8) {
            return 35;
        } else if (channel_num == 9) {
            return 39;
        } else if (channel_num == 10) {
            return 43;
        } else if (channel_num == 11) {
            return 47;
        }
    } else if (octave_num == 1) { // 51-101
        if (channel_num == 0) {
            return 54;
        } else if (channel_num == 1) {
            return 58;
        } else if (channel_num == 2) {
            return 62;
        } else if (channel_num == 3) {
            return 66;
        } else if (channel_num == 4) {
            return 70;
        } else if (channel_num == 5) {
            return 74;
        } else if (channel_num == 6) {
            return 78;
        } else if (channel_num == 7) {
            return 82;
        } else if (channel_num == 8) {
            return 86;
        } else if (channel_num == 9) {
            return 90;
        } else if (channel_num == 10) {
            return 94;
        } else if (channel_num == 11) {
            return 98;
        }
    } else if (octave_num == 2) { // 102-152
        if (channel_num == 0) {
            return 105;
        } else if (channel_num == 1) {
            return 109;
        } else if (channel_num == 2) {
            return 113;
        } else if (channel_num == 3) {
            return 117;
        } else if (channel_num == 4) {
            return 121;
        } else if (channel_num == 5) {
            return 125;
        } else if (channel_num == 6) {
            return 129;
        } else if (channel_num == 7) {
            return 133;
        } else if (channel_num == 8) {
            return 137;
        } else if (channel_num == 9) {
            return 141;
        } else if (channel_num == 10) {
            return 145;
        } else if (channel_num == 11) {
            return 149;
        }
    } else if (octave_num == 3) { // 153-203
        if (channel_num == 0) {
            return 156;
        } else if (channel_num == 1) {
            return 160;
        } else if (channel_num == 2) {
            return 164;
        } else if (channel_num == 3) {
            return 168;
        } else if (channel_num == 4) {
            return 172;
        } else if (channel_num == 5) {
            return 176;
        } else if (channel_num == 6) {
            return 180;
        } else if (channel_num == 7) {
            return 184;
        } else if (channel_num == 8) {
            return 188;
        } else if (channel_num == 9) {
            return 192;
        } else if (channel_num == 10) {
            return 196;
        } else if (channel_num == 11) {
            return 200;
        }
    } else { // 204-255
        if (channel_num == 0) {
            return 207;
        } else if (channel_num == 1) {
            return 211;
        } else if (channel_num == 2) {
            return 215;
        } else if (channel_num == 3) {
            return 219;
        } else if (channel_num == 4) {
            return 223;
        } else if (channel_num == 5) {
            return 227;
        } else if (channel_num == 6) {
            return 231;
        } else if (channel_num == 7) {
            return 235;
        } else if (channel_num == 8) {
            return 239;
        } else if (channel_num == 9) {
            return 243;
        } else if (channel_num == 10) {
            return 247;
        } else if (channel_num == 11) {
            return 251;
        } 
    }
    return 0x0;
}

// need to also set variable for NOTE ON and NOTE OFF (elsewhere likely, global variable)
uint8_t DAC_TO_MIDI(uint8_t val){
  uint8_t midi = 0;
  if((val > 0) && (val < 48)) midi = val/4;
  else midi = (val/4) - 1;

  return midi;
}

void midi_task(void)
{
  static uint32_t start_ms = 0;

  uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
  uint8_t const channel   = 0; // 0 for channel 1

  // The MIDI interface always creates input and output port/jack descriptors
  // regardless of these being used or not. Therefore incoming traffic should be read
  // (possibly just discarded) to avoid the sender blocking in IO
  uint8_t packet[4];
  while ( tud_midi_available() ) tud_midi_packet_read(packet);

  // send note periodically
  /*
  if (board_millis() - start_ms < 286) return; // not enough time
  start_ms += 286;
  */

  HAL_Delay(286);

  // Previous positions in the note sequence.
  int previous = (int) (note_pos - 1);

  // If we currently are at position 0, set the
  // previous position to the last note in the sequence.
  if (previous < 0) previous = sizeof(note_sequence) - 1;

  // Send Note On for current position at full velocity (127) on channel 1.
  uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
  tud_midi_stream_write(cable_num, note_on, 3);

  // Send Note Off for previous note.
  uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
  tud_midi_stream_write(cable_num, note_off, 3);

  // Increment position
  note_pos++;

  // If we are at the end of the sequence, start over.
  if (note_pos >= sizeof(note_sequence)) note_pos = 0;
}

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
