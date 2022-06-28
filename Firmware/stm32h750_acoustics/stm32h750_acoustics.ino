/************************************************************************************************************/
// Hornet 7.0 Acoustics (Apr-May 2022)

// Board package: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
// Board: Generic STM32H7 Series
// Board part number: DevEBox H750VBTX
// U(S)ART support: Enabled (no generic 'Serial')
// USB support: CDC (generic 'Serial' supersede U(S)ART)
// USB speed: Low/Full Speed
// Optimize: Fastest (-O3) with LTO
// Debug symbols: None
// C Runtime Library: Newlib Nano + Float Printf
/************************************************************************************************************/
#include "elapsedMillis.h"

// Useful links:
//   https://scistatcalc.blogspot.com/2013/12/fft-calculator.html
//   https://github.com/PaulStoffregen/cores/blob/master/teensy3/core_pins.h

const int textBufferLength = 256;
char textBuffer[textBufferLength] = {0};

#define PIN_BUTTON_K1 (PE3)
#define PIN_BUTTON_K2 (PC5)
/************************************************************************************************************/
/* LEDs */
#define PIN_LED_BUILTIN (PA1)
#define PIN_LED_RED     (PE0)
#define PIN_LED_BLUE    (PB8)
#define PIN_LED_GREEN   (PB6)
#define PIN_LED_YELLOW  (PB4)
#define PIN_LED_WHITE   (PD7)


#define LED_BUILTIN_Off()     GPIOA->BSRR = GPIO_PIN_1
#define LED_BUILTIN_On()      GPIOA->BSRR = GPIO_PIN_1 << 16 // Note: Active Low
#define LED_BUILTIN_Toggle()  GPIOA->ODR ^= GPIO_PIN_1

#define LED_RED_On()          GPIOE->BSRR = GPIO_PIN_0
#define LED_RED_Off()         GPIOE->BSRR = GPIO_PIN_0 << 16
#define LED_RED_Toggle()      GPIOE->ODR ^= GPIO_PIN_0

#define LED_BLUE_On()         GPIOB->BSRR = GPIO_PIN_8
#define LED_BLUE_Off()        GPIOB->BSRR = GPIO_PIN_8 << 16
#define LED_BLUE_Toggle()     GPIOB->ODR ^= GPIO_PIN_8

#define LED_GREEN_On()        GPIOB->BSRR = GPIO_PIN_6
#define LED_GREEN_Off()       GPIOB->BSRR = GPIO_PIN_6 << 16
#define LED_GREEN_Toggle()    GPIOB->ODR ^= GPIO_PIN_6

#define LED_YELLOW_On()       GPIOB->BSRR = GPIO_PIN_4
#define LED_YELLOW_Off()      GPIOB->BSRR = GPIO_PIN_4 << 16
#define LED_YELLOW_Toggle()   GPIOB->ODR ^= GPIO_PIN_4

#define LED_WHITE_On()        GPIOD->BSRR = GPIO_PIN_7
#define LED_WHITE_Off()       GPIOD->BSRR = GPIO_PIN_7 << 16
#define LED_WHITE_Toggle()    GPIOD->ODR ^= GPIO_PIN_7

void LED_Setup() {
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
}

/************************************************************************************************************/
/* ADC */
#define ADC_BUFFER_SIZE (40000)
#define ADC_BUFFER_SAMPLE_TIME_MS (79) // Buffer holds 80ms of data (Sample 1 ms less to prevent overflow)

#define PIN_ADC_H0 (PA0) // ADC1_INP16
#define PIN_ADC_H1 (PA3) // ADC12_INP15
#define PIN_ADC_H2 (PA4) // ADC12_INP18
#define PIN_ADC_H3 (PA7) // ADC12_INP7

#define CHANNEL_ADC_H0 (ADC_CHANNEL_16) // ADC1
#define CHANNEL_ADC_H1 (ADC_CHANNEL_15) // ADC2
#define CHANNEL_ADC_H2 (ADC_CHANNEL_18) // ADC1
#define CHANNEL_ADC_H3 (ADC_CHANNEL_7 ) // ADC2

uint16_t Hydrophone_Index = 0;
uint16_t Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
uint16_t Hydrophone0_Values[ADC_BUFFER_SIZE];
uint16_t Hydrophone1_Values[ADC_BUFFER_SIZE];
uint16_t Hydrophone2_Values[ADC_BUFFER_SIZE];
uint16_t Hydrophone3_Values[ADC_BUFFER_SIZE];

void ADC_InitGPIO() {
    // Initialise GPIO as analog input using Arduino library
    pinMode(PIN_ADC_H0, INPUT);
    pinMode(PIN_ADC_H1, INPUT);
    pinMode(PIN_ADC_H2, INPUT);
    pinMode(PIN_ADC_H3, INPUT);
    analogRead(PIN_ADC_H0);
    analogRead(PIN_ADC_H1);
    analogRead(PIN_ADC_H2);
    analogRead(PIN_ADC_H3);
}

void ADC_InitBuffer() {
    // Initialise all input arrays to zero
    Hydrophone_Index = 0;
    memset(Hydrophone0_Values, 0, sizeof(Hydrophone0_Values));
    memset(Hydrophone1_Values, 0, sizeof(Hydrophone1_Values));
    memset(Hydrophone2_Values, 0, sizeof(Hydrophone2_Values));
    memset(Hydrophone3_Values, 0, sizeof(Hydrophone3_Values));
    Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
}

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_ChannelConfTypeDef adcConfig = {0};
ADC_MultiModeTypeDef multimode = {0};

void ADC_Setup() {
    // ADC clocks
    __HAL_RCC_ADC12_CLK_ENABLE();

    // Configure the ADC parameters
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;  
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Instance = ADC1;
    HAL_ADC_Init(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    ADC_Enable(&hadc1);

    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc2.Init.Resolution = ADC_RESOLUTION_16B;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;  
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc2.Init.OversamplingMode = DISABLE; 
    hadc2.Instance = ADC2;
    HAL_ADC_Init(&hadc2);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    ADC_Enable(&hadc2);

    // Configure multimode
    multimode.Mode = ADC_MODE_INDEPENDENT;
    HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
    HAL_ADCEx_MultiModeConfigChannel(&hadc2, &multimode);

    // Configure the channels
    adcConfig.Rank = ADC_REGULAR_RANK_1; // TAKE NOTE: RANK MUST BE SET TO 1
    adcConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    adcConfig.SingleDiff = ADC_SINGLE_ENDED;
    adcConfig.OffsetNumber = ADC_OFFSET_NONE;
    adcConfig.Offset = 0;
    adcConfig.OffsetSignedSaturation = DISABLE;
    /*
     * ADC_SAMPLETIME_1CYCLE_5
     * ADC_SAMPLETIME_2CYCLES_5
     * ADC_SAMPLETIME_8CYCLES_5
     * ADC_SAMPLETIME_16CYCLES_5
     * ADC_SAMPLETIME_32CYCLES_5
     * ADC_SAMPLETIME_64CYCLES_5
     * ADC_SAMPLETIME_387CYCLES_5
     * ADC_SAMPLETIME_810CYCLES_5 
     */
    
    adcConfig.Channel = CHANNEL_ADC_H0;
    HAL_ADC_ConfigChannel(&hadc1, &adcConfig);
    adcConfig.Channel = CHANNEL_ADC_H1;
    HAL_ADC_ConfigChannel(&hadc2, &adcConfig);
    adcConfig.Channel = CHANNEL_ADC_H2;
    HAL_ADC_ConfigChannel(&hadc1, &adcConfig);
    adcConfig.Channel = CHANNEL_ADC_H3;
    HAL_ADC_ConfigChannel(&hadc2, &adcConfig);
}

inline void sampleHydrophones() __attribute__((always_inline));
inline void sampleHydrophones() {
    __disable_irq();
    
    //---  Sample hydrophone 0 & 1
    // Select channels
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, CHANNEL_ADC_H0);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, CHANNEL_ADC_H1);

    // Start conversion
    LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC2);

    // Wait until finished
    while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0);
    while (LL_ADC_IsActiveFlag_EOS(ADC2) == 0);

    // Stop conversion
    LL_ADC_REG_StopConversion(ADC1);
    LL_ADC_REG_StopConversion(ADC2);

    // Save results
    Hydrophone0_Values[Hydrophone_Index] = LL_ADC_REG_ReadConversionData16(ADC1);
    Hydrophone1_Values[Hydrophone_Index] = LL_ADC_REG_ReadConversionData16(ADC2);

    // Clear flags
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_ClearFlag_EOS(ADC2);

    //--- Sample hydrophone 2 & 3
    // Select channels
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, CHANNEL_ADC_H2);
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, CHANNEL_ADC_H3);

    // Start conversion
    LL_ADC_REG_StartConversion(ADC1);
    LL_ADC_REG_StartConversion(ADC2);

    // Wait until finished
    while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0);
    while (LL_ADC_IsActiveFlag_EOS(ADC2) == 0);

    // Stop conversion
    LL_ADC_REG_StopConversion(ADC1);
    LL_ADC_REG_StopConversion(ADC2);

    // Save results
    Hydrophone2_Values[Hydrophone_Index] = LL_ADC_REG_ReadConversionData16(ADC1);
    Hydrophone3_Values[Hydrophone_Index] = LL_ADC_REG_ReadConversionData16(ADC2);

    // Clear flags
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_ClearFlag_EOS(ADC2);

    //--- Prepare for next index
    Hydrophone_Index++;
    if (Hydrophone_Index >= Hydrophone_MaxIndex || Hydrophone_Index >= ADC_BUFFER_SIZE) {
        Hydrophone_Index = 0;
    }
    
    __enable_irq();
}

void setHydrophoneMaxIndex(int m) {
    if (0 < m && m <= ADC_BUFFER_SIZE) {
        Hydrophone_MaxIndex = m;
    } else {
        Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
    }
}

/************************************************************************************************************/
/* Timer */
#define SAMPLING_FREQUENCY (500000)

// Note: disable Arduino HardwareTimer library
// https://www.openstm32.org/forumthread6674
TIM_HandleTypeDef my_timer;

extern "C" void TIM2_IRQHandler() {
  __HAL_TIM_CLEAR_IT(&my_timer, TIM_IT_UPDATE);
  LED_BUILTIN_On();
  sampleHydrophones();
  LED_BUILTIN_Off();
}

void Timer_Setup() {
  HAL_NVIC_DisableIRQ(TIM2_IRQn);

  __HAL_RCC_TIM2_CLK_ENABLE();
  my_timer.Instance = TIM2;
  my_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  my_timer.Init.Period = (2) - 1; // Period minimum of 2 
  my_timer.Init.Prescaler = (HAL_RCC_GetPCLK1Freq()/SAMPLING_FREQUENCY) - 1; // TIM2 is clocked using (APB1 x2)
  HAL_TIM_Base_Init(&my_timer);
  HAL_TIM_Base_Start_IT(&my_timer);
}

inline void Timer_Enable() {
  HAL_NVIC_EnableIRQ(TIM2_IRQn); 
}

inline void Timer_Disable() {
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

/************************************************************************************************************/
/* Goertzel Algorithm */
#include "MyGoertzel.h"

// sample_rate/N = bin_width
// bin_width = 500kHz / 128 = 3.90 kHz
// time_delay = 128 / 500kHz
#define GOERTZEL_SAMPLE_FREQ (SAMPLING_FREQUENCY)
#define GOERTZEL_SAMPLE_SIZE (128)

#define HYDROPHONE_FREQ_DRUM (45e3)
#define HYDROPHONE_FREQ_FLARE (37.5e3)

// Precalculated coefficients
float Goertzel_coeff;

void Goertzel_Init() {
  Goertzel_SetFrequency(HYDROPHONE_FREQ_DRUM); // 45 kHz
}

void Goertzel_SetFrequency(float target_freq) {
  Goertzel_coeff = Goertzel_coefficient(target_freq, GOERTZEL_SAMPLE_FREQ);
}

volatile float Goertzel_magnitude[4] = {0};

inline void Goertzel_Compute(const uint16_t offset_index) __attribute__((always_inline));
inline void Goertzel_Compute(const uint16_t offset_index) {
  Goertzel_magnitude[0] = Goertzel_magnitudeSquare(&Hydrophone0_Values[offset_index], GOERTZEL_SAMPLE_SIZE, Goertzel_coeff);
  Goertzel_magnitude[1] = Goertzel_magnitudeSquare(&Hydrophone1_Values[offset_index], GOERTZEL_SAMPLE_SIZE, Goertzel_coeff);
  Goertzel_magnitude[2] = Goertzel_magnitudeSquare(&Hydrophone2_Values[offset_index], GOERTZEL_SAMPLE_SIZE, Goertzel_coeff);
  Goertzel_magnitude[3] = Goertzel_magnitudeSquare(&Hydrophone3_Values[offset_index], GOERTZEL_SAMPLE_SIZE, Goertzel_coeff);
}

void Goertzel_MovingAverageUnoptimised(uint16_t offset_index, uint16_t period) {
  float sum[4] = {0,0,0,0};
  for (uint16_t i = 0; i < period; i++) {
    // If we don't have enough data from the past, then repeat the zeroth data
    if (offset_index > i) {
      Goertzel_Compute(offset_index-i);
    } else {
      Goertzel_Compute(0);
    }
    // Keep track of the sums
    for (uint8_t x = 0; x < 4; x++) {
      sum[x] += Goertzel_magnitude[x];
    }
  }

  // After done, then write the average into the magnitude
  for (uint8_t x = 0; x < 4; x++) {
    Goertzel_magnitude[x] = sum[x] / period;
  }
}


uint16_t Goertzel_memo_offset_index = 0;
float Goertzel_memo_sum[4] = {0,0,0,0};

void Goertzel_MovingAverage(uint16_t offset_index, uint16_t period) { // MEMOISED
  // Compute normally if it is very first one, or if the index is not consecutive.
  if (offset_index == 0 || (offset_index-Goertzel_memo_offset_index) != 1) {
    float sum[4] = {0,0,0,0};
    for (uint16_t i = 0; i < period; i++) {
      // If we don't have enough data from the past, then repeat the zeroth data
      if (offset_index > i) {
        Goertzel_Compute(offset_index-i);
      } else {
        Goertzel_Compute(0);
      }
      // Keep track of the sums
      for (uint8_t x = 0; x < 4; x++) {
        sum[x] += Goertzel_magnitude[x];
      }
    }
    // After done, then write the average into the magnitude + memoise
    for (uint8_t x = 0; x < 4; x++) {
      Goertzel_magnitude[x] = sum[x] / period;
      Goertzel_memo_sum[x] = sum[x];
    }
  } else { // consecutive index: modify from memoised
    // Delete first entry
    if (offset_index < period) {
      Goertzel_Compute(0);
    } else {
      Goertzel_Compute(offset_index-period);
    }    
    for (uint8_t x = 0; x < 4; x++) {
      Goertzel_memo_sum[x] -= Goertzel_magnitude[x];
    }
  
    // Add new entry
    Goertzel_Compute(offset_index);
    for (uint8_t x = 0; x < 4; x++) {
      Goertzel_memo_sum[x] += Goertzel_magnitude[x];
    // After done, then write the average into the magnitude
      Goertzel_magnitude[x] = Goertzel_memo_sum[x] / period;
    }
  }
  Goertzel_memo_offset_index = offset_index;
}

float calculateCentimeters(float ticks) {
  const int SPEED_OF_SOUND = 1500; // meters per second

  // ticks to seconds
  float seconds = ticks * 1.0 / GOERTZEL_SAMPLE_FREQ;

  // milliseconds to meters
  float meters = seconds * SPEED_OF_SOUND;

  // convert to centimeters
  return meters * 100;
}


/************************************************************************************************************/
/* CAN communication */
HardwareSerial HWSERIAL(PA10, PA9);

void Comms_Init(void) {
  HWSERIAL.begin(115200);
}

// Format string: %d,%d,%d,%d,%d;
// Identifier: freq,h1,h2,h3,h4;
// Example: 37500,2877,3075,3432,2402;
void Comms_SendData(int freq, int * tick) {
  snprintf(textBuffer, textBufferLength,
           "%d,%d,%d,%d,%d;",
           freq, tick[0], tick[1], tick[2], tick[3]);
  HWSERIAL.println(textBuffer);
  HWSERIAL.flush();
//  HWSERIAL.print(freq, DEC);    HWSERIAL.print(',');
//  HWSERIAL.print(tick[0], DEC); HWSERIAL.print(',');
//  HWSERIAL.print(tick[1], DEC); HWSERIAL.print(',');
//  HWSERIAL.print(tick[2], DEC); HWSERIAL.print(',');
//  HWSERIAL.print(tick[3], DEC); HWSERIAL.println(';');
//  HWSERIAL.flush();
}

/************************************************************************************************************/
/* Buttons */

bool TestMode_A = false; // Send raw ADC output to the Serial USB
bool TestMode_B = false; // Live analogRead debugging to Serial USB

void Buttons_Init(void) {
    pinMode(PIN_BUTTON_K1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_K2, INPUT_PULLUP);
    delay(100); // Let button pullup stabilise
}

/************************************************************************************************************/
/* Main */

void setup() {
    //-- Initialise LEDs
    LED_Setup();

    //-- Button press mode
    Buttons_Init();
    
    if (digitalRead(PIN_BUTTON_K1) == LOW) {
      TestMode_A = true;
    }
    if (digitalRead(PIN_BUTTON_K2) == LOW) {
      TestMode_B = true;
    }

    if (TestMode_B) {
      while (TestMode_B) {
        if (digitalRead(PIN_BUTTON_K1) == LOW){
        Serial.print("TestMode_A,");
      }if (digitalRead(PIN_BUTTON_K2) == LOW){
        Serial.print("TestMode_B,");
      }
        Serial.print(analogRead(PIN_ADC_H0), DEC); Serial.print(",");
        Serial.print(analogRead(PIN_ADC_H1), DEC); Serial.print(",");
        Serial.print(analogRead(PIN_ADC_H2), DEC); Serial.print(",");
        Serial.print(analogRead(PIN_ADC_H3), DEC); Serial.println("");
      }
    }
    // Test LEDs
    for (int i = 0; i < 3; i++) {
      LED_RED_On();
      LED_BLUE_On();
      LED_GREEN_On();
      LED_YELLOW_On();
      LED_WHITE_On();
      delay(250);
      LED_RED_Off();
      LED_BLUE_Off();
      LED_GREEN_Off();
      LED_YELLOW_Off();
      LED_WHITE_Off();
      delay(250);
    }
    
    //-- Initialise ADC input pins
    ADC_InitGPIO();
    
    //-- Setup dual ADCs
    ADC_InitBuffer();
    ADC_Setup();

    //-- Setup hardware timer
    Timer_Setup();
    Timer_Disable();

    //-- Setup algorithms
    Goertzel_Init();
    Goertzel_SetFrequency(HYDROPHONE_FREQ_DRUM);

    //-- Start USB serial
    Serial.begin(1000000);
    //while(!Serial); // wait for PC to connect

    //-- Start CAN communication
    Comms_Init();

    //-- Start main process
#define TIME_NOMINAL_PING (1000) // Pings come every 1 second
#define TIME_PING_DURATION (10) // Each ping lasts for 10ms duration
#define TIME_CALIBRATION (1100) // 1.1 seconds is confirmed to contain a ping
#define TIME_WAIT_FOLLOWING_PEAK (960) // Start sampling into buffer just before the next peak

    // Timing parameters
    elapsedMillis timeElapsed;

    //float targetFrequency = HYDROPHONE_FREQ_FLARE; // 37.5kHz
    float targetFrequency = HYDROPHONE_FREQ_DRUM; // 45kHz

    while(true) {
      //-- Reset LEDs
      LED_RED_Off();
      LED_BLUE_Off();
      LED_GREEN_Off();
      LED_YELLOW_Off();
      LED_WHITE_Off();

      //-- Setup for next processing
      Goertzel_SetFrequency(targetFrequency);
      if (targetFrequency == HYDROPHONE_FREQ_DRUM) {
        LED_YELLOW_On();
      } else {
        LED_YELLOW_Off();
      }

      //---------------------------------------------------------------------------
      //-- Prepare for short sample buffer
      Timer_Disable();
      ADC_InitBuffer();
      setHydrophoneMaxIndex(GOERTZEL_SAMPLE_SIZE);

      //-- Calibrate limits for maximum 1.1 seconds, store peak time
      unsigned long peakTime = 0;
      float upperLimit = 0;

      LED_BLUE_On();
      LED_RED_On();

      timeElapsed = 0;
      Timer_Enable();

      while (timeElapsed < TIME_CALIBRATION) {
        for (uint16_t i = 255*2; i; i--) {
          Goertzel_Compute(0);
          // Check upper limit for all 4 hydrophones
          for (int x = 0; x < 4; x++) {
            if (Goertzel_magnitude[x] > upperLimit) {
              upperLimit = Goertzel_magnitude[x];
              peakTime = timeElapsed; // Store time of peak
            }
          }
        }
        LED_RED_Toggle();
      }
      LED_RED_Off();

      //---------------------------------------------------------------------------
      //-- Prepare for large sample buffer
      Timer_Disable();
      ADC_InitBuffer();
      setHydrophoneMaxIndex(ADC_BUFFER_SIZE); // Max buffer

      //-- Wait for 960ms after the previous peak
      LED_BLUE_On();
      while (timeElapsed > (peakTime+TIME_WAIT_FOLLOWING_PEAK)) {
        // if we missed the peak then adjust it later by 1 second
        peakTime += TIME_NOMINAL_PING;
      }
      while (timeElapsed < (peakTime+TIME_WAIT_FOLLOWING_PEAK));
      LED_BLUE_Off();

      //-- Sample for 99ms (slightly less than max buffer of 100ms)
      LED_GREEN_On();
      LED_RED_On();

      Timer_Enable();
      while (timeElapsed < (peakTime+TIME_WAIT_FOLLOWING_PEAK+ADC_BUFFER_SAMPLE_TIME_MS)) {
        LED_RED_Toggle();
      }
      Timer_Disable();

      LED_RED_Off();

      //---------------------------------------------------------------------------
      /*
      //-- Calculate the threshold value / magnitude threshold
      const int movingAveragePeriod = 6;
      float threshold = 2750 * 1e6;//(upperLimit * 0.30);

      LED_GREEN_On();

      //-- Algorithm and calculations
      int ticks[4] = {0};
      for (int i = movingAveragePeriod; i < (ADC_BUFFER_SIZE-GOERTZEL_SAMPLE_SIZE); i++) {
        //Goertzel_Compute(i);
        Goertzel_MovingAverage(i, movingAveragePeriod);

        // Check for threshold
        for (int x = 0; x < 4; x++) {
          if (Goertzel_magnitude[x] > threshold) {
            if (ticks[x] == 0) ticks[x] = i;
          }
        }

        // If found all ticks then exit
        if ((ticks[0] != 0) && (ticks[1] != 0) &&
            (ticks[2] != 0) && (ticks[3] != 0)) {
          break;
        }
      }
      LED_GREEN_Off();
      */

      //-- Algorithm and calculations (differential threshold)
      LED_GREEN_On();

      // Find normalisation
      float upperLimitArray[4] = {0};
      float normalisation[4] = {1,1,1,1};
      for (int i = 0; i < (ADC_BUFFER_SIZE-GOERTZEL_SAMPLE_SIZE); i++) {
        Goertzel_Compute(i);
        // Check upper limit for all 4 hydrophones
        for (int x = 0; x < 4; x++) {
          if (Goertzel_magnitude[x] > upperLimitArray[x]) {
            upperLimitArray[x] = Goertzel_magnitude[x];
          }
        }
      }
      // Calculate normalisation
//      for (int x = 0; x < 4; x++) {
//        if (upperLimitArray[x] > (upperLimit * 0.5)) {
//          normalisation[x] = upperLimit / upperLimitArray[x]; // there was a detection, normalise to upperLimit
//        } else {
//          normalisation[x] = 1; // don't normalise if it doesn't detect anything
//        }
//      }
      
      // Set algorithm parameters
      const int movingAveragePeriod = 50;
      const float differential_threshold = 100 * 1e6; // HERE!!
      const float amplitude_threshold = 13500 * 1e6;
      float previous_magnitude[4] = {0,0,0,0};
      int ticks[4] = {0,0,0,0};

      float exceededThreshold = upperLimit * 0.5;
      bool exceededQuit = false;

      // Based on absolute amplitude threshold, find approximate times where signal is present
      bool isValidSignal = false;
      const int min_start_tick = movingAveragePeriod; // first valid sample in goertzel
      const int max_end_tick = (Hydrophone_Index-GOERTZEL_SAMPLE_SIZE*2); // last valid sample in goertzel
 
      int approx_start_tick = 0;
      int approx_end_tick = max_end_tick;
      for (int i = movingAveragePeriod; i < (Hydrophone_Index-GOERTZEL_SAMPLE_SIZE*2); i++) {
        Goertzel_MovingAverage(i, movingAveragePeriod);
        for (int x = 0; x < 4; x++) {
          // If there is a rising signal from below to above absolute amplitude,
          // then there is a valid signal
          if (Goertzel_magnitude[x] >= amplitude_threshold) {
            isValidSignal = true;
            approx_start_tick = i;
          }
        }
        if (approx_start_tick > 0) break;
      }

      // Start analysis from 1 ping duration before to slightly after
      // If it results in a negative number (before the movingAveragePeriod),
      // then it is an invalid start position (false positive)
      if (approx_start_tick > 0) {
        const int one_ping = (int) (TIME_PING_DURATION/1000.0*SAMPLING_FREQUENCY);
        approx_start_tick -= one_ping;
        // Constraint to boundaries
        approx_start_tick = max(min_start_tick, approx_start_tick);
        approx_end_tick = max_end_tick; // TODO: min(max_end_tick, approx_start_tick + one_ping*2);
      }

      // Loop moving average
      for (int i = approx_start_tick; i < approx_end_tick; i++) {
        // Save previous magnitude
        for (int x = 0; x < 4; x++) {
          previous_magnitude[x] = Goertzel_magnitude[x];
        }

        //Goertzel_Compute(i);
        Goertzel_MovingAverage(i, movingAveragePeriod);

        // Check for threshold
        if (i > (movingAveragePeriod+2)) {
          // compare differential
          for (int y = 0; y < 4; y++) {
            // If gradient exceeds, then we found the start of the ping
            if ( (Goertzel_magnitude[y] - previous_magnitude[y])*normalisation[y] > differential_threshold) {
              if (ticks[y] == 0) ticks[y] = i;
            }
          }
        }
        // compare absolute
//        for (int y = 0; y < 4; y++) {
//          if (Goertzel_magnitude[y] > (8000 * 1e6)) {
//            if (ticks[y] == 0) ticks[y] = i;
//          }
//        }
//        
        // If found all ticks then exit
        if ((ticks[0] != 0) && (ticks[1] != 0) &&
            (ticks[2] != 0) && (ticks[3] != 0)) {
          break;
        }

        // If threshold exceeded then give up
        for (int x = 0; x < 4; x++) {
          if (Goertzel_magnitude[x] > exceededThreshold) {
            exceededQuit = true;
            break;
          }
        }
        if (exceededQuit) break;
      }

      // Set to max value to indicate error
      if (!isValidSignal) {
        ticks[0] = ADC_BUFFER_SIZE-1;
        ticks[1] = ADC_BUFFER_SIZE-1;
        ticks[2] = ADC_BUFFER_SIZE-1;
        ticks[3] = ADC_BUFFER_SIZE-1;
      }
      LED_GREEN_Off();


      //---------------------------------------------------------------------------
      //-- CAN communication
      Comms_SendData((int)targetFrequency, &ticks[0]);

      //-- USB serial debug
      LED_GREEN_Off();
      LED_WHITE_On();

      // Debug only if USB Serial is connected.
      if (Serial) {
        Serial.println("#START#");
        Serial.print("timeElapsed: "); Serial.print(timeElapsed, DEC);  Serial.println("ms");
        Serial.print("F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz.");
  
        Serial.print("Target frequency /1e3: "); Serial.println(targetFrequency/1e3, DEC);
        Serial.print("Upper limit /1e9: "); Serial.println(upperLimit/1e9, DEC);
        //Serial.print("Lowest limit: "); Serial.println(lowerLimit, DEC);
        Serial.print("Differential Threshold /1e6: "); Serial.println(differential_threshold/1e6, DEC);
  
        Serial.println("Hydrophone_Index : "); Serial.println(Hydrophone_Index, DEC);
  
        Serial.print("approx_start_tick: "); Serial.println(approx_start_tick, DEC);
        Serial.print("approx_end_tick: "); Serial.println(approx_end_tick, DEC);
        Serial.print("exceededQuit: "); Serial.println(exceededQuit, DEC);
          
        // ticks data
        Serial.println("ticks Raw : ");
        for (int x = 0; x < 4; x++) {
          Serial.println(ticks[x], DEC);
        }
        Serial.println("Ticks Diff : ");
        Serial.print(ticks[1] - ticks[0], DEC); Serial.println("");
        Serial.print(ticks[2] - ticks[0], DEC); Serial.println("");
        Serial.print(ticks[3] - ticks[0], DEC); Serial.println("");
  
        Serial.println("Centimeters : ");
        Serial.println(calculateCentimeters(ticks[1] - ticks[0]), DEC);
        Serial.println(calculateCentimeters(ticks[2] - ticks[0]), DEC);
        Serial.println(calculateCentimeters(ticks[3] - ticks[0]), DEC);

        if (TestMode_A) {
          Serial.println("Test mode A");
        }
            
  
        // goertzel outputs
        Serial.println("------");
        Serial.println("Goertzel:");

        // Triger send now to clear buffer
        //Serial.flush();

        int payload_size = 0;
        for (uint16_t i = 0; i < (ADC_BUFFER_SIZE-GOERTZEL_SAMPLE_SIZE); i++) {        
          Goertzel_MovingAverage(i, movingAveragePeriod);

          if (TestMode_A) {
            // Test mode A
            payload_size = snprintf(textBuffer, textBufferLength,"%.02f,%.02f,%.02f,%.02f,%d\n",
                   Hydrophone0_Values[i]*1.0,Hydrophone1_Values[i]*1.0,
                   Hydrophone2_Values[i]*1.0,Hydrophone3_Values[i]*1.0,
                   i);
          } else {
            // Normal mode
            payload_size = snprintf(textBuffer, textBufferLength,"%.02f,%.02f,%.02f,%.02f,%d\n",
                   Goertzel_magnitude[0]*1e-6*normalisation[0], Goertzel_magnitude[1]*1e-6*normalisation[1],
                   Goertzel_magnitude[2]*1e-6*normalisation[2], Goertzel_magnitude[3]*1e-6*normalisation[3],
                   i);
          }
  
          // Flow control
          while (Serial.availableForWrite() <= payload_size); // waits until the serial output buffer has enough space
  
          // Write payload to serial (fast)
          Serial.write(textBuffer, payload_size);
          //Serial.flush(); // trigger send now
        
          // Flash LED
          // if (i % 64) {
          if (i % 256 == 0) {
            LED_GREEN_Toggle();
          }
        }

        Serial.println("------");
        Serial.println("#END#");
  
        //Serial.send_now(); // trigger send now
      }

      // Change frequency if needed
      delay(10);
      // TODO: logic to change frequency
      //targetFrequency = (HYDROPHONE_FREQ_FLARE); // 37.5kHz
      targetFrequency = (HYDROPHONE_FREQ_DRUM); // 45kHz

      LED_WHITE_Off();
      LED_GREEN_Off();
      LED_BLUE_Off();
  }
}

// Should not reach here.
void loop() {
  Serial.printf("Hello world!, %d\n", F_CPU);
  delay(1000);
  long before = micros();
  sampleHydrophones();
  long after = micros();
  Serial.printf("Time  %d\n", after-before);
  Serial.printf("Reading %d, %d, %d, %d\n", Hydrophone0_Values[Hydrophone_Index-1],Hydrophone1_Values[Hydrophone_Index-1],Hydrophone2_Values[Hydrophone_Index-1],Hydrophone3_Values[Hydrophone_Index-1]);
}
