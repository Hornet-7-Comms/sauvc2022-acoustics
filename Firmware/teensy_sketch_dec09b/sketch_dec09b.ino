/************************************************************************************************************/
// Hornet 7.0 Acoustics (Jan-Mar 2022)

// Board: Teensy 3.6
// USB Type: Serial
// CPU Speed: 192 MHz (overclock)
// Optimize: Fastest with LTO

// Edit <ARDUINO>/arduino-1.8.19/hardware/teensy/avr/cores/teensy3/usb_desc.h
//     #define NUM_USB_BUFFERS 1024 // 12
/************************************************************************************************************/

#include <elapsedMillis.h>
#include <float.h>

// Useful links:
//   https://scistatcalc.blogspot.com/2013/12/fft-calculator.html
//   https://github.com/PaulStoffregen/cores/blob/master/teensy3/core_pins.h

const int textBufferLength = 256;
char textBuffer[textBufferLength] = {0};

/************************************************************************************************************/
#include <ADC.h>

#define ADC_BUFFER_SIZE (20000)

ADC *adc = new ADC(); // adc object

uint16_t Hydrophone_Index = 0;
uint16_t Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
uint16_t Hydrophone0_Values[ADC_BUFFER_SIZE]; // A14
uint16_t Hydrophone1_Values[ADC_BUFFER_SIZE]; // A15
uint16_t Hydrophone2_Values[ADC_BUFFER_SIZE]; // A16
uint16_t Hydrophone3_Values[ADC_BUFFER_SIZE]; // A17

void ADC_InitBuffer() {
    // Initialise all input arrays to zero
    Hydrophone_Index = 0;
    memset(Hydrophone0_Values, 0, sizeof(Hydrophone0_Values));
    memset(Hydrophone1_Values, 0, sizeof(Hydrophone0_Values));
    memset(Hydrophone2_Values, 0, sizeof(Hydrophone0_Values));
    memset(Hydrophone3_Values, 0, sizeof(Hydrophone0_Values));
    Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
}

void ADC_Setup() {
    adc->adc0->setAveraging(0); // set number of averages
    adc->adc0->setResolution(16); // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
    adc->adc0->singleMode(); // set to single mode (no continuous mode)

    adc->adc1->setAveraging(0); // set number of averages
    adc->adc1->setResolution(16); // set bits of resolution
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
    adc->adc1->singleMode(); // set to single mode (no continuous mode)
}

int ADC_CheckPin(int pin) {
  if (adc->adc0->checkPin(pin)) {
    return 0;
  }
  if (adc->adc1->checkPin(pin)) {
    return 1;
  }
  return -1;
}

/*
inline void FasterADC_startReadFast(uint8_t, uint8_t) __attribute__((always_inline, unused));
inline void FasterADC_startReadFast(uint8_t pin1, uint8_t pin2) {
    uint32_t adc0_X_SC1A;
    uint32_t adc1_X_SC1A;

    // Prepare ADC0 for single-ended mode without interrupt
    if (channel2sc1aADC0[pin1] & ADC_SC1A_PIN_MUX) { // mux a
        atomic::clearBitFlag(ADC0_CFG2, ADC_CFG2_MUXSEL);
    } else { // mux b
        atomic::setBitFlag(ADC0_CFG2, ADC_CFG2_MUXSEL);
    }
    adc0_X_SC1A = (channel2sc1aADC0[pin1] & ADC_SC1A_CHANNELS);

    // Prepare ADC1 for single-ended mode without interrupt
    if (channel2sc1aADC1[pin2] & ADC_SC1A_PIN_MUX) { // mux a
        atomic::clearBitFlag(ADC1_CFG2, ADC_CFG2_MUXSEL);
    } else { // mux b
        atomic::setBitFlag(ADC1_CFG2, ADC_CFG2_MUXSEL);
    }
    adc1_X_SC1A = (channel2sc1aADC1[pin2] & ADC_SC1A_CHANNELS);

    // Start almost instantaneously
    ADC0_SC1A = adc0_X_SC1A;
    ADC1_SC1A = adc1_X_SC1A;
}*/

uint16_t adc_results[4];

inline void FasterADC_analogSynchronizedRead(uint8_t, uint8_t, uint8_t, uint8_t) __attribute__((always_inline));
inline void FasterADC_analogSynchronizedRead(uint8_t adc0_pinA, uint8_t adc1_pinA, uint8_t adc0_pinB, uint8_t adc1_pinB) {
    // start both measurements
    adc->adc0->startReadFast(adc0_pinA);
    adc->adc1->startReadFast(adc1_pinA);

    // wait for both ADCs to finish
    while ((adc->adc0->isConverting()) || (adc->adc1->isConverting())) {
        yield();
    }
    __disable_irq();
    if (adc->adc0->isComplete()) {
        adc_results[0] = ADC0_RA; // adc->adc0->readSingle();
    }
    if (adc->adc1->isComplete()) {
        adc_results[1] = ADC1_RA; //adc->adc1->readSingle();
    }
    __enable_irq();

    // start both measurements
    adc->adc0->startReadFast(adc0_pinB);
    adc->adc1->startReadFast(adc1_pinB);

    // wait for both ADCs to finish
    while ((adc->adc0->isConverting()) || (adc->adc1->isConverting())) {
        yield();
    }
    __disable_irq();
    if (adc->adc0->isComplete()) {
        adc_results[2] = ADC0_RA; // adc->adc0->readSingle();
    }
    if (adc->adc1->isComplete()) {
        adc_results[3] = ADC1_RA; //adc->adc1->readSingle();
    }
    __enable_irq();
}

inline void sampleHydrophones() __attribute__((always_inline));
inline void sampleHydrophones() {
    // Sample 4 hydrophones quickly
    // ADC0 = A14, A15
    // ADC0 = A21
    // ADC1 = A16, A17
    // ADC1 = A20
    FasterADC_analogSynchronizedRead(A14, A16, A15, A20);
    //adc_result1 = Faster_analogSynchronizedRead(A14, A16); //adc_result1 = adc->analogSynchronizedRead(A14, A16);
    //adc_result2 = Faster_analogSynchronizedRead(A15, A17); //adc_result2 = adc->analogSynchronizedRead(A15, A17);

    // Pass it into the array
    Hydrophone0_Values[Hydrophone_Index] = (uint16_t) adc_results[0]; // A14 (ADC0)
    Hydrophone1_Values[Hydrophone_Index] = (uint16_t) adc_results[2]; // A15 (ADC0)
    Hydrophone2_Values[Hydrophone_Index] = (uint16_t) adc_results[1]; // A16 (ADC1)
    Hydrophone3_Values[Hydrophone_Index] = (uint16_t) adc_results[3]; // A17 (ADC1)

    Hydrophone_Index++;
    if (Hydrophone_Index >= Hydrophone_MaxIndex || Hydrophone_Index >= ADC_BUFFER_SIZE) {
        Hydrophone_Index = 0;
    }
}

void setHydrophoneMaxIndex(int m) {
  if (0 < m && m <= ADC_BUFFER_SIZE) {
    Hydrophone_MaxIndex = m;
  } else {
    Hydrophone_MaxIndex = ADC_BUFFER_SIZE;
  }
}
/************************************************************************************************************/

/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(0)      Prescaler = 1 [2^x]
  PDB_SC_MULT(1)           Prescaler multiplication factor = 1 [10*x]
  PDB_SC_DMAEN             Enable DMA
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(0) | PDB_SC_MULT(0))
#define PDB_FREQUENCY (200000)

void PDB_Setup() {
  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;

  // Modulus Register
  const uint32_t mod = F_BUS / 1 / 1 / PDB_FREQUENCY;

  PDB0_MOD = (uint16_t)(mod - 1);

  // Interrupt delay
  PDB0_IDLY = 0;

  // PDB status and control
  PDB0_SC = PDB_CONFIG;

  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;

  // Load OK
  PDB0_SC |= PDB_SC_LDOK;

  // Enable pre-trigger
  PDB0_CH0C1 = 0x0101;

  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

inline void PDB_Enable() {
  //NVIC_ENABLE_IRQ(IRQ_PDB);
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK; // reset configuration (also clears interrupt flag)
}

inline void PDB_Disable() {
  //NVIC_DISABLE_IRQ(IRQ_PDB);
  PDB0_SC &= ~PDB_SC_PDBIE; // Clear enable bit
}

// PDB interrupt handler
void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF; // Clear interrupt flag
  //PDB0_SC = PDB_CONFIG | PDB_SC_LDOK; // reset configuration (also clears interrupt flag)

  //CORE_PIN28_PORTCLEAR |= CORE_PIN28_BITMASK; // Clear pin 28 white led
  sampleHydrophones();
  //CORE_PIN28_PORTTOGGLE |= CORE_PIN28_BITMASK; // Toggle pin 28 white led
}


/************************************************************************************************************/
#include "MyGoertzel.h"

// sample_rate/N = bin_width
// bin_width = 200kHz / 64 = 3.12 kHz
// time_delay = 64 / 200kHz
#define GOERTZEL_SAMPLE_FREQ (PDB_FREQUENCY)
#define GOERTZEL_SAMPLE_SIZE (64)

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
#include <SoftwareSerial.h>

// Use software serial as a workaround as the hardware serial enables additional interrupts
// which uses up CPU time during the ADC conversion and causes the whole program to stall.
#define Serial4_RX (31)
#define Serial4_TX (32)
SoftwareSerial SoftwareSerial4 = SoftwareSerial(Serial4_RX, Serial4_TX);

#define HWSERIAL SoftwareSerial4

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

#define PIN_LED_BUILTIN (13)

#define PIN_LED_RED    (24)
#define PIN_LED_BLUE   (25)
#define PIN_LED_GREEN  (26)
#define PIN_LED_YELLOW (27)
#define PIN_LED_WHITE  (28)

#define LED_RED_On()        CORE_PIN24_PORTSET    |= CORE_PIN24_BITMASK
#define LED_RED_Off()       CORE_PIN24_PORTCLEAR  |= CORE_PIN24_BITMASK
#define LED_RED_Toggle()    CORE_PIN24_PORTTOGGLE |= CORE_PIN24_BITMASK

#define LED_BLUE_On()       CORE_PIN25_PORTSET    |= CORE_PIN25_BITMASK
#define LED_BLUE_Off()      CORE_PIN25_PORTCLEAR  |= CORE_PIN25_BITMASK
#define LED_BLUE_Toggle()   CORE_PIN25_PORTTOGGLE |= CORE_PIN25_BITMASK

#define LED_GREEN_On()      CORE_PIN26_PORTSET    |= CORE_PIN26_BITMASK
#define LED_GREEN_Off()     CORE_PIN26_PORTCLEAR  |= CORE_PIN26_BITMASK
#define LED_GREEN_Toggle()  CORE_PIN26_PORTTOGGLE |= CORE_PIN26_BITMASK

#define LED_YELLOW_On()     CORE_PIN27_PORTSET    |= CORE_PIN27_BITMASK
#define LED_YELLOW_Off()    CORE_PIN27_PORTCLEAR  |= CORE_PIN27_BITMASK
#define LED_YELLOW_Toggle() CORE_PIN27_PORTTOGGLE |= CORE_PIN27_BITMASK

#define LED_WHITE_On()      CORE_PIN28_PORTSET    |= CORE_PIN28_BITMASK
#define LED_WHITE_Off()     CORE_PIN28_PORTCLEAR  |= CORE_PIN28_BITMASK
#define LED_WHITE_Toggle()  CORE_PIN28_PORTTOGGLE |= CORE_PIN28_BITMASK

void setup() {
    //-- Initialise ADC input pins
    pinMode(A20, INPUT); // ADC1
    pinMode(A21, INPUT); // ADC0
    
    pinMode(A14, INPUT); // ADC0
    pinMode(A15, INPUT); // ADC0
    pinMode(A16, INPUT); // ADC1
    pinMode(A17, INPUT); // ADC1

    //-- Initialise LEDs
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_WHITE, OUTPUT);

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

    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, LOW);

    //-- Setup dual ADCs
    ADC_InitBuffer();
    ADC_Setup();

    if (false) {
      Serial.print("A21: "); Serial.println(ADC_CheckPin(A21), DEC);
      Serial.print("A20: "); Serial.println(ADC_CheckPin(A20), DEC);
      Serial.print("A19: "); Serial.println(ADC_CheckPin(A19), DEC);
      Serial.print("A18: "); Serial.println(ADC_CheckPin(A18), DEC);
      Serial.print("A17: "); Serial.println(ADC_CheckPin(A17), DEC);
      Serial.print("A16: "); Serial.println(ADC_CheckPin(A16), DEC);
      Serial.print("A15: "); Serial.println(ADC_CheckPin(A15), DEC);
      Serial.print("A14: "); Serial.println(ADC_CheckPin(A14), DEC);
      Serial.print("A13: "); Serial.println(ADC_CheckPin(A13), DEC);
      Serial.print("A12: "); Serial.println(ADC_CheckPin(A12), DEC);
    }
    
    //-- Setup hardware timer
    PDB_Setup();
    PDB_Disable();

    //-- Setup algorithms
    Goertzel_Init();
    Goertzel_SetFrequency(HYDROPHONE_FREQ_DRUM);

    //-- Start serial communication
    Serial.begin(1000000);
    //while(!Serial){}; // wait for PC to connect

    //-- Start CAN communication
    Comms_Init();

    // Timing parameters
    elapsedMillis timeElapsed;

    //-- Start main process
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

      //---------------------------------------------------------------------------
      //-- Prepare for short sample buffer
      PDB_Disable();
      ADC_InitBuffer();
      setHydrophoneMaxIndex(GOERTZEL_SAMPLE_SIZE);

      //-- Calibrate limits for maximum 1.1 seconds, store peak time
      unsigned long peakTime = 0;
      float upperLimit = 0;

      timeElapsed = 0;
      PDB_Enable();

      LED_BLUE_On();
      LED_RED_On();
      while (timeElapsed < 1100) {
        for (uint8_t i = 255; i; i--) {
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
      PDB_Disable();
      ADC_InitBuffer();
      setHydrophoneMaxIndex(ADC_BUFFER_SIZE); // Max buffer

      //-- Wait for 960ms after the previous peak
      LED_BLUE_On();
      while (timeElapsed > (peakTime+950)) {
        // if we missed the peak then adjust it later by 1 second
        peakTime += 1000;
      }
      while (timeElapsed < (peakTime+950));
      LED_BLUE_Off();

      //-- Sample for 99ms (slightly less than max buffer of 100ms)
      LED_GREEN_On();
      LED_RED_On();

      PDB_Enable();
      while (timeElapsed < (peakTime+950+99)) {
        LED_RED_Toggle();
      }
      PDB_Disable();

      LED_RED_Off();

      //---------------------------------------------------------------------------
      /*//-- Calculate the threshold value / magnitude threshold
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
        //
//        if (Goertzel_magnitude[0] > postUpperLimit) {
//          postUpperLimit = Goertzel_magnitude[0];
//        }

      }
      LED_GREEN_Off();*/

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
      const int movingAveragePeriod = 75;
      const float threshold = 400 * 1e6;
      float previous_magnitude[4] = {0,0,0,0};
      int ticks[4] = {0,0,0,0};

      float exceededThreshold = upperLimit * 0.5;
      bool exceededQuit = false;

      // Loop moving average
      for (int i = movingAveragePeriod; i < (ADC_BUFFER_SIZE-GOERTZEL_SAMPLE_SIZE); i++) {
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
            if ( (Goertzel_magnitude[y] - previous_magnitude[y])*normalisation[y] > threshold) {
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
        Serial.print("ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
        Serial.print("ADC_LIB_CFG1_ADIV: "); Serial.print(  pow(2, (get_CFG_VERY_HIGH_SPEED(ADC_F_BUS)>>5) & 0x03)  ); Serial.println(" divisor");
  
        Serial.print("Target frequency /1e3: "); Serial.println(targetFrequency/1e3, DEC);
        Serial.print("Upper limit /1e9: "); Serial.println(upperLimit/1e9, DEC);
        //Serial.print("Lowest limit: "); Serial.println(lowerLimit, DEC);
        Serial.print("Threshold /1e6: "); Serial.println(threshold/1e6, DEC);
  
        Serial.println("Hydrophone_Index : "); Serial.println(Hydrophone_Index, DEC);
  
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
  
        // goertzel outputs
        Serial.println("------");
        Serial.println("Goertzel:");

        // Triger send now to clear buffer
        Serial.send_now();

        for (int i = 0; i < (ADC_BUFFER_SIZE-GOERTZEL_SAMPLE_SIZE); i++) {        
          Goertzel_MovingAverage(i, movingAveragePeriod);
  
          int payload_size = snprintf(textBuffer, textBufferLength,"%.02f,%.02f,%.02f,%.02f,%d\n",
                   Goertzel_magnitude[0]*1e-6*normalisation[0], Goertzel_magnitude[1]*1e-6*normalisation[1],
                   Goertzel_magnitude[2]*1e-6*normalisation[2], Goertzel_magnitude[3]*1e-6*normalisation[3],
                   i);
  
          // Flow control
          while (Serial.availableForWrite() <= payload_size); // waits until the serial output buffer has enough space
  
          // Write payload to serial (fast)
          Serial.write(textBuffer, payload_size);
          Serial.send_now(); // trigger send now
        
          // Flash LED
          // if (i % 64) {
          if (i & 128) {
            LED_GREEN_Toggle();
          }
        }

        Serial.println("------");
        Serial.println("#END#");
  
        Serial.send_now(); // trigger send now
      }

      // Change frequency if needed
      delay(50);
      // TODO: logic to change frequency
      //targetFrequency = (HYDROPHONE_FREQ_FLARE); // 37.5kHz
      targetFrequency = (HYDROPHONE_FREQ_DRUM); // 45kHz

      LED_WHITE_Off();
      LED_GREEN_Off();
      LED_BLUE_Off();
    }
}

void debug_array(String name, uint16_t * array, int length) {
  Serial.println("===");
  Serial.print(name + " = [");
  for (int x = 0; x < length; x++) {
  Serial.print(array[x], DEC);
  Serial.print(",");
  }
  Serial.println("]");
}

void loop() {
  // Should never reach here
  digitalWrite(PIN_LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);
  digitalWrite(PIN_LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(250);
}


void loop_test2() {
    // BENCHMARK
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    long before = millis();
    for (uint32_t i = 0; i < 1000000; i++) {
        sampleHydrophones();
        //adc_result1 = adc->analogSynchronizedRead(A14, A15);
        //adc_result2 = adc->analogSynchronizedRead(A16, A17);
    }
    long after = millis();
    //digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW

    // Print time taken
    Serial.print("Time taken: ");
    Serial.print(after-before, DEC);
    Serial.println(" ms");

    // Hydrophone1_Values
    Serial.print("ADC (1): ");
    Serial.print(Hydrophone0_Values[0], DEC);
    Serial.println("");
    Serial.print("ADC (2): ");
    Serial.print(Hydrophone1_Values[0], DEC);
    Serial.println("");
    Serial.print("ADC (3): ");
    Serial.print(Hydrophone2_Values[0], DEC);
    Serial.println("");
    Serial.print("ADC (4): ");
    Serial.print(Hydrophone3_Values[0], DEC);
    Serial.println("");
}
