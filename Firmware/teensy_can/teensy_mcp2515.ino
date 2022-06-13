#include <ACAN2515.h>
// Error codes and possible causes:
//    In case you see "Configuration error 0x1", the Arduino doesn't communicate
//       with the 2515. You will get this error if there is no CAN shield or if
//       the CS pin is incorrect.
//    In case you see succes up to "Sent: 17" and from then on "Send failure":
//       There is a problem with the interrupt. Check if correct pin is configured
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_SCK = 13 ; // SCK input of MCP2515
static const byte MCP2515_SI  = 11 ; // SI input of MCP2515
static const byte MCP2515_SO  = 12 ; // SO output of MCP2515

static const byte MCP2515_CS  = 15 ; // CS input of MCP2515 (adapt to your design)

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, 255) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 8 MHz


void setup () {
  //--- Start serial
  Serial.begin (9600) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }


  Serial.print ("Using pin #") ;
  Serial.print (MCP2515_SI) ;
  Serial.print (" for MOSI: ") ;
  Serial.println (SPI.pinIsMOSI (MCP2515_SI) ? "yes" : "NO!!!") ;
  Serial.print ("Using pin #") ;
  Serial.print (MCP2515_SO) ;
  Serial.print (" for MISO: ") ;
  Serial.println (SPI.pinIsMISO (MCP2515_SO) ? "yes" : "NO!!!") ;
  Serial.print ("Using pin #") ;
  Serial.print (MCP2515_SCK) ;
  Serial.print (" for SCK: ") ;
  Serial.println (SPI.pinIsSCK (MCP2515_SCK) ? "yes" : "NO!!!") ;
  SPI.setMOSI (MCP2515_SI) ;
  SPI.setMISO (MCP2515_SO) ;
  SPI.setSCK (MCP2515_SCK) ;
  
  //--- Begin SPI
  SPI.begin () ;
  
  //--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode ; // Select normal mode
  const uint16_t errorCode = can.begin (settings, NULL) ;
  
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  } else {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//----------------------------------------------------------------------------------------------------------------------

static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  can.poll ();
  CANMessage transmit;
  transmit.len = 8;
  
  transmit.id = 3;
  transmit.datac[0] = 't';
  transmit.datac[1] = 'e';
  transmit.datac[2] = 'e';
  transmit.datac[3] = 'n';
 

  if (gSendDate <= millis ()) {
    const bool ok = can.tryToSend (transmit) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.print (gSentCount) ;
      Serial.print (" msg sent: ") ;
      Serial.println (transmit.datac);
    }
  }
  
  CANMessage frame ;

  if (can.receive(frame)) {
    gReceivedFrameCount ++ ;
    Serial.print ("Received: ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print(" ID ");
    Serial.print(frame.id);
    Serial.print(" Msg: ");
    Serial.println(frame.datac);
  }
 
}
