void setup() {
  /* Teensy UART communication */
  Serial1.begin(115200);
}

char Serial1_Buffer[100] = {0};
int Serial1_Index = 0;

void loop() {

  /* Teensy UART communication */
  //   Format string: %d,%d,%d,%d,%d;
  //   Identifier: freq,h1,h2,h3,h4;
  //   Example for pinger 1: 37500,2877,3075,3432,2402;
  //   Example for pinger 2: 45000,2877,3075,3432,2402;
  while (Serial1.available() >= 0) {
    char receivedChar = Serial1.read();

    // Valid chars are digits between 0 to 9 inclusive and the comma delimiter
    bool isValid = ('0' <= receivedChar && receivedChar <= '9') || receivedChar == ',';

    /* if: valid character, then store in buffer */
    if (isValid) {
      Serial1_Buffer[Serial1_Index] = receivedChar;
      Serial1_Index++;

    /* else if: it is the terminating character then process the line and submit to CAN */
    } else if (receivedChar == ';') {
      // Terminate string
      Serial1_Buffer[Serial1_Index] = '\0';
      Serial1_Index = 0;

      // Process line
      int freq, h1, h2, h3, h4;
      sscanf(Serial1_Buffer, "%d,%d,%d,%d,%d", &freq, &h1, &h2, &h3, &h4);

      // Send to CAN: 4x unsigned 16 bit integer
      CANMessage transmit;
      transmit.len = 8;
      transmit.data16[0] = h1;
      transmit.data16[1] = h2;
      transmit.data16[2] = h3;
      transmit.data16[3] = h4;

      // 0x301: Values for first set of pingers
      // 0x302: Values for second set of pingers
      if (freq == 37500) {
          transmit.id = 0x301;
      } else if (freq == 45000) {
          transmit.id = 0x302;
      } else {
          transmit.id = 0x303; // Unknown frequency lol
      }

      const bool ok = can.tryToSend(transmit);
      if (ok) {
        //Serial.print("Teensy CAN msg sent: ");
        //Serial.println(Serial1_Buffer);
      }

    /* else: ignore all invalid chars */
    } else {
      // do nothing
    }
  }
}
