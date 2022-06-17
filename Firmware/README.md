# Firmware

### stm32h750_acoustics

- Compile firmware using Arduino and stm32duino package for the STM32H750 dev board
- Time critical code is written in LL library or direct registers. Non-time critical codes are written in HAL or Arduino library.

### teensy_can

- Code for testing CAN and serial comms

### teensy_sketch_dec09b

- Archived code for Teensy 3.6 

### hydrophone-simulator

FPGA Verilog code to produce 45kHz and 37.5kHz bursts for purpose of simulating hydrophone signals to the filter board. The bursts occur on 2 pins which are delayed to simulate time-difference of arrival.

Hardware used:
- [Tang Nano 1K](https://wiki.sipeed.com/hardware/en/tang/Tang-Nano-1K/Nano-1k.html)

Connect these pins from the FPGA to the filter board.

    pulse45[0] = 38
    pulse45[1] = 39

    pulse37_5[0] = 40
    pulse37_5[1] = 41


