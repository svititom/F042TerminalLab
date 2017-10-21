# Overview #
This project aims to provide a simple multimeter (voltmeter, counter, pwm generator) for the STM32F042 microcontrollers via native USB and a virtuar COM port. It is used via a terminal application such a PuTTY running on the host PC. 
Since the firmware was created with the help of STM32CubeMX, porting it to any other STM32 microcontroller should be no problem if the need arises. 

# Installation #
## Hardware Requirements ##
- An STM32F042 microcontroller in any package, either on a develpoment board (such as the nucleo) or on a breakout board
- A **USB header on a breakout board** this firmware uses native USB with VCP, not the actual COM port, so the micro has to be connected via a usb cable, and not just a RS-232<->USB converter

If you have a microcontroller on a breakout board
- Reset switch
- Boot mode selector (SPDT switch)
- Decoupling capacitors (a ceramic 100nF near the micro and a 1uF electrolyt should be more than enough) 
- 3.3V regulator such as the HT7833
- Breadboard
- Jumper wires

## Software Requirements ##
- [ST Virtual Com Port](http://www.st.com/en/development-tools/stsw-stm32102.html)

If flashing the microcontroller via ST-Link
- [St-link Utility](http://www.st.com/en/development-tools/stsw-link004.html)

If flashing the microcontroller via DFU
- [DFuSe](http://www.st.com/en/development-tools/stsw-stm32080.html) I have tested only with ST DFuSe. Any other USB compliant DFU loader should work also

## Flashing ##

Load the TerminalLab*ddmmyyyy*.hex or .dfu to the processor

# Usage #


# todo: #
- actually write the readme, maybe create a git page?
- fix timer scaling at low freqs (if using registers, add one and then perform multiplication/division and subtract one)
- add schematic
