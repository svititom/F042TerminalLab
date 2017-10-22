# 0 Overview #
This project aims to provide a simple multimeter (voltmeter, counter, pwm generator) for the STM32F042 microcontrollers via native USB and a virtuar COM port. It is used via a terminal application such a PuTTY running on the host PC. 
Since the firmware was created with the help of STM32CubeMX, porting it to any other STM32 microcontroller should be no problem if the need arises. 
![Overview](/Documentation/Images/OperationScreen.png)

# 1 Specification #
Compatible with all STM32F042 packages, including the TSSOP20

- 2 ADC channels on pins PA1 and PA2 and Vdda 
	- 100Hz sampling frequency per channel
	- Moving average from 50 samples  per channel
	- Correction to Vdda supply 
	- 5uS sampling time
	- Difference between PA1 and PA2 in voltmeter mode
- Frequency counter on PA0
	- 2 timer direct frequency measurement
	- Up to 48 MHz input
	- 1s measurement period
	- Crystal frequency correction via USB clock
- PWM Generator on PA14
	- Adjustable frequency 15Hz - 500kHz
	- Adjustable duty cycle (step depends on frequency)

# 2 Installation #
## 2.1 Hardware Requirements ##
- An STM32F042 microcontroller in any package, either on a develpoment board (such as the nucleo) or on a breakout board
- A **USB header on a breakout board** this firmware uses native USB with VCP, not the actual COM port, so the micro has to be connected via a usb cable, and not just a RS-232<->USB converter

If you have a microcontroller on a breakout board
- Reset switch
- Boot mode selector (SPDT switch)
- Decoupling capacitors (a ceramic 100nF near the micro and a 1uF electrolyt should be more than enough) 
- 3.3V regulator such as the HT7833
- Breadboard
- Jumper wires

## 2.2 Software Requirements ##
- [ST Virtual Com Port](http://www.st.com/en/development-tools/stsw-stm32102.html)

If flashing the microcontroller via ST-Link
- [St-link Utility](http://www.st.com/en/development-tools/stsw-link004.html)

If flashing the microcontroller via DFU
- [DFuSe](http://www.st.com/en/development-tools/stsw-stm32080.html) I have tested only with ST DFuSe. Any other USB compliant DFU loader should work also

## 2.3 Wiring ##

[TSSOP20 Pinout](/Documentation/Images/TSSOP20_pinout.PNG)
[LQFP32 Pinout](/Documentation/Images/LQFP32_pinout.PNG)

## 2.4 Flashing ##

Load the TerminalLab*ddmmyyyy*.hex or .dfu to the processor
If you are using the dfu file, 
- to enter DFU mode, switch the boot mode selector and reset the microcontroller. DFuSe will automatically detect your microcontroller
- to exit DFU mode either use the "Leave DFU mode" in DFuSe, or switch th eboot mode selector and reset the microcontroller

# 3 Usage #


# -1 todo: #
- actually write the readme, maybe create a git page?
- fix timer scaling at low freqs (if using registers, add one and then perform multiplication/division and subtract one)
- add schematic
- characterize accuriacies with calibrated instruments