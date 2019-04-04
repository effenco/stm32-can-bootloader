# stm32-can-bootloader
can bootloader for use with stm32 microcontrollers

everything in this repo will be specific for use with STM32F373VCT6 MCU but you can follow the migration guide to use it with any stm32 except series that uses the M0 core (because of the lack of the SCB->VTOR register)

## Getting Started/Prerequisites

### Software tools
This project uses the free tools from ST and the STM32 HAL library from ST. This can be easily ported to another compiler/ide.

Install CubeMx from ST website : https://www.st.com/en/development-tools/stm32cubemx.html
Install Atollic Truestudio from Atollic/stm32 website : https://atollic.com/truestudio/

furthermore, to use the loader, you will need a python environement, we suggest Anaconda : https://www.anaconda.com/distribution/

you will also need the following python packages (install using pip in the anaconda prompt):
* pip install intelhex
* pip install crcmod
* pip install python-can

### Project organisation

#### BOOT
Contain the bootloader project.
You can build this using atollic.

#### APP
Contain a demo application to load.
You can build this using atollic.

#### LOADER
Contain the python script used to load the application using the bootloader.

### Hardware

#### Target

You will need any Board with a STM32F373VCT (otherwise you can port using the Migration guide).

##### default configurations are :

you can see this graphically using STM32CubeMx in the .ioc file
* SWDIO on pin PA13 (72)
* SWCLK on pin PA14 (76)
* a RGB led on pins PA15 (77) ,PC10 (78) and PC11(79)
* CANrx on pin PD0 (81)
* CANtx on pin PD1(82)

CAN bitrate is set at 500 kbps but can be adjusted in STM32CubeMx

#### Debug adapter

ST-link/V2 usb swd adapter is used to program and debug. see "Debug configurations" section for more informations.

#### CAN-USB Adapter

We use the pcan-usb adapter from Peak system, but any adapter supported by the pyhton-can library will work with minor change to the loader script (just one line).
https://www.peak-system.com/PCAN-USB.199.0.html?&L=1

## Bootloader information
		Flash memory organisation (STM32F373VCT6)

		0x8040000 -> +-------------+
					 | 0x803F000   |	\
					 |    to       |	 \
					 | 0x803FFFF   |	  |- Reserved for other informations
					 |   4 KB      |	 /
					 | data        |	/
		0x803F000 -> +-------------+
					 |             |	\
					 |             |	 \
					 |             |	  |
					 |             |	  |
					 | 0x8008000   |	  |
					 |    to       |	  |
					 | 0x803EFFF   |	  |- Contain the application software
					 |   220 KB    |	  |
					 | application |	  |
					 |             |	  |
					 |             |	  |
					 |             |	 /
					 |             |	/
		0x8008000 -> +-------------+
					 |             |	\
					 | 0x8000000   |	 \
					 |    to       |	  |
					 | 0x8007FFF   |	  |- Contain bootloader software
					 |   32 KB     |	  |
					 | bootloader  |	 /
					 |             |	/
		0x8000000 -> +-------------+
		
## Debug configurations

three debug configurations are provided, in atollic, use right-click and "debug as" and "Debug configurations" to choose the one ou wish to use at the moment.

### BOOT Debug configuration

Use this configuration to load and debug the Bootloader only

### APP Debug configuration

Use this configuration to load and debug the demo application only (won't boot after disconnected from st-link/V2 because there will be nothing at address 0x8000000)

### BOOT+APP Debug configuration

Use this to load the complete bootloader and demo application.

## Using the bootloader

* edit the loader script with the path of the hex file to load
* Start the loader script
* Reboot the board in your prefered way

## Special guidelines
* No merge to Master branch on fridays
* don't drink and commit
* Commit responsibly

## Migration guide

migration guide will be added soon.

## FAQ

None at the moment

