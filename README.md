# ORX_Boot
STM32 Compatible Serial Bootloader for the ORX (OrangeRx) module used with Multiprotocol and Flash-Multi.

This serial bootloader is compatible with the STM32 bootloader, however it runs on a Atmel / Microchip
ATXmega32d4 as fitted in the OrangeRx "ORX" RF module.
It allows the use of the flash-multi tool by Ben Lye.
For the purpose of loading new application firmware to the ORX module, the bootloader emulates a STM32F103#6.
The xmega has 32KB application section, plus a 4KB bootloader section.

To program this bootloader you will need a PDI capable programmer such as the Olimex AVR-ISP-MK2.
Either build the code yourself or program the file "stmboot-boot.hex" into the ORX module.

To program the .hex file into the ORX module use the following command.

avrdude -c avrisp2 -p x32d4 -Uboot:w:stmboot-boot.hex:i

To build the bootloader from source, do the following :-

make clean

make all

make program


Ideally the xmega32d4 fuses should be :-

fuse1 0x00

fuse2 0xBF

fuse4 0xFF

fuse5 0xF7

TO ACTIVATE THE BOOTLOADER. POWER IT UP WITH THE "ID" BUTTON PRESSED.
The LED will stay lit and the module will beep briefly.

Related Repositories and information.

https://www.st.com/resource/en/application_note/an3155-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf

https://github.com/benlye/flash-multi

https://github.com/pascallanger/DIY-Multiprotocol-TX-Module
