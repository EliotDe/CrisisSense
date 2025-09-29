# CrisisSense

## Description
This is a prototype for a low-power environmental sensor module built with an stm32 (stm32l432), a bme280 environmental sensor and LoRa comms. It was built for use in rural areas / areas with little existing communication infrastructure. 

## Status
I am currently testing and debugging my sensor integration code, and will be working on LoRa integration next. This project uses SPI to communicate between the sensor and communication modules and USART to debug the code. I have integrated the bme280 API code and ARM/STM's CMSIS code, found at X and Y repos, respectively. 

## Toolchain
ARM GNU toolchain. I use my own makefile and linker script found in the repos for compilation and linking, as well as flashing and testing. 

## Next Steps

- Get a simple P2P LoRa working
- Upgrade the current "CI pipeline" to use docker
- PCB design?
- Move to the LoRaWAN
