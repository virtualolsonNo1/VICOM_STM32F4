# VICOM DESCRIPTION
- This is a communications protocol I wrote for my job (VICOM standing for VI Communications protocol) using UART and DMA to communicate with different nodes on the robot on an STM32F411 microcontroller. I've now written it for Nuvoton, SAM G, and ST microcontrollers, with this being the most recent ST example.

# HOW IT WORKS
- This comms protocol is meant to successfully transfer the correct state struct. To do so, UART, DMA, protocol buffers, and CRC checks were used to send and error-check the data being transmitted and received. Additionally, address mark detection was used to allow the slaves to wait in mute mode until an address bit (the MSB) of 1 is received, not wasting unnecessary CPU time on processing unnecessary data received over the comms bus to all of the slaves. 
- Data to each of the slaves is done in the format of the address (with an address bit of 1), followed by the length of the payload, CRC check, the payload itself, and another CRC check to ensure the data came through correct. 

# MISSING ASPECTS FROM CODE SHOWN
- Since this was code for my job, I am unable to share the entire code base or the spinning up and configuration of the proper peripherals at the start of everything, etc., but it has been thoroughly tested and is in our production code. 
- Additionally, code such as interrupt handlers, which then call the functions here, are also not shown in the code provided, but such functionality was implemented by me as part of this project and is implied through the enabling of particular interrupts (i.e. code enabling the proper DMA stream as well as it's transfer complete interrupt before starting a transmission, etc.), as well as through function names (i.e. VICCOM_uartHandler)
- Implementing this comms protocol for an STM32F4 microcontroller was only the first aspect of this project for me, after which I had to successfully add it to our production code base, make it utilize protobuffers and work within our existing system, etc, all of which sadly cannot be shown here due to the limited scope of the files shown.
