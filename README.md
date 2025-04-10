# Final-Project

This GitHub repository contains the code required to implement TCP and UDP on the STM32F439ZI Development Board, alongside the data visualisation tool for UDP throughput as well as the existing filter implementation. 


The folder titled "TCP/IP implementation" contains two important files are the "main.c" and "main.h" files under \core\src, with the other files being either driver files or CubeMX-generated code for middleware and GPIO functionality.  "main.c" contains the primary code running on the STM32 microcontroller, including the function prototypes and the state machine implementation as highlighted in Section 3.2.4 of the thesis. The "main.h" file contains all custom struct definitions used to store the client context and event state for successful TCP implementation 

Similar to the TCP/IP implementation folder, "filter" contains all the relevant code required to run the IIR filter on the STM32 microcontroller. "main.c" contains the primary code running on the STM32 microcontroller, and it is currently set up to output the impulse response of the filter, however, this can easily be modified through the use of a parameter. 

The final folder, "data visualisation" contains the Python script used to measure the TCP throughput from the STM32. Its main dependencies are "socket", "matplotlib", "sys" and "numpy", used to either create the graphs, record the data or to aid in data visualisation. These can all be installed using pip.  
