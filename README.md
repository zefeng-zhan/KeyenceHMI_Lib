# Introduction
This is a library for Arduino in PlatformIO, which provides function to connect and communicate with "Keyence Touch Panel Display VT5 series"(Keyence HMI) (via RS232). 
# Usage
1. put function cyclic() into main loop
2. use Send... to send data to HMI memories
3. use Read... to read data from a specific memory location
4. For how to use the other functions, please refer to the example file.
# Example
There's an example in \examples, and create an ini file (you can create an PIO project)

set the src_dir value in the [platformio] section in the platform.ini file to the folder where the example is located you can compile and run the example 