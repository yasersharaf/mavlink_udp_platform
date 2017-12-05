################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/libraries/UDP_Client.cpp \
../src/libraries/autopilot_interface.cpp \
../src/libraries/client.cpp \
../src/libraries/serial_port.cpp 

OBJS += \
./src/libraries/UDP_Client.o \
./src/libraries/autopilot_interface.o \
./src/libraries/client.o \
./src/libraries/serial_port.o 

CPP_DEPS += \
./src/libraries/UDP_Client.d \
./src/libraries/autopilot_interface.d \
./src/libraries/client.d \
./src/libraries/serial_port.d 


# Each subdirectory must supply rules for building sources it contributes
src/libraries/%.o: ../src/libraries/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++0x -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


