################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/can_frame.c \
../src/decoder.c \
../src/main.c \
../src/monitor.c \
../src/static_message.c 

OBJS += \
./src/can_frame.o \
./src/decoder.o \
./src/main.o \
./src/monitor.o \
./src/static_message.o 

C_DEPS += \
./src/can_frame.d \
./src/decoder.d \
./src/main.d \
./src/monitor.d \
./src/static_message.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/can_frame.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


