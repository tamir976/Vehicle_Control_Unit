################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/can_frame.c \
../src/control.c \
../src/decoder.c \
../src/encoder.c \
../src/flexcan_conf.c \
../src/hacked_panda.c \
../src/main.c \
../src/monitor.c \
../src/oled_display.c \
../src/static_message.c 

OBJS += \
./src/can_frame.o \
./src/control.o \
./src/decoder.o \
./src/encoder.o \
./src/flexcan_conf.o \
./src/hacked_panda.o \
./src/main.o \
./src/monitor.o \
./src/oled_display.o \
./src/static_message.o 

C_DEPS += \
./src/can_frame.d \
./src/control.d \
./src/decoder.d \
./src/encoder.d \
./src/flexcan_conf.d \
./src/hacked_panda.d \
./src/main.d \
./src/monitor.d \
./src/oled_display.d \
./src/static_message.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/can_frame.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


