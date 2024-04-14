################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpios.c \
../Src/keypad_4x4.c \
../Src/main.c \
../Src/stepper_motor.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_settings.c 

OBJS += \
./Src/gpios.o \
./Src/keypad_4x4.o \
./Src/main.o \
./Src/stepper_motor.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_settings.o 

C_DEPS += \
./Src/gpios.d \
./Src/keypad_4x4.d \
./Src/main.d \
./Src/stepper_motor.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_settings.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpios.cyclo ./Src/gpios.d ./Src/gpios.o ./Src/gpios.su ./Src/keypad_4x4.cyclo ./Src/keypad_4x4.d ./Src/keypad_4x4.o ./Src/keypad_4x4.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/stepper_motor.cyclo ./Src/stepper_motor.d ./Src/stepper_motor.o ./Src/stepper_motor.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_settings.cyclo ./Src/system_settings.d ./Src/system_settings.o ./Src/system_settings.su

.PHONY: clean-Src

