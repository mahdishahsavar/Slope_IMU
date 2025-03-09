################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/GY86/MPU6050.c \
../Drivers/GY86/MS5611.c 

C_DEPS += \
./Drivers/GY86/MPU6050.d \
./Drivers/GY86/MS5611.d 

OBJS += \
./Drivers/GY86/MPU6050.o \
./Drivers/GY86/MS5611.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/GY86/%.o Drivers/GY86/%.su: ../Drivers/GY86/%.c Drivers/GY86/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/SSD1306 -I../Drivers/GY86 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-GY86

clean-Drivers-2f-GY86:
	-$(RM) ./Drivers/GY86/MPU6050.d ./Drivers/GY86/MPU6050.o ./Drivers/GY86/MPU6050.su ./Drivers/GY86/MS5611.d ./Drivers/GY86/MS5611.o ./Drivers/GY86/MS5611.su

.PHONY: clean-Drivers-2f-GY86

