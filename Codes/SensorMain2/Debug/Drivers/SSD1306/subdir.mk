################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306/ssd1306.c \
../Drivers/SSD1306/ssd1306_fonts.c \
../Drivers/SSD1306/ssd1306_tests.c 

CPP_SRCS += \
../Drivers/SSD1306/RJA_SSD1306.cpp 

C_DEPS += \
./Drivers/SSD1306/ssd1306.d \
./Drivers/SSD1306/ssd1306_fonts.d \
./Drivers/SSD1306/ssd1306_tests.d 

OBJS += \
./Drivers/SSD1306/RJA_SSD1306.o \
./Drivers/SSD1306/ssd1306.o \
./Drivers/SSD1306/ssd1306_fonts.o \
./Drivers/SSD1306/ssd1306_tests.o 

CPP_DEPS += \
./Drivers/SSD1306/RJA_SSD1306.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306/%.o Drivers/SSD1306/%.su: ../Drivers/SSD1306/%.cpp Drivers/SSD1306/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/SSD1306/%.o Drivers/SSD1306/%.su: ../Drivers/SSD1306/%.c Drivers/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306

clean-Drivers-2f-SSD1306:
	-$(RM) ./Drivers/SSD1306/RJA_SSD1306.d ./Drivers/SSD1306/RJA_SSD1306.o ./Drivers/SSD1306/RJA_SSD1306.su ./Drivers/SSD1306/ssd1306.d ./Drivers/SSD1306/ssd1306.o ./Drivers/SSD1306/ssd1306.su ./Drivers/SSD1306/ssd1306_fonts.d ./Drivers/SSD1306/ssd1306_fonts.o ./Drivers/SSD1306/ssd1306_fonts.su ./Drivers/SSD1306/ssd1306_tests.d ./Drivers/SSD1306/ssd1306_tests.o ./Drivers/SSD1306/ssd1306_tests.su

.PHONY: clean-Drivers-2f-SSD1306

