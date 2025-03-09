################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306/ssd1306.c \
../Drivers/SSD1306/ssd1306_fonts.c \
../Drivers/SSD1306/ssd1306_tests.c 

C_DEPS += \
./Drivers/SSD1306/ssd1306.d \
./Drivers/SSD1306/ssd1306_fonts.d \
./Drivers/SSD1306/ssd1306_tests.d 

OBJS += \
./Drivers/SSD1306/ssd1306.o \
./Drivers/SSD1306/ssd1306_fonts.o \
./Drivers/SSD1306/ssd1306_tests.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306/%.o Drivers/SSD1306/%.su: ../Drivers/SSD1306/%.c Drivers/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SSD1306

clean-Drivers-2f-SSD1306:
	-$(RM) ./Drivers/SSD1306/ssd1306.d ./Drivers/SSD1306/ssd1306.o ./Drivers/SSD1306/ssd1306.su ./Drivers/SSD1306/ssd1306_fonts.d ./Drivers/SSD1306/ssd1306_fonts.o ./Drivers/SSD1306/ssd1306_fonts.su ./Drivers/SSD1306/ssd1306_tests.d ./Drivers/SSD1306/ssd1306_tests.o ./Drivers/SSD1306/ssd1306_tests.su

.PHONY: clean-Drivers-2f-SSD1306

