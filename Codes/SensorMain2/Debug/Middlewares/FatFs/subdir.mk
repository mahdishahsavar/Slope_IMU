################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/diskio.c \
C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/ff.c \
C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/option/syscall.c 

C_DEPS += \
./Middlewares/FatFs/ccsbcs.d \
./Middlewares/FatFs/diskio.d \
./Middlewares/FatFs/ff.d \
./Middlewares/FatFs/ff_gen_drv.d \
./Middlewares/FatFs/syscall.d 

OBJS += \
./Middlewares/FatFs/ccsbcs.o \
./Middlewares/FatFs/diskio.o \
./Middlewares/FatFs/ff.o \
./Middlewares/FatFs/ff_gen_drv.o \
./Middlewares/FatFs/syscall.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FatFs/ccsbcs.o: C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/option/ccsbcs.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/diskio.o: C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/diskio.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff.o: C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/ff.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/ff_gen_drv.o: C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/FatFs/syscall.o: C:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src/option/syscall.c Middlewares/FatFs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include -I"C:/Users/mshahsavar/STM32CubeIDE/workspace_1.9.0/SensorMain2/Drivers/SSD1306" -I../FATFS/Target -I../FATFS/App -IC:/Users/mshahsavar/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.0/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-FatFs

clean-Middlewares-2f-FatFs:
	-$(RM) ./Middlewares/FatFs/ccsbcs.d ./Middlewares/FatFs/ccsbcs.o ./Middlewares/FatFs/ccsbcs.su ./Middlewares/FatFs/diskio.d ./Middlewares/FatFs/diskio.o ./Middlewares/FatFs/diskio.su ./Middlewares/FatFs/ff.d ./Middlewares/FatFs/ff.o ./Middlewares/FatFs/ff.su ./Middlewares/FatFs/ff_gen_drv.d ./Middlewares/FatFs/ff_gen_drv.o ./Middlewares/FatFs/ff_gen_drv.su ./Middlewares/FatFs/syscall.d ./Middlewares/FatFs/syscall.o ./Middlewares/FatFs/syscall.su

.PHONY: clean-Middlewares-2f-FatFs

