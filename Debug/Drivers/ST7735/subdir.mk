################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ST7735/fonts.c \
../Drivers/ST7735/st7735.c 

OBJS += \
./Drivers/ST7735/fonts.o \
./Drivers/ST7735/st7735.o 

C_DEPS += \
./Drivers/ST7735/fonts.d \
./Drivers/ST7735/st7735.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ST7735/%.o Drivers/ST7735/%.su Drivers/ST7735/%.cyclo: ../Drivers/ST7735/%.c Drivers/ST7735/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F030xC -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/ST7735 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-ST7735

clean-Drivers-2f-ST7735:
	-$(RM) ./Drivers/ST7735/fonts.cyclo ./Drivers/ST7735/fonts.d ./Drivers/ST7735/fonts.o ./Drivers/ST7735/fonts.su ./Drivers/ST7735/st7735.cyclo ./Drivers/ST7735/st7735.d ./Drivers/ST7735/st7735.o ./Drivers/ST7735/st7735.su

.PHONY: clean-Drivers-2f-ST7735

