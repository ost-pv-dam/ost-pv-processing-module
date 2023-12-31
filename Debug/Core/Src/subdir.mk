################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/freertos.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

CPP_SRCS += \
../Core/Src/ESP32.cpp \
../Core/Src/MPL3115A2.cpp \
../Core/Src/SHT30.cpp \
../Core/Src/SMU.cpp \
../Core/Src/VC0706.cpp \
../Core/Src/data.cpp \
../Core/Src/logger.cpp \
../Core/Src/main.cpp \
../Core/Src/real_time_clock.cpp \
../Core/Src/selector.cpp \
../Core/Src/thermistor_array.cpp 

C_DEPS += \
./Core/Src/freertos.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 

OBJS += \
./Core/Src/ESP32.o \
./Core/Src/MPL3115A2.o \
./Core/Src/SHT30.o \
./Core/Src/SMU.o \
./Core/Src/VC0706.o \
./Core/Src/data.o \
./Core/Src/freertos.o \
./Core/Src/logger.o \
./Core/Src/main.o \
./Core/Src/real_time_clock.o \
./Core/Src/selector.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/thermistor_array.o 

CPP_DEPS += \
./Core/Src/ESP32.d \
./Core/Src/MPL3115A2.d \
./Core/Src/SHT30.d \
./Core/Src/SMU.d \
./Core/Src/VC0706.d \
./Core/Src/data.d \
./Core/Src/logger.d \
./Core/Src/main.d \
./Core/Src/real_time_clock.d \
./Core/Src/selector.d \
./Core/Src/thermistor_array.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.cpp Core/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fexceptions -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano_c_standard_cpp.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano_c_standard_cpp.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ESP32.cyclo ./Core/Src/ESP32.d ./Core/Src/ESP32.o ./Core/Src/ESP32.su ./Core/Src/MPL3115A2.cyclo ./Core/Src/MPL3115A2.d ./Core/Src/MPL3115A2.o ./Core/Src/MPL3115A2.su ./Core/Src/SHT30.cyclo ./Core/Src/SHT30.d ./Core/Src/SHT30.o ./Core/Src/SHT30.su ./Core/Src/SMU.cyclo ./Core/Src/SMU.d ./Core/Src/SMU.o ./Core/Src/SMU.su ./Core/Src/VC0706.cyclo ./Core/Src/VC0706.d ./Core/Src/VC0706.o ./Core/Src/VC0706.su ./Core/Src/data.cyclo ./Core/Src/data.d ./Core/Src/data.o ./Core/Src/data.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/real_time_clock.cyclo ./Core/Src/real_time_clock.d ./Core/Src/real_time_clock.o ./Core/Src/real_time_clock.su ./Core/Src/selector.cyclo ./Core/Src/selector.d ./Core/Src/selector.o ./Core/Src/selector.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/thermistor_array.cyclo ./Core/Src/thermistor_array.d ./Core/Src/thermistor_array.o ./Core/Src/thermistor_array.su

.PHONY: clean-Core-2f-Src

