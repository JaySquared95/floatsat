################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Template.cpp 

OBJS += \
./Template.o 

CPP_DEPS += \
./Template.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C++ Compiler'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DUSE_STM32_DISCOVERY -DSTM32F40_41xxx -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\STM32F4xx_StdPeriph_Driver\inc" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\CMSIS\Device\ST\STM32F4xx\Include" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\CMSIS\Include" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\hal" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal-generic" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\independent\gateway" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\independent" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\api" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\sdCard" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\api\hal" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\default_usr_configs" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\support_libs" -I"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\support_libs\flash\spiFlash_AT45DBxxx" -fabi-version=0 -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


