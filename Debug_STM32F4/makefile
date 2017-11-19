################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
STM32F4_RODOS_Template.hex \

SECONDARY_SIZE += \
STM32F4_RODOS_Template.siz \


# All Target
all: STM32F4_RODOS_Template.elf secondary-outputs

# Tool invocations
STM32F4_RODOS_Template.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM C++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\src\bare-metal\stm32f4\scripts\stm32_flash.ld" -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -L"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\rodos\Debug_STM32F4" -L"I:\Eclipse Luna\New Rodos\Eclipse Luna SR1 121\Workspace\support_libs\Debug_STM32F4" -Wl,-Map,"STM32F4_RODOS_Template.map" -o "STM32F4_RODOS_Template.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

STM32F4_RODOS_Template.hex: STM32F4_RODOS_Template.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "STM32F4_RODOS_Template.elf"  "STM32F4_RODOS_Template.hex"
	@echo 'Finished building: $@'
	@echo ' '

STM32F4_RODOS_Template.siz: STM32F4_RODOS_Template.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "STM32F4_RODOS_Template.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(C++_DEPS)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(CC_DEPS)$(SECONDARY_FLASH)$(CPP_DEPS)$(CXX_DEPS)$(C_UPPER_DEPS)$(S_UPPER_DEPS) STM32F4_RODOS_Template.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
