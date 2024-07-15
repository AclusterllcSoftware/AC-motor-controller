################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := OC_Test_FWV3.4.6_Robince_20240421
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
OC_Test_FWV3.4.6_Robince_20240421.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
OC_Test_FWV3.4.6_Robince_20240421.list \

OBJCOPY_HEX += \
OC_Test_FWV3.4.6_Robince_20240421.hex \

OBJCOPY_BIN += \
OC_Test_FWV3.4.6_Robince_20240421.bin \


# All Target
all: main-build

# Main-build Target
main-build: OC_Test_FWV3.4.6_Robince_20240421.elf secondary-outputs

# Tool invocations
OC_Test_FWV3.4.6_Robince_20240421.elf: $(OBJS) $(USER_OBJS) D:\acluster\ LLC\AC\ motor\ Controller\ Triac\OC_Test_FWV3.4.6_Robince_20240421\OC_Test_FWV3.4.6_Robince_20240421\STM32F030CCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "OC_Test_FWV3.4.6_Robince_20240421.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"D:\acluster LLC\AC motor Controller Triac\OC_Test_FWV3.4.6_Robince_20240421\OC_Test_FWV3.4.6_Robince_20240421\STM32F030CCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="OC_Test_FWV3.4.6_Robince_20240421.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

OC_Test_FWV3.4.6_Robince_20240421.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "OC_Test_FWV3.4.6_Robince_20240421.list"
	@echo 'Finished building: $@'
	@echo ' '

OC_Test_FWV3.4.6_Robince_20240421.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "OC_Test_FWV3.4.6_Robince_20240421.hex"
	@echo 'Finished building: $@'
	@echo ' '

OC_Test_FWV3.4.6_Robince_20240421.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "OC_Test_FWV3.4.6_Robince_20240421.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) OC_Test_FWV3.4.6_Robince_20240421.bin OC_Test_FWV3.4.6_Robince_20240421.elf OC_Test_FWV3.4.6_Robince_20240421.hex OC_Test_FWV3.4.6_Robince_20240421.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
