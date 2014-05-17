################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/CCStudio_v5.4/ccsv5/tools/compiler/c6000_7.4.2

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./user_xy.obj" \
"./user_finalp.obj" \
"./pru.obj" \
"./finalpcfg_c.obj" \
"./finalpcfg.obj" \
"./MatrixMath.obj" \
"./Ladar.obj" \
"./LCDprintf.obj" \
"./ColorVision.obj" \
"./ColorLCD.obj" \
"./COECSL_mcbsp.obj" \
"./COECSL_edma3.obj" \
-l"libc.a" \
"C:/robobuddies/repo/c67xmathlib_2_01_00_00/lib/c674xfastMath.lib" \
"C:/robobuddies/repo/bsl/lib/evmomapl138_bsl.lib" \
"C:/robobuddies/repo/finalp/lnk.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: finalp.out

dependents:
	-cd C:/robobuddies/repo/bsl/ccs5/Debug && $(MAKE) all

# Tool invocations
finalp.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS) C:/robobuddies/repo/bsl/ccs5/Debug/../../lib/evmomapl138_bsl.lib
	@echo 'Building target: $@'
	@echo 'Invoking: C6000 Linker'
	"C:/CCStudio_v5.4/ccsv5/tools/compiler/c6000_7.4.2/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 -z --stack_size=0xc00 -m"finalp.map" -i"C:/CCStudio_v5.4/ccsv5/tools/compiler/c6000_7.4.2/lib" -i"C:/CCStudio_v5.4/ccsv5/tools/compiler/c6000_7.4.2/include" -i"C:/CCStudio_v5.4/bios_5_42_01_09/packages/ti/rtdx/lib/c6000" -i"C:/CCStudio_v5.4/bios_5_42_01_09/packages/ti/bios/lib" --reread_libs --warn_sections --rom_model -o "finalp.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(GEN_SRCS__QUOTED)$(GEN_MISC_FILES__QUOTED)$(C6000_EXECUTABLE_OUTPUTS__QUOTED)$(GEN_CMDS__QUOTED) "finalp.out"
	-$(RM) "COECSL_edma3.pp" "COECSL_mcbsp.pp" "ColorLCD.pp" "ColorVision.pp" "LCDprintf.pp" "Ladar.pp" "MatrixMath.pp" "finalpcfg_c.pp" "pru.pp" "user_finalp.pp" "user_xy.pp" 
	-$(RM) "finalpcfg.pp" 
	-$(RM) "COECSL_edma3.obj" "COECSL_mcbsp.obj" "ColorLCD.obj" "ColorVision.obj" "LCDprintf.obj" "Ladar.obj" "MatrixMath.obj" "finalpcfg.obj" "finalpcfg_c.obj" "pru.obj" "user_finalp.obj" "user_xy.obj" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"C:/robobuddies/repo/finalp/finalpProject/../postBuildStep_Debug.bat" PROJECT_ROOT C:/robobuddies/repo/finalp/finalpProject !
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build
C:/robobuddies/repo/bsl/ccs5/Debug/../../lib/evmomapl138_bsl.lib:

-include ../makefile.targets
