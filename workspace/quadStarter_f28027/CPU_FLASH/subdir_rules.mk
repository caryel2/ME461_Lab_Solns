################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt -Ooff --include_path="C:/dan/GitHubs/ME461Fall22/workspace/quadStarter_f28027" --include_path="C:/dan/GitHubs/ME461Fall22/C2000Ware_4_01_00_00/device_support/f2802x/common/include" --include_path="C:/dan/GitHubs/ME461Fall22/C2000Ware_4_01_00_00/device_support/f2802x/headers/include" --include_path="C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --define=DEBUG --define=_FLASH --c11 --printf_support=minimal --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/bin/cl2000" -v28 -ml -mt -Ooff --include_path="C:/dan/GitHubs/ME461Fall22/workspace/quadStarter_f28027" --include_path="C:/dan/GitHubs/ME461Fall22/C2000Ware_4_01_00_00/device_support/f2802x/common/include" --include_path="C:/dan/GitHubs/ME461Fall22/C2000Ware_4_01_00_00/device_support/f2802x/headers/include" --include_path="C:/ti/ccs1120/ccs/tools/compiler/ti-cgt-c2000_21.6.0.LTS/include" --define=DEBUG --define=_FLASH --c11 --printf_support=minimal --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


