################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../external/glm/test/external/gli/core/dummy.cpp 

OBJS += \
./external/glm/test/external/gli/core/dummy.o 

CPP_DEPS += \
./external/glm/test/external/gli/core/dummy.d 


# Each subdirectory must supply rules for building sources it contributes
external/glm/test/external/gli/core/%.o: ../external/glm/test/external/gli/core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


