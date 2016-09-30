################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../external/glm/glm/detail/dummy.cpp \
../external/glm/glm/detail/glm.cpp 

OBJS += \
./external/glm/glm/detail/dummy.o \
./external/glm/glm/detail/glm.o 

CPP_DEPS += \
./external/glm/glm/detail/dummy.d \
./external/glm/glm/detail/glm.d 


# Each subdirectory must supply rules for building sources it contributes
external/glm/glm/detail/%.o: ../external/glm/glm/detail/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


