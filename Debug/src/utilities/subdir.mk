################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/utilities/Logger.cpp \
../src/utilities/conversionUtilities.cpp \
../src/utilities/utilities.cpp 

OBJS += \
./src/utilities/Logger.o \
./src/utilities/conversionUtilities.o \
./src/utilities/utilities.o 

CPP_DEPS += \
./src/utilities/Logger.d \
./src/utilities/conversionUtilities.d \
./src/utilities/utilities.d 


# Each subdirectory must supply rules for building sources it contributes
src/utilities/%.o: ../src/utilities/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


