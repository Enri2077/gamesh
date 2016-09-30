################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/cam_parsers/CamParser.cpp \
../src/cam_parsers/OpenMvgParser.cpp \
../src/cam_parsers/PointsParserFromOut.cpp 

OBJS += \
./src/cam_parsers/CamParser.o \
./src/cam_parsers/OpenMvgParser.o \
./src/cam_parsers/PointsParserFromOut.o 

CPP_DEPS += \
./src/cam_parsers/CamParser.d \
./src/cam_parsers/OpenMvgParser.d \
./src/cam_parsers/PointsParserFromOut.d 


# Each subdirectory must supply rules for building sources it contributes
src/cam_parsers/%.o: ../src/cam_parsers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


