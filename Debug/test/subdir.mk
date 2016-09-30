################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/EdgePointSpaceCarver.cpp \
../test/KittiCamParser.cpp \
../test/KittiRectifiedCamParser.cpp \
../test/Main.cpp \
../test/PathCreator.cpp \
../test/ReconstructFromSfMData.cpp \
../test/ReconstructorFromOut.cpp \
../test/Track.cpp \
../test/VideoSimulator.cpp 

OBJS += \
./test/EdgePointSpaceCarver.o \
./test/KittiCamParser.o \
./test/KittiRectifiedCamParser.o \
./test/Main.o \
./test/PathCreator.o \
./test/ReconstructFromSfMData.o \
./test/ReconstructorFromOut.o \
./test/Track.o \
./test/VideoSimulator.o 

CPP_DEPS += \
./test/EdgePointSpaceCarver.d \
./test/KittiCamParser.d \
./test/KittiRectifiedCamParser.d \
./test/Main.d \
./test/PathCreator.d \
./test/ReconstructFromSfMData.d \
./test/ReconstructorFromOut.d \
./test/Track.d \
./test/VideoSimulator.d 


# Each subdirectory must supply rules for building sources it contributes
test/%.o: ../test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


