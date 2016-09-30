################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../external/glm/test/gtc/gtc_bitfield.cpp \
../external/glm/test/gtc/gtc_constants.cpp \
../external/glm/test/gtc/gtc_epsilon.cpp \
../external/glm/test/gtc/gtc_integer.cpp \
../external/glm/test/gtc/gtc_matrix_access.cpp \
../external/glm/test/gtc/gtc_matrix_integer.cpp \
../external/glm/test/gtc/gtc_matrix_inverse.cpp \
../external/glm/test/gtc/gtc_matrix_transform.cpp \
../external/glm/test/gtc/gtc_noise.cpp \
../external/glm/test/gtc/gtc_packing.cpp \
../external/glm/test/gtc/gtc_quaternion.cpp \
../external/glm/test/gtc/gtc_random.cpp \
../external/glm/test/gtc/gtc_reciprocal.cpp \
../external/glm/test/gtc/gtc_round.cpp \
../external/glm/test/gtc/gtc_type_precision.cpp \
../external/glm/test/gtc/gtc_type_ptr.cpp \
../external/glm/test/gtc/gtc_ulp.cpp \
../external/glm/test/gtc/gtc_user_defined_types.cpp \
../external/glm/test/gtc/gtc_vec1.cpp 

OBJS += \
./external/glm/test/gtc/gtc_bitfield.o \
./external/glm/test/gtc/gtc_constants.o \
./external/glm/test/gtc/gtc_epsilon.o \
./external/glm/test/gtc/gtc_integer.o \
./external/glm/test/gtc/gtc_matrix_access.o \
./external/glm/test/gtc/gtc_matrix_integer.o \
./external/glm/test/gtc/gtc_matrix_inverse.o \
./external/glm/test/gtc/gtc_matrix_transform.o \
./external/glm/test/gtc/gtc_noise.o \
./external/glm/test/gtc/gtc_packing.o \
./external/glm/test/gtc/gtc_quaternion.o \
./external/glm/test/gtc/gtc_random.o \
./external/glm/test/gtc/gtc_reciprocal.o \
./external/glm/test/gtc/gtc_round.o \
./external/glm/test/gtc/gtc_type_precision.o \
./external/glm/test/gtc/gtc_type_ptr.o \
./external/glm/test/gtc/gtc_ulp.o \
./external/glm/test/gtc/gtc_user_defined_types.o \
./external/glm/test/gtc/gtc_vec1.o 

CPP_DEPS += \
./external/glm/test/gtc/gtc_bitfield.d \
./external/glm/test/gtc/gtc_constants.d \
./external/glm/test/gtc/gtc_epsilon.d \
./external/glm/test/gtc/gtc_integer.d \
./external/glm/test/gtc/gtc_matrix_access.d \
./external/glm/test/gtc/gtc_matrix_integer.d \
./external/glm/test/gtc/gtc_matrix_inverse.d \
./external/glm/test/gtc/gtc_matrix_transform.d \
./external/glm/test/gtc/gtc_noise.d \
./external/glm/test/gtc/gtc_packing.d \
./external/glm/test/gtc/gtc_quaternion.d \
./external/glm/test/gtc/gtc_random.d \
./external/glm/test/gtc/gtc_reciprocal.d \
./external/glm/test/gtc/gtc_round.d \
./external/glm/test/gtc/gtc_type_precision.d \
./external/glm/test/gtc/gtc_type_ptr.d \
./external/glm/test/gtc/gtc_ulp.d \
./external/glm/test/gtc/gtc_user_defined_types.d \
./external/glm/test/gtc/gtc_vec1.d 


# Each subdirectory must supply rules for building sources it contributes
external/glm/test/gtc/%.o: ../external/glm/test/gtc/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


