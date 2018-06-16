include (CMakeForceCompiler)
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

set(ARM_NONE_EABI_GCC_BIN "" CACHE PATH "Path to GCC bin")

#if (ARM_NONE_EABI_GCC_BIN STREQUAL "")
#    message(FATAL_ERROR "ARM_NONE_EABI_GCC_BIN must be set!")
#endif()

# specify the cross compiler
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER ${ARM_NONE_EABI_GCC_BIN}/arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER ${ARM_NONE_EABI_GCC_BIN}/arm-none-eabi-g++)
set(CMAKE_SIZE ${ARM_NONE_EABI_GCC_BIN}/arm-none-eabi-size)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F042C6Tx_FLASH.ld)
SET(COMMON_FLAGS "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft -ffunction-sections -fdata-sections -Og -g3 -fno-common -fmessage-length=0 -Wall -DSTM32F042x6")
SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-specs=nosys.specs -specs=nano.specs -Wl,-gc-sections -T ${LINKER_SCRIPT}")
