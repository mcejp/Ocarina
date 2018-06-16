cmake_minimum_required(VERSION 3.6)
project(Ocarina C ASM)

set(BOARD "" CACHE STRING "Target board version")

if (BOARD STREQUAL "")
    message(FATAL_ERROR "BOARD must be set according to target board version (2001, 3001...)!")
endif()

add_subdirectory(dependencies/libs-tx)

file(GLOB_RECURSE USER_SOURCES "Src/*.c")
file(GLOB_RECURSE HAL_SOURCES "Drivers/STM32F0xx_HAL_Driver/Src/*.c"
        "Middlewares/ST/STM32_USB_Device_Library/*.c")

add_library(CMSIS
        Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
        Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc/startup_stm32f042x6.s)

include_directories(Inc)
include_directories(Drivers/STM32F0xx_HAL_Driver/Inc)
include_directories(Drivers/CMSIS/Include)
include_directories(Drivers/CMSIS/Device/ST/STM32F0xx/Include)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Core/Inc)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${HAL_SOURCES} ${LINKER_SCRIPT})

target_compile_definitions(${PROJECT_NAME}.elf PUBLIC BOARD=${BOARD})

target_include_directories(${PROJECT_NAME}.elf PRIVATE Src)

target_link_libraries(${PROJECT_NAME}.elf CMSIS)
target_link_libraries(${PROJECT_NAME}.elf tx)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map")

set(HEX_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        )

add_custom_target(flash
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND dfu-util -a 0 -D ${BIN_FILE} --dfuse-address 0x08000000
        COMMENT "Uploading ${BIN_FILE}")
