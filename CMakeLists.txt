cmake_minimum_required(VERSION 3.21)
project(lc3_vm_c LANGUAGES C)

# Set the C compiler and standard
set(CMAKE_C_COMPILER gcc)
set(CMAKE_C_STANDARD 17)

# Specify the output directory for the build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add optional tracing support
option(ENABLE_TRACING "Enable trace-level logging of the VM state" OFF)

# Generate lc3os.c from lc3os.obj
add_custom_command(
    COMMAND xxd -i lc3os.obj > ${CMAKE_CURRENT_BINARY_DIR}/lc3os.c
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/lc3os.obj
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lc3os.c
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add the executable and include the generated lc3os.c
add_executable(lc3_vm_c 
    main.c 
    ${CMAKE_CURRENT_BINARY_DIR}/lc3os.c
)

# Add compile options for stricter code checks
target_compile_options(lc3_vm_c PRIVATE -Wall -Wextra -Wpedantic)

# Add tracing support if enabled
if (ENABLE_TRACING)
    target_compile_definitions(lc3_vm_c PRIVATE TRACE)
endif()
