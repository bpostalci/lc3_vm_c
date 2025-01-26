cmake_minimum_required(VERSION 3.21)
project(lc3_vm_c LANGUAGES C)

# Set the C compiler and standard
set(CMAKE_C_COMPILER gcc)
set(CMAKE_C_STANDARD 17)

# Specify the output directory for the build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Add the executable and include the generated lc3os.c
add_executable(lc3_vm_c 
    main.c 
)

# Add compile options for stricter code checks
target_compile_options(lc3_vm_c PRIVATE -Wall -Wextra -Wpedantic)
