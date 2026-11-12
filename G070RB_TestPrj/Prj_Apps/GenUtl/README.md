# GenUtl
General utilities for embedded c projects.

## Requirements
- GCC Compiler + arm-none-eabi packages (https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
- CMake
- STM32Cube extension
- West
- Extension: Cortex-Debug, STM32

## Usage
1. Generate from CubeMX a device-related CMake project 
2. Open it with VScode and set up as stm32 project (follow extension directives)
3. Fork from module template and clone straight to the local STM32 project directory
4. Add to main CMakeLists.txt 
    ```C
        add_subdirectory(module_name) 

        target_link_libraries(${CMAKE_PROJECT_NAME}
            [...]
            module_name
        )
    ```

## Notes
- Remember to check .gitignore to avoid unwanted code on main repos
- Remember to check launch.json for correct device target
- Rememeber to check your arm-none-eabi toolchain is set correctly

## TODOs
- move all of ths to containerized working space to set it up exactly as this
- automate stm32 configuration and project creation with tasks

