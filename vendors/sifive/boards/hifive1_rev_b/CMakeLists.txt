cmake_minimum_required(VERSION 3.13)
# These Amazon FreeRTOS related global variables are available to use.
# AFR_ROOT_DIR                  Amazon FreeRTOS source root.
# AFR_KERNEL_DIR                FreeRTOS kernel root.
# AFR_MODULES_DIR               Amazon FreeRTOS modules root.
# AFR_MODULES_C_SDK_DIR         C-SDK libraries root.
# AFR_MODULES_FREERTOS_PLUS_DIR FreeRTOS-Plus libraries root.
# AFR_MODULES_ABSTRACTIONS_DIR  Abstractions layers root.
# AFR_DEMOS_DIR                 Amazon FreeRTOS demos root.
# AFR_TESTS_DIR                 Amazon FreeRTOS common tests and framework root.
# AFR_VENDORS_DIR               vendors content root.
# AFR_3RDPARTY_DIR              3rdparty libraries root.

# AFR_VENDOR_NAME           Folder name for vendor.
# AFR_BOARD_NAME            Folder name for this board.

# AFR_TOOLCHAIN             Compiler chosen by the user. Should be one of
#                           the file names under ${AFR_ROOT_DIR}/tools/cmake/toolchains
# AFR_IS_TESTING            1 if testing enabled, otherwise, 0.

# You may also use these 2 functions we defined to glob files when needed. However, we recommend
# to specify your source files explicitly to avoid unexpected behavior unless you're 100% sure.
# CMake reference link: https://cmake.org/cmake/help/latest/command/file.html#filesystem
# afr_glob_files(<out_var> [RECURSE] <DIRECTORY> <directory> [<GLOBS> <glob-expressions>...])
# afr_glob_src(<out_var> [RECURSE] <DIRECTORY> <directory> [<EXTENSIONS> <file-extensions>...])

# If you don't specify GLOBS or EXTENSIONS parameters,
# afr_glob_files: glob all files including hidden files in the specified directory.
# afr_glob_src:   glob all files ending with either .c, .h, .s or .asm

# Use RECURSE if you want to recursively search all subdirectories.

# Example usage,
# afr_glob_src(board_code DIRECTORY "${board_dir}/application_code/${vendor}_code")
# afr_glob_src(driver_code RECURSE DIRECTORY "${driver_path}")
# afr_glob_src(headers DIRECTORY "${some_path}" EXTENSIONS h)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS Console metadata
# -------------------------------------------------------------------------------------------------
# Provide metadata for listing on Amazon FreeRTOS console.
afr_set_board_metadata(NAME "HiFive1 Rev B")
afr_set_board_metadata(DISPLAY_NAME "HiFive1 Rev B")
afr_set_board_metadata(DESCRIPTION "RISC-V development board producted by SiFive featuring the Freedom E310")
afr_set_board_metadata(VENDOR_NAME "SiFive")
afr_set_board_metadata(FAMILY_NAME "HiFive1")
afr_set_board_metadata(DATA_RAM_MEMORY "64KB")
afr_set_board_metadata(PROGRAM_MEMORY "4MB")
afr_set_board_metadata(CODE_SIGNER "null")
afr_set_board_metadata(SUPPORTED_IDE "CMakeBuildSystem")
afr_set_board_metadata(IDE_CMakeBuildSystem_NAME "CMakeBuildSystem")
afr_set_board_metadata(IDE_CMakeBuildSystem_COMPILERS "GCC")

set (bsp_board_name sifive-hifive1-revb)
#enable_language(C CXX ASM)
# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------
# If you support multiple compilers, you can use AFR_TOOLCHAIN to conditionally define the compiler
# settings. This variable will be set to the file name of CMAKE_TOOLCHAIN_FILE. It might also be a
# good idea to put your compiler settings to different files and just include them here, e.g.,
# include(compilers/${AFR_TOOLCHAIN}.cmake)


set(vendor ${AFR_VENDOR_NAME})
set(board ${AFR_BOARD_NAME})
set(portable_dir "portable/${vendor}/${board}")

set(board_demos_dir "${AFR_VENDORS_DIR}/${vendor}/boards/${board}/aws_demos")
set(board_tests_dir "${AFR_VENDORS_DIR}/${vendor}/boards/${board}/aws_tests")
if(AFR_IS_TESTING)
 set(board_dir "${board_tests_dir}")
 set(aws_credentials_include "${AFR_TESTS_DIR}/include")
else()
 set(board_dir "${board_demos_dir}")
 set(aws_credentials_include "${AFR_DEMOS_DIR}/include")
endif()
set(driver_dir "${board_dir}/application_code/sifive_code")

set(linker_script ${driver_dir}/bsp/freertos.sifive-hifive1-revb.lds)
set(bsp_metal_header ${driver_dir}/bsp/machine.h)
set(bsp_metal_inline_header ${driver_dir}/bsp/machine-inline.h)
set(afr_ports_dir "${AFR_VENDORS_DIR}/${vendor}/boards/${board}/ports")

afr_mcu_port(compiler)
set(
	defined_symbols
	__riscv_xlen=32
    USE_OFFLOAD_SSL
)
# Compile definitions/macros
target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE
        AmazonFreeRTOS
        PACKAGE_NAME="freedom-metal"
        PACKAGE_TARNAME="freedom-metal"
        PACKAGE_VERSION="v0.1.2"
        PACKAGE_STRING="freedom-metal v0.1.2"
        PACKAGE_BUGREPORT="palmer@sifive.com"
        PACKAGE_URL=""
        PACKAGE="freedom-metal"
        VERSION="v0.1.2"
        portasmHANDLE_INTERRUPT=handle_trap
        __METAL_MACHINE_HEADER="${bsp_metal_header}"
        MACHINE_INLINE="${bsp_metal_inline_header}"
		$<$<NOT:$<COMPILE_LANGUAGE:ASM>>:${defined_symbols}>
)

# Compiler flags
target_compile_options(
     AFR::compiler::mcu_port
     INTERFACE
	 -c 
	 -fmessage-length=0
	 -specs=nano.specs
     -march=rv32imac
     -mabi=ilp32
     -mcmodel=medlow
     -ffunction-sections
     -fdata-sections
     -falign-functions=4
	 -falign-jumps=4
	 -falign-loops=4
     -g
     -O2
)

# Global include directories
target_include_directories(
     AFR::compiler::mcu_port
     INTERFACE
         ${driver_dir}/bsp
         ${driver_dir}/bsp/install/include
         ${driver_dir}/bsp/install/include/metal
         ${driver_dir}/freedom-metal
         ${driver_dir}/hal
)

# Linker flags
target_link_options(
     AFR::compiler::mcu_port
     INTERFACE
     -march=rv32imac
     -mabi=ilp32
     -mcmodel=medlow
     -Wl,--gc-sections
     -Wl,-Map=output.map
     -nostartfiles
     -nostdlib
     -T${linker_script}
     -Wl,--start-group
     -lc
     -lgcc
     -lgloss
     -Wl,--end-group
)

# Library search path for linker
# target_link_directories(
#     AFR::compiler::mcu_port
#     INTERFACE
#         "<lib_path_0>"
#         "<lib_path_1>"
# )

# Libraries to link
target_link_libraries(
    AFR::compiler::mcu_port
    INTERFACE
       "c"
       "gcc"
)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS portable layers
# -------------------------------------------------------------------------------------------------
# Define portable layer targets with afr_mcu_port(<module_name>). We will create an CMake
# INTERFACE IMPORTED target called AFR::${module_name}::mcu_port for you. You can use it with
# standard CMake functions like target_*. To better organize your files, you can define your own
# targets and use target_link_libraries(AFR::${module_name}::mcu_port INTERFACE <your_targets>)
# to provide the public interface you want expose.

# Kernel
afr_mcu_port(kernel)

afr_glob_src(metal_code DIRECTORY "${driver_dir}/freedom-metal/src" EXTENSIONS c)
afr_glob_src(gloss_code RECURSE DIRECTORY "${driver_dir}/freedom-metal/gloss" EXTENSIONS c)
afr_glob_src(hal_code RECURSE DIRECTORY "${driver_dir}/hal" EXTENSIONS c)

set_source_files_properties(${driver_dir}/freedom-metal/src/entry.S PROPERTIES LANGUAGE C)

target_sources(
    AFR::kernel::mcu_port
    INTERFACE
        ${metal_code}
        ${gloss_code}
        ${hal_code}
        ${driver_dir}/freedom-metal/src/entry.S
        ${driver_dir}/freedom-metal/gloss/crt0.S
        ${AFR_KERNEL_DIR}/portable/GCC/RISC-V/port.c
        ${AFR_KERNEL_DIR}/portable/GCC/RISC-V/portASM.S
        ${AFR_KERNEL_DIR}/portable/MemMang/heap_4.c
        ${driver_dir}/freedom-metal/src/drivers/fixed-clock.c
        ${driver_dir}/freedom-metal/src/drivers/riscv_cpu.c
        ${driver_dir}/freedom-metal/src/drivers/riscv_clint0.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_uart0.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_gpio0.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_spi0.c
        ${driver_dir}/freedom-metal/src/drivers/riscv_plic0.c
        ${driver_dir}/freedom-metal/src/drivers/inline.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_fe310-g000_prci.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_fe310-g000_hfrosc.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_fe310-g000_hfxosc.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_fe310-g000_prci.c
        ${driver_dir}/freedom-metal/src/drivers/sifive_fe310-g000_pll.c
)
target_include_directories(
    AFR::kernel::mcu_port
    INTERFACE
        ${driver_dir}
        ${board_dir}/config_files
        ${driver_dir}/freedom-metal
		${driver_dir}/bsp/install/include/metal
        ${metal_headers}
        ${AFR_KERNEL_DIR}/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions
        ${AFR_KERNEL_DIR}/portable/GCC/RISC-V
        ${aws_credentials_include}
        "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/freertos_plus_tcp/include"
        "${AFR_MODULES_FREERTOS_PLUS_DIR}/standard/freertos_plus_tcp/source/portable/Compiler/GCC"
)


# ESP-AT-Lib library integration
set (esp_at_lib_path "${AFR_VENDORS_DIR}/${AFR_VENDOR_NAME}/3dparty/ESP_AT_Lib")
file(GLOB
    esp_at_lib_src_esp
    ${esp_at_lib_path}/src/esp/*.c
)
set (esp_at_lib_src
     ${esp_at_lib_path}/src/api/esp_netconn.c
     ${esp_at_lib_src_esp}
     ${esp_at_lib_path}/src/system/esp_ll_hifive1_revb.c
     ${esp_at_lib_path}/src/system/esp_sys_freertos_os.c
     ${esp_at_lib_path}/src/system/circular_buffer.c
)

add_library(esp-at-lib STATIC ${esp_at_lib_src})

target_link_libraries(
    esp-at-lib
    PRIVATE
        AFR::compiler::mcu_port
        AFR::kernel

# Or use your own target if you already have it.
#   PRIVATE ${compiler_settings_target}
)

target_include_directories(
    esp-at-lib
    PUBLIC ${esp_at_lib_path}/src/include
    PUBLIC ${esp_at_lib_path}/src/include/esp
    PUBLIC ${esp_at_lib_path}/src/include/system
)

#target_link_libraries(
#    AFR::kernel::mcu_port
#    INTERFACE ${other_targets}
#)

# If you defined the driver and freertos portable target separately, you can use afr_mcu_port with
# DEPENDS keyword, e.g.,
# afr_mcu_port(kernel DEPENDS my_board_driver freertos_port)

# POSIX
# afr_mcu_port(posix)
# target_include_directories(
#     AFR::posix::mcu_port
#     INTERFACE "${AFR_MODULES_DIR}/FreeRTOS-Plus-POSIX/include/${portable_dir}"
# )

# WiFi
afr_mcu_port(wifi)
target_sources(
    AFR::wifi::mcu_port
    INTERFACE "${afr_ports_dir}/wifi/iot_wifi.c"
)

target_link_libraries(
    AFR::wifi::mcu_port
    INTERFACE esp-at-lib
    )


# PKCS11
afr_mcu_port(pkcs11_implementation)
target_sources(
    AFR::pkcs11_implementation::mcu_port
    INTERFACE "${afr_ports_dir}/pkcs11/iot_pkcs11_pal.c"
)
# Link to AFR::pkcs11_mbedtls if you want to use default implementation based on mbedtls.
target_link_libraries(
    AFR::pkcs11_implementation::mcu_port
    INTERFACE
        AFR::pkcs11_mbedtls
        AFR::wifi
)


# Secure sockets
afr_mcu_port(secure_sockets)
target_sources(
     AFR::secure_sockets::mcu_port
     INTERFACE "${afr_ports_dir}/secure_sockets/iot_secure_sockets.c"
)

target_link_libraries(
    AFR::secure_sockets::mcu_port
    INTERFACE AFR::pkcs11
    INTERFACE AFR::wifi
    INTERFACE esp-at-lib
    INTERFACE AFR::tls
)

# OTA
# afr_mcu_port(ota)
# target_sources(
#     AFR::ota::mcu_port
#     INTERFACE "${AFR_MODULES_DIR}/ota/${portable_dir}/aws_ota_pal.c"
# )

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS demos and tests
# -------------------------------------------------------------------------------------------------
# We require you to define at least demos and tests executable targets. Available demos and tests
# will be automatically enabled by us. You need to provide other project settings such as linker
# scripts and post build commands.

# ==================== Example ====================
set(CMAKE_EXECUTABLE_SUFFIX ".out")

if(AFR_IS_TESTING)
 set(exe_target aws_tests)
else()
 set(exe_target aws_demos)
endif()

add_executable(
    ${exe_target}
    "${board_dir}/application_code/main.c"
    ${gloss_code}
    )

target_link_libraries(
    ${exe_target}
    PUBLIC
         AFR::kernel
         AFR::utils
         AFR::secure_sockets
         AFR::wifi
         AFR::tls
         AFR::pkcs11
         esp-at-lib
         AFR::mqtt
)

get_filename_component(COMPILER_PATH ${CMAKE_C_COMPILER} DIRECTORY)
message("Compiler path is ${COMPILER_PATH}")


add_custom_command(
     TARGET ${exe_target} POST_BUILD
     COMMAND "${COMPILER_PATH}/riscv64-unknown-elf-objcopy" "-O" "ihex" "$<TARGET_FILE:${exe_target}>" "$<TARGET_FILE_DIR:${exe_target}>/${exe_target}.hex"
)

add_custom_command(
     TARGET ${exe_target} POST_BUILD
     COMMAND "${COMPILER_PATH}/riscv64-unknown-elf-size" "$<TARGET_FILE:${exe_target}>"
)
