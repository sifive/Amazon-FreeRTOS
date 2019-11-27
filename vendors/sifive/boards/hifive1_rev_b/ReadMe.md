# Build
Ensure that you have risc-v toolchain installed. Later it will be referred as <path to toolchain>. It will typically be something like `/home/user/sdk/riscv64-unknown-elf-gcc-8.2.0-2019.02.0-x86_64-linux-centos6/bin/`. It seems that the trainling slash is important.

Also you will need CMake of at least version 3.13.  

To build the project, run
    cmake -DAFR_TOOLCHAIN_PATH=<path to toolchain> -DVENDOR=sifive -DBOARD=hifive1_rev_b -DCOMPILER=risc-v-gcc -S . -B build  -DAFR_MODULE_defender=OFF -DAFR_MODULE_shadow=OFF -DAFR_MODULE_mqtt=OFF -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON 
If you want to create an Freedom Studio project, run
    cmake -DAFR_TOOLCHAIN_PATH=<path to toolchain> -DVENDOR=sifive -DBOARD=hifive1_rev_b -DCOMPILER=risc-v-gcc -S . -B build  -DAFR_MODULE_defender=OFF -DAFR_MODULE_shadow=OFF -DAFR_MODULE_mqtt=OFF -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON -G"Eclipse CDT4 - Unix Makefiles"

After that 
    cd build
    make

The built binary file will be located in `build/vendors/sifive/boards/hifive1_rev_b/aws_demos.out`
