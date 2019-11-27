@echo off
  cd %~dp0
  cd ..\..\..\..\
  set enableTests=%1
  if %enableTests% equ 1 (set nameProj=aws_tests) else (set nameProj=aws_demos)
  set buildDir=%cd%\projects\sifive\hifive1_rev_b\cmake\%nameProj%\build
  rmdir /s /q "%buildDir%"
  cmake -DVENDOR=sifive -DBOARD=hifive1_rev_b -DCOMPILER=risc-v-gcc -DAFR_ENABLE_TESTS=%enableTests% -S . -B %buildDir% -G "Unix Makefiles"
  cd %buildDir%
  make -j4