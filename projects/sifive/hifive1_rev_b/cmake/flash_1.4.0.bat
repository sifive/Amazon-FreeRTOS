@echo off 
  set buildImageName=%3
  if "%buildImageName%" == "aws_tests.hex" (set nameProj=aws_tests) else (set nameProj=aws_demos)
  setlocal
  set vName=Hifive
  set n=0
  set ltrs=ABCDEFGHIJKLMNOPQRSTUVWXYZ
 :Loop
  call set ltr=%%ltrs:~%n%,1%%
  set /a n +=1
  vol %ltr%: 2>nul|find /i " %vname%">nul||if %n% lss 26 goto :loop
  if %n% equ 26 (set "ltr="
    echo No matching %vName% device.
  ) else (echo Loading firmware to %vName% device...)
  endlocal & set ltr=%ltr%
  
  cd %~dp0
  set flashFile=%cd%\%nameProj%\build\%nameProj%.hex
  xcopy %flashFile% %ltr%:\