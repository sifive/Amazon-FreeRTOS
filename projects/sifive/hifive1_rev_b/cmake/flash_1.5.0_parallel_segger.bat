@echo off 
set buildImageName=%3
if "%buildImageName%" == "aws_tests.hex" (set nameProj=aws_tests) else (set nameProj=aws_demos)
cd %~dp0
set flashFile=%cd%\%nameProj%\build\%nameProj%.hex
rm JLinkCommandFile.jlink
echo h >> JLinkCommandFile.jlink
echo r >> JLinkCommandFile.jlink
echo loadfile %flashFile% >> JLinkCommandFile.jlink
echo h >> JLinkCommandFile.jlink
echo r >> JLinkCommandFile.jlink
echo g >> JLinkCommandFile.jlink
echo exit >> JLinkCommandFile.jlink
JLink -device FE310 -if JTAG -jtagconf -1,-1 -speed 4000 -autoconnect 1 -CommanderScript JLinkCommandFile.jlink
rm JLinkCommandFile.jlink
