@echo off
setlocal
cd /d "%~dp0\..\.."
if not exist "manual-build-simple-line-tests" mkdir "manual-build-simple-line-tests"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
if not "%errorlevel%"=="0" exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /ISimpleLine\Inc tests\simple_line\test_controller.c SimpleLine\Src\line_follow.c SimpleLine\Src\operator_input.c /Fomanual-build-simple-line-tests\ /Femanual-build-simple-line-tests\test_controller.exe
if not "%errorlevel%"=="0" exit /b 1
manual-build-simple-line-tests\test_controller.exe
if not "%errorlevel%"=="0" exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /Itests\simple_line\stubs /ISimpleLine\Inc /ICore\Inc tests\simple_line\test_board.c Core\Src\motorPWM.c SimpleLine\Src\board.c SimpleLine\Src\line_follow.c SimpleLine\Src\operator_input.c /Fomanual-build-simple-line-tests\ /Femanual-build-simple-line-tests\test_board.exe
if not "%errorlevel%"=="0" exit /b 1
manual-build-simple-line-tests\test_board.exe
exit /b %errorlevel%
