@echo off
setlocal
cd /d "%~dp0\..\.."
if not exist "manual-build-sep3-host-test" mkdir "manual-build-sep3-host-test"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
if errorlevel 1 exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /Itests\line_recovery\stubs /ICore\Inc tests\line_recovery\test_sep3_spin.c Core\Src\line_tracking.c /Fomanual-build-sep3-host-test\ /Femanual-build-sep3-host-test\test_sep3_spin.exe
if errorlevel 1 exit /b 1
manual-build-sep3-host-test\test_sep3_spin.exe
exit /b %errorlevel%
