@echo off
setlocal
cd /d "%~dp0\..\.."
if not exist "manual-build-line-host-test" mkdir "manual-build-line-host-test"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 >nul
if not "%errorlevel%"=="0" exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /Itests\line_recovery\stubs /ICore\Inc tests\line_recovery\test_line_recovery.c Core\Src\line_tracking.c Core\Src\line_recovery.c Core\Src\buzzer_phrase_40077493715.c /Fomanual-build-line-host-test\ /Femanual-build-line-host-test\test_line_recovery.exe
if not "%errorlevel%"=="0" exit /b 1
manual-build-line-host-test\test_line_recovery.exe
if not "%errorlevel%"=="0" exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /DLINE_SEARCH_NOMINAL_YAW_MDEG_S=90000 /Itests\line_recovery\stubs /ICore\Inc tests\line_recovery\test_line_recovery.c Core\Src\line_tracking.c Core\Src\line_recovery.c Core\Src\buzzer_phrase_40077493715.c /Fomanual-build-line-host-test\ /Femanual-build-line-host-test\test_line_recovery_slow.exe
if not "%errorlevel%"=="0" exit /b 1
manual-build-line-host-test\test_line_recovery_slow.exe
if not "%errorlevel%"=="0" exit /b 1
cl /nologo /W4 /WX /utf-8 /std:c11 /Itests\line_recovery\stubs /ICore\Inc tests\line_recovery\test_line_turn_load.c Core\Src\drive_base.c Core\Src\line_turn_load.c Core\Src\line_tracking.c Core\Src\line_recovery.c Core\Src\buzzer_phrase_40077493715.c /Fomanual-build-line-host-test\ /Femanual-build-line-host-test\test_line_turn_load.exe
if not "%errorlevel%"=="0" exit /b 1
manual-build-line-host-test\test_line_turn_load.exe
exit /b %errorlevel%
