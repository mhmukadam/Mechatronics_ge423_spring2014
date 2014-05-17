@echo off
pushd ..\..\
setlocal

:process_arg
if "%1"=="" goto end_process_arg
set name=%1
set value=

:process_arg_value
if NOT "%value%"=="" set value=%value% %2
if "%value%"=="" set value=%2
shift
if "%2"=="!" goto set_arg
if "%2"=="" goto set_arg
goto process_arg_value

:set_arg
set %name%=%value%
shift
shift
goto process_arg
:end_process_arg

echo. > temp_postBuildStep_Debug.bat

echo hex6x.exe %PROJECT_ROOT%\..\out2bootbin.cmd -o %PROJECT_ROOT%\..\finalp.bin %PROJECT_ROOT%\debug\finalp.out >> temp_postBuildStep_Debug.bat

call temp_postBuildStep_Debug.bat
del temp_postBuildStep_Debug.bat

endlocal
popd
 
