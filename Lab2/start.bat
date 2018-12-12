rem Sets the necessary environment variables for successful execution
rem of SGCT and OSG. Call this script from your Command Window before
rem running SGCT+OSG applications.
rem


set SCRIPT_PATH=%~dp0
call %SCRIPT_PATH%/setenv.bat

set CONFIG=D:/VRSystem/Desktop-folder/sgct_config/vr_lab_workbench_vive.xml

start solution.exe -config %CONFIG% -local 0
start solution.exe -config %CONFIG% -local 1 --slave
