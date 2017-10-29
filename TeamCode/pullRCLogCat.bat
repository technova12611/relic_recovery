SET PATH=%PATH%;%LOCALAPPDATA%\Android\Sdk\platform-tools

SET myDate=%date:~10,4%%date:~4,2%%date:~7,2%

SET myLogDir=.\logcat\%myDate%
if not exist %myLogDir% mkdir %myLogDir%

adb pull /storage/emulated/0/robotControllerLog.txt %myLogDir%\robotControllerLog.txt
adb pull /storage/emulated/0/robotControllerLog.txt.1 %myLogDir%\robotControllerLog.txt.1
adb pull /storage/emulated/0/robotControllerLog.txt.2 %myLogDir%\robotControllerLog.txt.2 
adb pull /storage/emulated/0/robotControllerLog.txt.3 %myLogDir%\robotControllerLog.txt.3 
adb pull /storage/emulated/0/robotControllerLog.txt.4 %myLogDir%\robotControllerLog.txt.4 
adb pull /storage/emulated/0/robotControllerLog.txt.5 %myLogDir%\robotControllerLog.txt.5 
