@echo off
set PATH=C:\Program Files\Java\jre\bin;G:\VStudio\msys2\usr\bin;

rem This batch files reads rusefi_config.txt and produses firmware persistent configuration headers
rem the storage section of rusefi.ini is updated as well

cd ../../..
cp tunerstudio/rusefi.input config/boards/kinetis/config/tunerstudio/rusefi.input 
java -jar ../java_tools/ConfigDefinition.jar integration config/boards/kinetis/config/tunerstudio controllers/algo ../java_console config/boards/kinetis/config/rusefi_config_kinetis.txt


rem This would automatically copy latest file to 'dev' TS project
set ts_path="%HOMEDRIVE%%HOMEPATH%\Documents\TunerStudioProjects\RUSefi_test\projectCfg"
echo %ts_path%
cp tunerstudio/rusefi.ini %ts_path%\mainController.ini