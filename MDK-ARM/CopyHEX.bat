:: copy -r .\MDK-ARM\BallGame\*.hex .

@echo off
:: 要复制的文件夹
set SOUECE=.
:: 复制到的目录
set DESTINATION=..
for /d %%i in (%SOUECE%\*) do (
xcopy %%i\*.hex %DESTINATION%\ /s/y/i)
pause