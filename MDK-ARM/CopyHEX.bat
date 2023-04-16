:: copy -r .\MDK-ARM\BallGame\*.hex .

@echo off
:: 要复制的文件夹
set SOUECE=.
:: 复制到的目录
set DESTINATION=..
:: for后面的​​/d​​表示遍历当前路径的文件夹下的文件夹，如果不加，会获得dir目录下的所有文件名
:: for​​循环在cmd命令窗口中使用的时候，变量​​i​​用​​%i​​表示，但是在cmd文件中保存后运行时，cnd文件里的​​i​​要用​​%%i​​表示
:: 使用shell中的cp命令 cp -r ./*.hex  ..
for /d %%i in (%SOUECE%\*) do (
xcopy %%i\*.hex %DESTINATION%\ /s/y/i)
exit