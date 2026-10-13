@echo off
setlocal
set "APP=%~dp0PiStressTest.py"
if not exist "%APP%" goto missing

set "PYW="
for /f "delims=" %%I in ('where pythonw.exe 2^>nul') do if not defined PYW set "PYW=%%I"
if not defined PYW if exist "%LOCALAPPDATA%\Programs\Python314\pythonw.exe" set "PYW=%LOCALAPPDATA%\Programs\Python314\pythonw.exe"
if not defined PYW if exist "%LOCALAPPDATA%\Programs\Python313\pythonw.exe" set "PYW=%LOCALAPPDATA%\Programs\Python313\pythonw.exe"
if not defined PYW if exist "%LOCALAPPDATA%\Programs\Python312\pythonw.exe" set "PYW=%LOCALAPPDATA%\Programs\Python312\pythonw.exe"
if defined PYW goto run

where python.exe >nul 2>nul || goto nopython
echo 未找到 pythonw.exe，改用 python.exe 启动。
python "%APP%"
exit /b %errorlevel%

:run
start "" "%PYW%" "%APP%"
exit /b 0

:missing
echo [错误] 找不到 PiStressTest.py，请把它和本文件放在同一个文件夹里。
pause
exit /b 1

:nopython
echo [错误] 没有找到 Python，请先安装 Python 3.8 或更高版本：https://www.python.org/downloads/
pause
exit /b 1
