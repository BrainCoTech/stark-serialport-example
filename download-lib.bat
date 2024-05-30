@echo off
setlocal
cd %~dp0
set SCRIPT_DIR=%cd%
set LIB_VERSION=v0.0.8
set URL=app.brainco.cn/universal/stark-serialport-prebuild/%LIB_VERSION%
goto :main

:: echo yellow
:echo_y
    echo [93m%*[0m
goto :eof

:: echo red
:echo_r
    echo [91m%*[0m
goto :eof

:: check choco
:check_choco
    :: check choco
    where choco >nul 2>&1 || (
        call :echo_r [libstark][win] command 'choco' is not found
        call :echo_y [libstark][win] Please install at https://chocolatey.org/
        exit /b 1
    )

    :: check admin
    net session >nul 2>&1 || (
        call :echo_r [libstark][win] Require admin premission for 'choco install'
        call :echo_y [libstark][win] Please restart the terminal with admin premission
        exit /b 1
    )
goto :eof

:main
    :: 1. check VERSION file
    findstr %LIB_VERSION% VERSION >nul 2>&1 && (
        call :echo_y [libstark][win] libstark "%LIB_VERSION%" has already been installed
        call type VERSION
        exit /b 0
    )

    :: 2. check 7z command
    where 7z >nul 2>&1 || (
        call :echo_r [libstark][win] command '7z' is not found
        call :echo_y [libstark][win] choco install 7zip -y
        call :check_choco           || exit /b 1
        call choco install 7zip -y  || exit /b 1
    )

    :: 3. check wget command
    where wget >nul 2>&1 || (
        call :echo_r [libstark][win] command 'wget' is not found
        call :echo_y [libstark][win] choco install wget -y
        call :check_choco           || exit /b 1
        call choco install wget -y  || exit /b 1
    )

    call :echo_y [libstark][win] download libstark "%LIB_VERSION%" ...
    rmdir /s /q include win android
    del VERSION

    :: 1. download lib
    call :echo_y [libstark][win] download header
    call wget %URL%/win.zip
    call 7z x -y win.zip
    del win.zip

    :: 2. create VERSION file
    echo libstark Version: %LIB_VERSION% > VERSION
    for /f "tokens=*" %%t in ('date /t') do (
        echo Update Time: %%t%time%
    ) >> VERSION

    call :echo_y [libstark][win] libstark "%LIB_VERSION%" is downloaded
goto :eof
