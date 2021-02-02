@echo off

:: go to current directory
pushd %~dp0

:: clean build directory
rd /s /q build

:: create build files
cmake -B build

:: build
cmake --build build --config Release

:: get current sha
for /f %%i in ('git rev-parse --short HEAD') do set GIT_SHA=%%i

:: copy result
copy build\Release\fdr2csv.exe fdr2csv_%GIT_SHA%.exe

:: restore directory
popd
