@echo off
REM ================================================================
REM  飞控固件单元测试 — 编译 & 运行
REM  要求：GCC (MinGW-w64) 已在 PATH 中
REM ================================================================
setlocal enabledelayedexpansion

set "ROOT=%~dp0"
set "FW_SRC=%ROOT%..\..\ANO_LX_FC_T265代替光流\DriversBsp"
set "MOCK=%ROOT%mock"
set "BUILD=%ROOT%build"

if not exist "%BUILD%" mkdir "%BUILD%"

echo === 飞控固件单元测试 ===
echo.

REM ---- 编译 Ano_Math 测试 ----
echo [编译] test_ano_math ...
pushd "%ROOT%"
gcc -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -std=c99 ^
    -I ".\mock" ^
    -I "..\..\ANO_LX_FC_T265代替光流\DriversBsp" ^
    "..\..\ANO_LX_FC_T265代替光流\DriversBsp\Ano_Math.c" ^
    ".\test_ano_math.c" ^
    -o ".\build\test_ano_math.exe" ^
    -lm
set "COMPILE_RESULT=%ERRORLEVEL%"
popd

if %COMPILE_RESULT% neq 0 (
    echo [错误] 编译失败！
    exit /b 1
)
echo [编译] 成功
echo.

REM ---- 运行测试 ----
echo [运行] test_ano_math ...
echo ----------------------------------------
pushd "%ROOT%"
".\build\test_ano_math.exe"
set "RESULT=%ERRORLEVEL%"
popd
echo ----------------------------------------
echo.

if "%RESULT%"=="0" (
    echo [结果] 全部通过
) else (
    echo [结果] 有测试失败 (exit code: %RESULT%)
)

exit /b %RESULT%
