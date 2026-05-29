@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ============================================
echo   STM32H7 项目环境一键部署脚本
echo   Robot_Drive 项目自动配置工具
echo ============================================
echo.

:: 设置安装路径
set "ARM_TOOLCHAIN_DIR=C:\ARM_Toolchain"
set "ARM_GCC_VERSION=arm-gnu-toolchain-13.2.Rel1-mingw-w64-i686-arm-none-eabi"
set "ARM_GCC_PATH=%ARM_TOOLCHAIN_DIR%\%ARM_GCC_VERSION%\bin"
set "MSYS2_PATH=C:\msys64\ucrt64\bin"

:: 步骤1: 检查并创建ARM工具链目录
echo [步骤1/6] 检查ARM GNU Toolchain...
if exist "%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" (
    echo     [OK] ARM GNU Toolchain 已存在: %ARM_GCC_PATH%
) else (
    echo     [!] 未找到ARM GNU Toolchain，开始下载...
    
    if not exist "%ARM_TOOLCHAIN_DIR%" mkdir "%ARM_TOOLCHAIN_DIR%"
    
    echo     正在下载 ARM GNU Toolchain (约350MB)...
    curl.exe -L -o "%ARM_TOOLCHAIN_DIR%\arm-gnu-toolchain.zip" "https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi.zip"
    
    echo     正在解压 ARM GNU Toolchain...
    powershell -Command "Expand-Archive -Path '%ARM_TOOLCHAIN_DIR%\arm-gnu-toolchain.zip' -DestinationPath '%ARM_TOOLCHAIN_DIR%' -Force"
    
    :: 使用tar重新解压以确保完整
    if not exist "%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" (
        echo     使用tar重新解压...
        tar -xf "%ARM_TOOLCHAIN_DIR%\arm-gnu-toolchain.zip" -C "%ARM_TOOLCHAIN_DIR%"
    )
    
    if exist "%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" (
        echo     [OK] ARM GNU Toolchain 安装成功!
    ) else (
        echo     [X] ARM GNU Toolchain 安装失败!
        goto :error
    )
)

:: 步骤2: 检查MSYS2
echo.
echo [步骤2/6] 检查MSYS2环境...
if exist "%MSYS2_PATH%\mingw32-make.exe" (
    echo     [OK] MSYS2 MinGW-w64 已存在
) else (
    echo     [!] 未找到MSYS2，请手动安装MSYS2: https://www.msys2.org/
    echo     或使用: winget install MSYS2.MSYS2
    goto :error
)

:: 步骤3: 检查GCC (用于IntelliSense)
if exist "%MSYS2_PATH%\gcc.exe" (
    echo     [OK] UCRT64 GCC 已存在
) else (
    echo     [!] UCRT64 GCC 不存在，正在安装...
    C:\msys64\usr\bin\bash.exe -lc "pacman -Sy --noconfirm mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-make"
    if exist "%MSYS2_PATH%\gcc.exe" (
        echo     [OK] UCRT64 GCC 安装成功!
    ) else (
        echo     [X] UCRT64 GCC 安装失败!
    )
)

:: 步骤4: 检查OpenOCD
echo.
echo [步骤3/6] 检查OpenOCD调试工具...
if exist "%MSYS2_PATH%\openocd.exe" (
    echo     [OK] OpenOCD 已存在
) else (
    echo     [!] OpenOCD不存在，正在安装...
    C:\msys64\usr\bin\bash.exe -lc "pacman -S --noconfirm mingw-w64-ucrt-x86_64-openocd"
    if exist "%MSYS2_PATH%\openocd.exe" (
        echo     [OK] OpenOCD 安装成功!
    ) else (
        echo     [X] OpenOCD 安装失败，请手动安装
    )
)

:: 步骤5: 配置PATH环境变量
echo.
echo [步骤4/6] 配置系统PATH环境变量...
set "NEW_PATHS=%ARM_GCC_PATH%;%MSYS2_PATH%"

:: 检查PATH是否已包含
set "PATH_UPDATED=0"
echo %PATH% | findstr /C:"%ARM_GCC_PATH%" >nul
if errorlevel 1 (
    set "PATH_UPDATED=1"
)

if "%PATH_UPDATED%"=="1" (
    echo     正在添加PATH环境变量...
    powershell -Command "[Environment]::SetEnvironmentVariable('Path', [Environment]::GetEnvironmentVariable('Path', 'User') + ';%ARM_GCC_PATH%;%MSYS2_PATH%', 'User')"
    echo     [OK] PATH环境变量已更新
    echo     [!] 请重启终端或VSCode以使PATH生效
) else (
    echo     [OK] PATH环境变量已配置
)

:: 步骤6: 验证安装
echo.
echo [步骤5/6] 验证工具安装...
set "ALL_OK=1"

"%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" --version >nul 2>&1
if errorlevel 1 (
    echo     [X] arm-none-eabi-gcc 验证失败
    set "ALL_OK=0"
) else (
    for /f "tokens=*" %%i in ('"%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" --version 2^>nul ^| findstr "arm-none-eabi-gcc"') do (
        echo     [OK] %%i
    )
)

"%MSYS2_PATH%\mingw32-make.exe" --version >nul 2>&1
if errorlevel 1 (
    echo     [X] mingw32-make 验证失败
    set "ALL_OK=0"
) else (
    for /f "tokens=*" %%i in ('"%MSYS2_PATH%\mingw32-make.exe" --version 2^>nul ^| findstr "GNU Make"') do (
        echo     [OK] %%i
    )
)

"%MSYS2_PATH%\openocd.exe" --version >nul 2>&1
if errorlevel 1 (
    echo     [X] openocd 验证失败
    set "ALL_OK=0"
) else (
    for /f "tokens=*" %%i in ('"%MSYS2_PATH%\openocd.exe" --version 2^>nul ^| findstr "Open On-Chip"') do (
        echo     [OK] %%i
    )
)

:: 步骤7: 编译项目
echo.
echo [步骤6/6] 编译项目...
if exist build (
    echo     清理旧的编译文件...
    rmdir /s /q build
)

echo     正在编译 (使用8线程)...
"%MSYS2_PATH%\mingw32-make.exe" -j8 GCC_PATH=%ARM_GCC_PATH%

if exist "build\Omni_damiao.elf" (
    echo.
    echo ============================================
    echo   [SUCCESS] 编译成功!
    echo ============================================
    echo.
    echo   生成文件:
    for %%f in (build\Omni_damiao.*) do (
        echo     - %%f
    )
) else (
    echo.
    echo ============================================
    echo   [FAILED] 编译失败!
    echo ============================================
    goto :error
)

echo.
echo ============================================
echo   环境配置完成!
echo ============================================
echo.
echo 工具路径:
echo   - ARM GCC: %ARM_GCC_PATH%
echo   - Make:    %MSYS2_PATH%\mingw32-make.exe
echo   - OpenOCD: %MSYS2_PATH%\openocd.exe
echo.
echo 使用方法:
echo   编译项目: mingw32-make GCC_PATH=%ARM_GCC_PATH%
echo   清理项目: mingw32-make clean
echo   调试烧录: 使用VSCode Cortex-Debug插件
echo.

if "%ALL_OK%"=="1" (
    echo 状态: 所有工具已正确安装并验证通过
) else (
    echo 状态: 部分工具验证失败，请检查安装
)

goto :end

:error
echo.
echo ============================================
echo   部署过程中出现错误!
echo ============================================
echo 请检查网络连接或手动安装缺失的组件
exit /b 1

:end
echo.
pause