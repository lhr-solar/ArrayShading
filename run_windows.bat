@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "STL_PATH=%~1"
if not defined STL_PATH set "STL_PATH=%USERPROFILE%\Downloads\_24-000.stl"

if not exist "%STL_PATH%" (
  echo ERROR: STL file not found:
  echo   %STL_PATH%
  echo.
  echo Put _24-000.stl in your Downloads folder, or run:
  echo   .\run_windows.bat "C:\full\path\car.stl"
  exit /b 2
)

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: Visual Studio Build Tools 2022 is not installed.
  echo Install "Desktop development with C++" in Visual Studio Installer.
  exit /b 3
)

set "VS_INSTALL="
for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"

if not defined VS_INSTALL (
  echo ERROR: The Visual Studio C++ workload is not installed.
  echo Open Visual Studio Installer, choose Modify, then install:
  echo   Desktop development with C++
  echo Include MSVC, the Windows SDK, and CMake tools for Windows.
  exit /b 4
)

call "%VS_INSTALL%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
if errorlevel 1 (
  echo ERROR: Could not initialize the Visual Studio compiler.
  exit /b 5
)

where cmake >nul 2>nul
if errorlevel 1 (
  echo ERROR: CMake was not found. Add CMake tools for Windows in Visual Studio Installer.
  exit /b 6
)

where ninja >nul 2>nul
if errorlevel 1 (
  echo ERROR: Ninja was not found. Add CMake tools for Windows in Visual Studio Installer.
  exit /b 7
)

set "VCPKG_ROOT=%USERPROFILE%\vcpkg"
if not exist "%VCPKG_ROOT%\vcpkg.exe" (
  where git >nul 2>nul
  if errorlevel 1 (
    echo ERROR: Git was not found. Install Git for Windows and restart VS Code.
    exit /b 8
  )

  if not exist "%VCPKG_ROOT%\.git" (
    echo Downloading vcpkg...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_ROOT%"
    if errorlevel 1 exit /b 9
  )

  echo Preparing vcpkg...
  call "%VCPKG_ROOT%\bootstrap-vcpkg.bat" -disableMetrics
  if errorlevel 1 exit /b 10
)

echo Configuring simulator and dependencies...
cmake --preset release-vcpkg
if errorlevel 1 exit /b 11

echo Building simulator...
cmake --build --preset release-vcpkg
if errorlevel 1 exit /b 12

echo Starting simulator with:
echo   %STL_PATH%
"%~dp0build\release\solar_whitted_cpp.exe" --stl "%STL_PATH%" --rays 64 --output "%~dp0results\cell-irradiance.csv"
exit /b %errorlevel%
