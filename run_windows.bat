@echo off
setlocal EnableExtensions EnableDelayedExpansion
cd /d "%~dp0"

set "STL_PATH=%~1"
set "TOP_SHELL_PATH=%~2"
set "AERO_NAME=24-AER-1000 (Aeroshell).STL"
set "FIXED_SHELL_NAME=24-AER-1001 (Top Shell) Fixed Coords.STL"
if not defined STL_PATH if exist "%USERPROFILE%\Downloads\%AERO_NAME%" if exist "%USERPROFILE%\Downloads\%FIXED_SHELL_NAME%" set "STL_PATH=%USERPROFILE%\Downloads\%AERO_NAME%"
if not defined STL_PATH set "STL_PATH=%USERPROFILE%\Downloads\_24-000.stl"

if not exist "%STL_PATH%" (
  echo Select the FULL CAR STL from your computer...
  set "STL_PATH="
  for /f "usebackq delims=" %%F in (`powershell -NoProfile -STA -Command "Add-Type -AssemblyName System.Windows.Forms; $solarPicker = New-Object System.Windows.Forms.OpenFileDialog; $solarPicker.Filter = 'STL files (*.stl)' + [char]124 + '*.stl'; $solarPicker.Title = 'Select the full car STL'; if ($solarPicker.ShowDialog() -eq 'OK') { [Console]::WriteLine($solarPicker.FileName) }"`) do set "STL_PATH=%%F"
)
if not defined STL_PATH (
  echo No full-car STL selected.
  exit /b 2
)
if not exist "%STL_PATH%" exit /b 2

rem The verified aero exports use millimeters and differ by +83.449 mm in X.
set "MODEL_ARGS="
for %%F in ("%STL_PATH%") do if /I "%%~nxF"=="%AERO_NAME%" (
  set "MODEL_ARGS=--scale 0.001"
  if not defined TOP_SHELL_PATH if exist "%%~dpF%FIXED_SHELL_NAME%" set "TOP_SHELL_PATH=%%~dpF%FIXED_SHELL_NAME%"
  if defined TOP_SHELL_PATH for %%S in ("!TOP_SHELL_PATH!") do if /I "%%~nxS"=="%FIXED_SHELL_NAME%" set "MODEL_ARGS=--scale 0.001 --top-offset-x 0.083449"
)

where winget >nul 2>nul
if errorlevel 1 (
  set "HAS_WINGET=0"
) else (
  set "HAS_WINGET=1"
)

rem Install or repair the complete Visual Studio C++ toolchain.
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  if "!HAS_WINGET!"=="0" (
    echo ERROR: Windows Package Manager is unavailable.
    echo Install App Installer from Microsoft Store, then run this file again.
    exit /b 3
  )
  echo Installing Visual Studio C++ Build Tools, Windows SDK, CMake, and Ninja...
  winget install --id Microsoft.VisualStudio.2022.BuildTools -e --source winget --accept-source-agreements --accept-package-agreements --override "--wait --passive --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
  if errorlevel 1 exit /b 4
)

if not exist "%VSWHERE%" (
  echo ERROR: Visual Studio Installer did not provide vswhere.exe.
  echo Restart Windows, then run this file again.
  exit /b 5
)

set "VS_INSTALL="
for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"

if not defined VS_INSTALL (
  set "VS_BASE="
  for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -property installationPath`) do set "VS_BASE=%%I"

  if defined VS_BASE (
    set "VS_SETUP=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\setup.exe"
    if not exist "!VS_SETUP!" (
      echo ERROR: Visual Studio Installer setup.exe was not found.
      exit /b 6
    )
    echo Adding the Desktop development with C++ workload...
    "!VS_SETUP!" modify --installPath "!VS_BASE!" --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --passive --norestart
    set "VS_RESULT=!errorlevel!"
    if not "!VS_RESULT!"=="0" if not "!VS_RESULT!"=="3010" exit /b 7
  ) else (
    if "!HAS_WINGET!"=="0" (
      echo ERROR: Visual Studio C++ Build Tools are missing and winget is unavailable.
      exit /b 8
    )
    echo Installing Visual Studio C++ Build Tools, Windows SDK, CMake, and Ninja...
    winget install --id Microsoft.VisualStudio.2022.BuildTools -e --source winget --accept-source-agreements --accept-package-agreements --override "--wait --passive --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    if errorlevel 1 exit /b 9
  )

  set "VS_INSTALL="
  for /f "usebackq tokens=*" %%I in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"
)

if not defined VS_INSTALL (
  echo ERROR: The C++ workload installation has not become available yet.
  echo Restart Windows, then run this file again.
  exit /b 10
)

call "%VS_INSTALL%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
if errorlevel 1 (
  echo ERROR: Could not initialize the Visual Studio compiler.
  exit /b 11
)

rem Prefer the CMake and Ninja copies bundled with Visual Studio.
set "VS_CMAKE_BIN=%VS_INSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
set "VS_NINJA_BIN=%VS_INSTALL%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
if exist "%VS_CMAKE_BIN%\cmake.exe" set "PATH=%VS_CMAKE_BIN%;%PATH%"
if exist "%VS_NINJA_BIN%\ninja.exe" set "PATH=%VS_NINJA_BIN%;%PATH%"

rem Fall back to standalone packages when a partial VS installation omitted them.
where cmake >nul 2>nul
if errorlevel 1 (
  if "!HAS_WINGET!"=="0" (
    echo ERROR: CMake is missing and winget is unavailable.
    exit /b 12
  )
  echo Installing CMake...
  winget install --id Kitware.CMake -e --source winget --accept-source-agreements --accept-package-agreements
  if errorlevel 1 exit /b 13
  set "PATH=%ProgramFiles%\CMake\bin;%ProgramFiles(x86)%\CMake\bin;!PATH!"
)

where cmake >nul 2>nul
if errorlevel 1 (
  echo ERROR: CMake was installed but is not available yet. Restart VS Code and rerun.
  exit /b 14
)

where ninja >nul 2>nul
if errorlevel 1 (
  if "!HAS_WINGET!"=="0" (
    echo ERROR: Ninja is missing and winget is unavailable.
    exit /b 15
  )
  echo Installing Ninja...
  winget install --id Ninja-build.Ninja -e --source winget --accept-source-agreements --accept-package-agreements
  if errorlevel 1 exit /b 16
  set "PATH=%LOCALAPPDATA%\Microsoft\WinGet\Links;!PATH!"
)

where ninja >nul 2>nul
if errorlevel 1 (
  echo ERROR: Ninja was installed but is not available yet. Restart VS Code and rerun.
  exit /b 17
)

where git >nul 2>nul
if errorlevel 1 (
  if "!HAS_WINGET!"=="0" (
    echo ERROR: Git is missing and winget is unavailable.
    exit /b 18
  )
  echo Installing Git for Windows...
  winget install --id Git.Git -e --source winget --accept-source-agreements --accept-package-agreements
  if errorlevel 1 exit /b 19
  set "PATH=%ProgramFiles%\Git\cmd;!PATH!"
)

where git >nul 2>nul
if errorlevel 1 (
  echo ERROR: Git was installed but is not available yet. Restart VS Code and rerun.
  exit /b 20
)

rem vcpkg installs Embree, GLFW, GLEW, GLM, Dear ImGui, CPR, and JSON support.
set "VCPKG_ROOT=%USERPROFILE%\vcpkg"
if not exist "%VCPKG_ROOT%\vcpkg.exe" (
  if not exist "%VCPKG_ROOT%\.git" (
    echo Downloading vcpkg...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_ROOT%"
    if errorlevel 1 exit /b 21
  )

  echo Preparing vcpkg...
  call "%VCPKG_ROOT%\bootstrap-vcpkg.bat" -disableMetrics
  if errorlevel 1 exit /b 22
)

echo Configuring simulator and installing C++ libraries...
cmake --preset release-vcpkg
if errorlevel 1 exit /b 23

echo Building simulator...
cmake --build --preset release-vcpkg
if errorlevel 1 exit /b 24

echo Starting simulator with:
echo   "%STL_PATH%"
if defined TOP_SHELL_PATH (
  "%~dp0build\release\solar_whitted_cpp.exe" --stl "%STL_PATH%" --top-shell "%TOP_SHELL_PATH%" %MODEL_ARGS% --rays 64 --output "%~dp0results\cell-irradiance.csv"
) else (
  "%~dp0build\release\solar_whitted_cpp.exe" --stl "%STL_PATH%" %MODEL_ARGS% --rays 64 --output "%~dp0results\cell-irradiance.csv"
)
exit /b %errorlevel%
