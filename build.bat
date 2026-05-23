@echo off
setlocal

echo [drone_sim] Initialising submodules...
git submodule update --init --recursive

echo [drone_sim] Configuring (Release)...
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 goto :error

echo [drone_sim] Building...
cmake --build build --config Release --parallel
if errorlevel 1 goto :error

echo.
echo [drone_sim] Build succeeded.
echo Output: gdextension\bin\drone_sim.windows.release.x86_64.dll
goto :eof

:error
echo.
echo [drone_sim] BUILD FAILED — check output above.
exit /b 1
