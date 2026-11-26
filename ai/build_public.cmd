@echo off
setlocal
rem Builds the AI as a static library and links the host skeleton against it (the link test), MSVC x64, /W4 /WX.
rem   build\ground_ai.lib          the library (every public .c file; the .inc files are included by them)
rem   build\host_stub_link.exe     host_stub\ground_host_stub.c + the whole library: links only if the skeleton
rem                                provides every symbol the library needs. Not meant to be run.
rem Intermediates go to build\obj (overwritten in place, never deleted).
for /f "usebackq delims=" %%I in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "GA_VS=%%I"
if not defined GA_VS (
    echo Install the Visual Studio x64 C build tools.
    exit /b 1
)
call "%GA_VS%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
set "ROOT=%~dp0"
set "OUT=%~dp0build"
if not exist "%OUT%" mkdir "%OUT%" || exit /b 1
if not exist "%OUT%\obj" mkdir "%OUT%\obj" || exit /b 1
set "SRC=ground_brain ground_command ground_motion ground_nav ground_motion_control ground_lidar ground_overhead ground_routes ground_network"
set "CFLAGS=/nologo /std:c11 /utf-8 /W4 /WX /O2 /MT"
set "OBJS="
for %%S in (%SRC%) do (
    cl %CFLAGS% /c /Fo"%OUT%\obj\%%S.obj" "%ROOT%%%S.c" || exit /b 1
    call set "OBJS=%%OBJS%% "%OUT%\obj\%%S.obj""
)
lib /nologo /OUT:"%OUT%\ground_ai.lib" %OBJS% || exit /b 1
cl %CFLAGS% /DGROUND_HOST_STUB_MAIN /I"%ROOT%." /Fo"%OUT%\obj\ground_host_stub.obj" "%ROOT%host_stub\ground_host_stub.c" /link /WHOLEARCHIVE:"%OUT%\ground_ai.lib" "%OUT%\ground_ai.lib" /OUT:"%OUT%\host_stub_link.exe" /PDB:"%OUT%\obj\host_stub_link.pdb" || exit /b 1
echo build\ground_ai.lib and build\host_stub_link.exe built
exit /b 0
