@echo off

set CompilerFlags=-g -fno-limit-debug-info -fdiagnostics-absolute-paths -I ..\thirdparty -D_CRT_SECURE_NO_WARNINGS
set CompilerErrorFlags=-Werror -Wall
set Shader=.\shaders\shader.hlsl

where /Q cl.exe || (
    echo Could not locate cl.exe
    exit /b 1
)

if "%1" equ "debug" (
    set CL=/MTd /Od /Zi /D_DEBUG /fsanitize=address
    set LINK=/DEBUG:FULL
) else (
    set CL=/GL /O2 /DNDEBUG /GS-
    set LINK=/OPT:REF /OPT:ICF
)

set CL=%CL% /I ../thirdparty
set LINK=%LINK% User32.lib d3d11.lib dxguid.lib

where /Q fxc.exe || (
if not exist .\src\generated mkdir .\src\generated
fxc /nologo /Vn g_VertexShaderCode /E VertexShaderMain /T vs_4_0 /Fh .\src\generated\vertex_shader.h %Shader%
fxc /nologo /Vn g_PixelShaderCode /E PixelShaderMain /T ps_4_0 /Fh .\src\generated\pixel_shader.h %Shader%
fxc /nologo /Vn g_DebugVertexShaderCode /E DEBUGVertexShaderMain /T vs_4_0 ^
    /Fh .\src\generated\dvertex_shader.h %Shader%
fxc /nologo /Vn g_DebugPixelShaderCode /E DEBUGPixelShaderMain /T ps_4_0 ^
    /Fh .\src\generated\dpixel_shader.h %Shader%
)

if not exist build mkdir build
pushd build
cl.exe /nologo /FC /MP ..\src\platform_win32.cpp /Fe:app.exe /link /INCREMENTAL:NO /SUBSYSTEM:WINDOWS
popd build

