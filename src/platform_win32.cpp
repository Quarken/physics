#include <stdint.h>
#include <string.h>

#include "common.h"
#include "mathlib.h"
#include "geometry.h"
#include "main.h"
#include "renderer.h"
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "bvh.cpp"
#include "geometry.cpp"
#include "physics.cpp"
#include "main.cpp"
#include "renderer_dx11.cpp"

#include "imgui.cpp"
#include "imgui_draw.cpp"
#include "imgui_tables.cpp"
#include "imgui_widgets.cpp"
#include "imgui_impl_win32.cpp"
#include "imgui_impl_dx11.cpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>
#include <hidusage.h>

static struct
{
    bool is_running;
} globals;

void *PlatformReserveMemory(u64 size)
{
    return VirtualAlloc(0, size, MEM_RESERVE, PAGE_NOACCESS);
}

void PlatformCommitMemory(void *memory, u64 size)
{
    VirtualAlloc(memory, size, MEM_COMMIT, PAGE_READWRITE);
}

void PlatformReleaseMemory(void *memory)
{
    VirtualFree(memory, 0, MEM_RELEASE);
}

LRESULT Win32WindowProc(HWND window, UINT msg, WPARAM wparam, LPARAM lparam)
{
    LRESULT result = 0;
    
    switch (msg)
    {
        case WM_QUIT:
        case WM_DESTROY:
        {
            globals.is_running = false;
        } break;

        default:
        {
            return DefWindowProcA(window, msg, wparam, lparam);
        }
    }

    return result;
}

int WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int)
{
    LARGE_INTEGER PerformanceFrequency;
    QueryPerformanceFrequency(&PerformanceFrequency);

    WNDCLASSEXA window_class = {};
    window_class.cbSize = sizeof(WNDCLASSEXA);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = (WNDPROC)Win32WindowProc;
    window_class.hInstance = instance;
    window_class.lpszClassName = "MainWindowClass";
    
    if (!RegisterClassExA(&window_class))
    {
        return -1;
    }

    HWND main_window = CreateWindowExA(0,
                                       window_class.lpszClassName,
                                       "Manifold",
                                       WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                                       CW_USEDEFAULT, CW_USEDEFAULT,
                                       1920, 1080,
                                       NULL, NULL,
                                       instance,
                                       NULL);

    if (!main_window)
    {
        return -1;
    }

    RAWINPUTDEVICE RawDevice;
    RawDevice.usUsagePage = HID_USAGE_PAGE_GENERIC;
    RawDevice.usUsage = HID_USAGE_GENERIC_MOUSE;
    RawDevice.dwFlags = 0;
    RawDevice.hwndTarget = main_window;
    RegisterRawInputDevices(&RawDevice, 1, sizeof(RAWINPUTDEVICE));

    HCURSOR Cursor = LoadCursor(NULL, IDC_ARROW);
    SetCursor(Cursor);

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= 
        ImGuiConfigFlags_NavEnableKeyboard |
        ImGuiConfigFlags_DockingEnable;
    ImGui_ImplWin32_Init(main_window);
    RendererInitialize();

    LARGE_INTEGER CurrentTime;
    LARGE_INTEGER NewTime;
    QueryPerformanceCounter(&CurrentTime);

    app_input Input;
    globals.is_running = true;
    while (globals.is_running)
    {
        QueryPerformanceCounter(&NewTime);
        float FrameTimeInSeconds = 
            (float)(NewTime.QuadPart - CurrentTime.QuadPart) /
            (float)(PerformanceFrequency.QuadPart);
        CurrentTime = NewTime;

        Input.MouseDeltaX = 0;
        Input.MouseDeltaY = 0;
        Input.MouseWheelDelta = 0;
        MSG Message;
        while (PeekMessage(&Message, NULL, 0, 0, PM_REMOVE))
        {
            ImGui_ImplWin32_WndProcHandler(main_window, Message.message, Message.wParam, Message.lParam);
            switch (Message.message)
            {
                case WM_INPUT:
                {
                    if (io.WantCaptureMouse) break;
                    if (GET_RAWINPUT_CODE_WPARAM(Message.wParam) == RIM_INPUTSINK) break;
                    RAWINPUT RawInput;
                    u32 RawInputSize = 48;
                    GetRawInputData((HRAWINPUT)Message.lParam, RID_INPUT, &RawInput,
                                    &RawInputSize, sizeof(RAWINPUTHEADER));
                    if (RawInput.header.dwType == RIM_TYPEMOUSE)
                    {
                        Input.MouseDeltaX += RawInput.data.mouse.lLastX;
                        Input.MouseDeltaY += RawInput.data.mouse.lLastY;
                    }
                    DefWindowProc(main_window, Message.message, Message.wParam, Message.lParam);
                } break;

                case WM_LBUTTONDOWN:
                case WM_LBUTTONUP:
                {
                    if (io.WantCaptureMouse) break;
                    Input.MouseDown = (Message.wParam & MK_LBUTTON) != 0;
                } break;
                case WM_MOUSEWHEEL:
                {
                    if (io.WantCaptureMouse) break;
                    Input.MouseWheelDelta = GET_WHEEL_DELTA_WPARAM(Message.wParam);
                } break;

                default:
                {
                    TranslateMessage(&Message);
                    DispatchMessage(&Message);
                } break;
            }
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        UpdateAndRender(FrameTimeInSeconds, &Input);
    }

    return 0;
}

