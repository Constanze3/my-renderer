
#include <Windows.h>
#include <windowsx.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <chrono>

#include "draw.h"


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

HWND hwnd;

const wchar_t* CLASSNAME = L"my_window";
uint32_t win_width = 640;
uint32_t win_height = 480;
HBITMAP hBitmap = nullptr;
void* pBits = nullptr;
bool is_drawing = false;
HDC hdcOffscreen = nullptr;

Clock::time_point start;

auto CreateBitmapFromRGB(char* pData, int width, int height)
-> std::pair<HBITMAP, void*> {
    BITMAPINFO bmi = { 0 };

    bmi.bmiHeader.biSize = sizeof(bmi.bmiHeader);
    bmi.bmiHeader.biWidth = width;
    bmi.bmiHeader.biHeight = -437;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 24;
    bmi.bmiHeader.biCompression = BI_RGB;

    HDC hdc = GetDC(nullptr);
    void* pBits;

    // Create a Device Independent Bitmap
    // 
    // Why isn't CreateDIBitmap used here?
    // Because CreateDIBitmap uses the color depth of the device context (HDC) 
    // i.e. the (primary) screen and here we want a bit depth of exactly 24 (biPlanes * biBitCount).
    HBITMAP hbm = CreateDIBSection(hdc, &bmi, DIB_RGB_COLORS, &pBits, nullptr, 0);
    /*if (hbm != nullptr) {
        std::memcpy(pBits, pData, width * height * 3);
    }*/
    ReleaseDC(nullptr, hdc);
    return { hbm, pBits };

}

void InitializeOffScreenDC(HWND hwnd) {
    std::unique_ptr<char[]> raw_data(new char[win_width * win_height * 3]);
    memset(raw_data.get(), 0x0, win_width * win_height * 3);

    /*
    std::ifstream ifs("./sample.pbm", std::ios::binary);
    if (ifs.fail()) {
        std::cout << "error opening sample.pbm";
    }
    std::string header;
    int width, height, bpp;

    ifs >> header;
    ifs >> width >> height >> bpp;
    ifs.ignore();
    ifs.read(raw_data.get(), win_width * win_height * 3);
    for (uint32_t i = 0; i < win_width * win_height * 3; i += 3) {
        std::swap(raw_data[i], raw_data[i + 2]);
    }
    ifs.close();
    */

    auto bitmap_data = CreateBitmapFromRGB(raw_data.get(), win_width, win_height);
    hBitmap = bitmap_data.first;
    pBits = bitmap_data.second;

    HDC hdc = GetDC(hwnd);
    hdcOffscreen = CreateCompatibleDC(hdc);
    SelectObject(hdcOffscreen, hBitmap);
    ReleaseDC(hwnd, hdc);
}

void CleanupOffScreenDc() {
    if (hdcOffscreen) DeleteDC(hdcOffscreen);
}

void SetPixelColor(void* pBits, int width, int height, int x, int y, uint8_t red, uint8_t green, uint8_t blue) {
    if (!pBits) return;

    int pixel_index = (y * width + x) * 3;

    uint8_t* pPixel = static_cast<uint8_t*>(pBits) + pixel_index;
    
    pPixel[0] = blue;
    pPixel[1] = green;
    pPixel[2] = red;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CLOSE:
        if (hBitmap != nullptr) {
            DeleteObject(hBitmap);
            hBitmap = nullptr;
        }
        CleanupOffScreenDc();
        DestroyWindow(hWnd);
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    case WM_LBUTTONDOWN:
        is_drawing = true;
        break;
    case WM_LBUTTONUP:
        is_drawing = false;
        break;
    case WM_MOUSEMOVE: {
        int xpos = GET_X_LPARAM(lParam);
        int ypos = GET_Y_LPARAM(lParam);

        if (is_drawing) {
            SetPixelColor(pBits, win_width, win_height, xpos, ypos, 255, 0, 0);
            InvalidateRect(hWnd, NULL, TRUE);
        }
        break;
    }
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            BitBlt(hdc, 0, 0, win_width, 437, hdcOffscreen, 0, 0, SRCCOPY);
            EndPaint(hWnd, &ps);
        }
        break;
    default:
        return DefWindowProc(hWnd, msg, wParam, lParam);
    }

    return 0;
}

void CreateAndRegisterWindow(HINSTANCE hInstance) {
    WNDCLASSEX wc = {0};
    wc.cbSize = sizeof(WNDCLASSEX);
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = CLASSNAME;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hIcon = LoadIcon(hInstance, IDI_APPLICATION);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.lpszMenuName = nullptr;
    wc.hIconSm = LoadIcon(hInstance, IDI_APPLICATION);

    if (!RegisterClassEx(&wc)) {
        MessageBox(nullptr, L"Window Registration Failed", L"Error",
            MB_ICONEXCLAMATION | MB_OK);
    }

    hwnd = CreateWindowEx(
        WS_EX_CLIENTEDGE,
        CLASSNAME,
        L"Foo",
        WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
        CW_USEDEFAULT, CW_USEDEFAULT, win_width, win_height,
        nullptr, nullptr, hInstance, nullptr);

    if (hwnd == nullptr) {
        MessageBox(nullptr, L"Window Creation Failed", L"Error",
            MB_ICONEXCLAMATION | MB_OK);
    }
    else {
        InitializeOffScreenDC(hwnd);

        ShowWindow(hwnd, SW_SHOWDEFAULT);
        UpdateWindow(hwnd);
    }
}

int main(int argc, char** argv) {
    start = Clock::now();
    HINSTANCE hInstance = GetModuleHandle(NULL);

    CreateAndRegisterWindow(hInstance);
    MSG msg;

    InvalidateRect(hwnd, NULL, TRUE);

    RECT rect;
    GetClientRect(hwnd, &rect);
    
    std::cout << rect.right - rect.left << "\n";
    std::cout << rect.top - rect.bottom << "\n";

    while (1) {
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE) != 0) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) {
                break;
            }
        }

        if (msg.message == WM_QUIT) {
            break;
        }

        
        float elapsed = std::chrono::duration_cast<milliseconds>(Clock::now() - start).count() / 1000.f;
        draw(pBits, win_width, 437, elapsed);
        InvalidateRect(hwnd, NULL, TRUE);
    }

    return 0;
}