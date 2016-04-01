// Win32Project3.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "Win32Project3.h"
#include <cwchar>
#include <gdiplus.h>




#define MAX_LOADSTRING 100
#define IDC_GDI_CAPTURINGANIMAGE  103

// Global Variables:
HINSTANCE hInst;                        // current instance
TCHAR szTitle[MAX_LOADSTRING];          // The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];    // the main window class name
HWND cliwin;
IStorage* youStorage = NULL;

										// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
BOOL CALLBACK EnumWindowsProc(HWND hWnd, long lParam);
INT GetEncoderClsid(const WCHAR* format, CLSID* pClsid);  // helper function

int APIENTRY _tWinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_GDI_CAPTURINGANIMAGE, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Create a compound file object, and get
	// a pointer to its IStorage interface.
	StgCreateDocfile(
		L"CompoundFile.cmp",
		STGM_READWRITE | STGM_CREATE | STGM_SHARE_EXCLUSIVE,
		0,
		&youStorage);

//	if (FAILED(hr)) hr is the return value of the above function
//		goto Exit;

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));
	UINT_PTR timer = SetTimer(
		cliwin,
		0,
		1,//Milliseconds
		NULL
		);

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}


	return (int)msg.wParam;
}


int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

	Gdiplus::GetImageEncodersSize(&num, &size);
	if (size == 0)
		return -1;  // Failure

	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
	if (pImageCodecInfo == NULL)
		return -1;  // Failure

	Gdiplus::GetImageEncoders(num, size, pImageCodecInfo);

	for (UINT j = 0; j < num; ++j)
	{
		if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}
	}

	free(pImageCodecInfo);
	return -1;  // Failure
}


//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage are only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	HWND hWnd;

	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		500, 100, 300, 100, NULL, NULL, hInstance, NULL);

	/*hWnd = CreateWindow(
	L"BUTTON",  // Predefined class; Unicode assumed
	L"Press me?",      // Button text
	WS_TABSTOP | WS_CAPTION	 | BS_DEFPUSHBUTTON,  // Styles
	10,         // x position
	10,         // y position
	150,        // Button width
	100,        // Button height
	NULL,     // Parent window
	NULL,       // No menu.
	hInstance,
	NULL);      // Pointer not needed.*/
	cliwin = hWnd;
	if (!hWnd)
	{
		return FALSE;
	}
	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// Copyright (c) Microsoft Corporation. All rights reserved

//
//   FUNCTION: CaptureAnImage(HWND active)
//
//   PURPOSE: Captures a screenshot and saves it in a .bmp file.
//
//   COMMENTS: 
//
//      Note: This sample will attempt to create a file with the same title as the window, 
//			and a max limit of approximately 100 chars 
//        

//hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
	//CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);


int CaptureAnImage(HWND active)
{
	HWND hWnd = cliwin;
	HDC hdcActive;
	HDC hdcMemDC = NULL;
	HBITMAP hbmActive = NULL;
	IStream* youStream = NULL;
//	BITMAP bmpActive;

// Initialize GDI+.
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	// Retrieve the handle to a display device context for the client 
	// area of the window. 
	hdcActive = GetWindowDC(active);

	// Create a compatible DC which is used in a BitBlt from the window DC
	hdcMemDC = CreateCompatibleDC(hdcActive);

	if (!hdcMemDC)
	{
		MessageBox(hWnd, L"CreateCompatibleDC has failed", L"Failed", MB_OK);
		goto done;
	}

	// Get the Window area for size calculation
	RECT rcWindow;
	GetWindowRect(active, &rcWindow);

	// Create a compatible bitmap from the Active DC
	hbmActive = CreateCompatibleBitmap(hdcActive, rcWindow.right - rcWindow.left, rcWindow.bottom - rcWindow.top);
	// Select the compatible bitmap into the compatible memory DC.
	SelectObject(hdcMemDC, hbmActive);

	// Bit block transfer into our compatible memory DC.
	if (!BitBlt(hdcMemDC,
		0, 0,
		rcWindow.right - rcWindow.left, rcWindow.bottom - rcWindow.top,
		hdcActive,
		0, 0,
		SRCCOPY))
	{
		MessageBox(hWnd, L"BitBlt has failed", L"Failed", MB_OK);
		goto done;
	}
	if (!hbmActive)
	{
		MessageBox(hWnd, L"CreateCompatibleBitmap Failed", L"Failed", MB_OK);
		goto done;
	}
	CreateDirectory(L"pictures", NULL);
	wchar_t titley[100];
	wchar_t title[100];
	GetWindowText(active, titley, 50);
	wcsncpy_s(title, 100, L"pictures/", 9);
	wcsncat_s(title, 100, titley, 50);
	wcsncat_s(title, 100, L".jpg", 4);
	youStorage->CreateStream(
		title,
		STGM_READWRITE | STGM_SHARE_EXCLUSIVE,
		0,
		0,
		&youStream);

//	IStream* watermelon = SHCreateMemStream(NULL, 0);

	CLSID *jpgclsid = new CLSID;
	GetEncoderClsid(L"image/jpeg", jpgclsid);
	Gdiplus::Bitmap* sah = Gdiplus::Bitmap::FromHBITMAP(hbmActive, NULL);
	sah->Save(title, jpgclsid, 0);
	sah->Save(youStream, jpgclsid, 0);
	/*


	// Select the compatible bitmap into the compatible memory DC.
	SelectObject(hdcMemDC, hbmActive);

	// Bit block transfer into our compatible memory DC.
	if (!BitBlt(hdcMemDC,
		0, 0,
		rcWindow.right - rcWindow.left, rcWindow.bottom - rcWindow.top,
		hdcActive,
		0, 0,
		SRCCOPY))
	{
		MessageBox(hWnd, L"BitBlt has failed", L"Failed", MB_OK);
		goto done;
	}

	// Get the BITMAP from the HBITMAP
	GetObject(hbmActive, sizeof(BITMAP), &bmpActive);

	BITMAPFILEHEADER   bmfHeader;
	BITMAPINFOHEADER   bi;

	bi.biSize = sizeof(BITMAPINFOHEADER);
	bi.biWidth = bmpActive.bmWidth;
	bi.biHeight = bmpActive.bmHeight;
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = BI_RGB;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	DWORD dwBmpSize = ((bmpActive.bmWidth * bi.biBitCount + 31) / 32) * 4 * bmpActive.bmHeight;

	// Starting with 32-bit Windows, GlobalAlloc and LocalAlloc are implemented as wrapper functions that 
	// call HeapAlloc using a handle to the process's default heap. Therefore, GlobalAlloc and LocalAlloc 
	// have greater overhead than HeapAlloc.
	HANDLE hDIB = GlobalAlloc(GHND, dwBmpSize);
	char *lpbitmap = (char *)GlobalLock(hDIB);

	// Gets the "bits" from the bitmap and copies them into a buffer 
	// which is pointed to by lpbitmap.
	GetDIBits(hdcActive, hbmActive, 0,
		(UINT)bmpActive.bmHeight,
		lpbitmap,
		(BITMAPINFO *)&bi, DIB_RGB_COLORS);

	// A file is created, this is where we will save the screen capture.
	/*
	CreateDirectory(L"pictures", NULL);
	wchar_t titley[100];
	wchar_t title[100];
	GetWindowText(active, titley, 50);
	wcsncpy_s(title,100,L"pictures\/", 9);
	wcsncat_s(title, 100, titley, 50);
	wcsncat_s(title, 100, L".bmp", 4);*/

	/*
	HANDLE hFile = CreateFile(title,
		GENERIC_WRITE,
		0,
		NULL,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL, NULL);

	// Add the size of the headers to the size of the bitmap to get the total file size
	DWORD dwSizeofDIB = dwBmpSize + sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	//Offset to where the actual bitmap bits start.
	bmfHeader.bfOffBits = (DWORD)sizeof(BITMAPFILEHEADER) + (DWORD)sizeof(BITMAPINFOHEADER);

	//Size of the file
	bmfHeader.bfSize = dwSizeofDIB;

	//bfType must always be BM for Bitmaps
	bmfHeader.bfType = 0x4D42; //BM   

	DWORD dwBytesWritten = 0;
	WriteFile(hFile, (LPSTR)&bmfHeader, sizeof(BITMAPFILEHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)&bi, sizeof(BITMAPINFOHEADER), &dwBytesWritten, NULL);
	WriteFile(hFile, (LPSTR)lpbitmap, dwBmpSize, &dwBytesWritten, NULL);
	System.Drawing::bmpActive.Save(title, System::Drawing::Imaging::ImageFormat::Jpeg);

	*/
	//Unlock and Free the DIB from the heap
	//GlobalUnlock(hDIB);
	//GlobalFree(hDIB);

	//Close the handle for the file that was created
	//Do we need to close folder?
	//CloseHandle(hFile);

	//Clean up
done:
	DeleteObject(hbmActive);
	DeleteObject(hdcMemDC);
	ReleaseDC(active, hdcActive);
	if (youStream) {
		youStream->Release();
	}
	Gdiplus::GdiplusShutdown(gdiplusToken);
	return 0;
}

BOOL CALLBACK EnumWindowsProc(HWND hWnd, long lParam) {
	TCHAR szText[256];
	if (IsWindowVisible(hWnd) && GetWindow(hWnd, GW_OWNER) == NULL) {
		//Visible, has no owners
		if (GetWindowText(hWnd, szText, 256) == 0) // No text in window
			return TRUE;
		//Checking to see if the window has a title bar.
		CaptureAnImage(hWnd);
	}
	UINT_PTR timer = SetTimer(
		NULL,
		0,
		2,//Milliseconds
		NULL
		);

	return TRUE;
}
//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND    - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY    - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_CREATE:
	{
		break;
	}
	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;

	case WM_TIMER:

	case WM_MOVE:

	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		EnumWindows(EnumWindowsProc, 0);
		EndPaint(hWnd, &ps);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
