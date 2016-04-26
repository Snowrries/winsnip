// Win32Project3.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "Win32Project3.h"
#include <cwchar>
#include <gdiplus.h>
#include <Windows.h>
#include <tchar.h>
#include <string>

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")
//For use with network socketing

#define DEFAULT_PORT "8000"

#define MAX_LOADSTRING 100
#define IDC_GDI_CAPTURINGANIMAGE  103
#define IDC_MAIN_BUTTON	101			// Button identifier
#define IDC_MAIN_EDIT	102			// Edit box identifier
HWND hEdit;

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
// Global Variables:
HINSTANCE hInst;                        // current instance
TCHAR szTitle[MAX_LOADSTRING];          // The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];    // the main window class name
HWND cliwin;
//IStorage* youStorage = NULL; //Currently unnecessary
SOCKET ConnectSocket = INVALID_SOCKET;

/// Forward declarations of functions included in this code module:
//ATOM                MyRegisterClass(HINSTANCE hInstance);
//BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
BOOL CALLBACK		EnumWindowsProc(HWND hWnd, long lParam);
INT					GetEncoderClsid(const WCHAR* format, CLSID* pClsid);  // helper function

char ip[25] = { 0 };
int buttonpress = 0;


///Main function. First argument in command line should be the IP address of the server. 
int APIENTRY _tWinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow)
{

//	HACCEL hAccelTable;


	HACCEL hAccelTable;
	int iResult; //To take the result of function calls
	WSADATA wsaData; //Sockets.
	struct addrinfo *result = NULL, //For socketing address purposes
		*ptr = NULL,
		hints;

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	LPTSTR windowClass = TEXT("WinApp");
	LPTSTR windowTitle = TEXT("WinSnip Texture Streamer");
	WNDCLASSEX wcex;

	wcex.cbClsExtra = 0;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.cbWndExtra = 0;
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));
	wcex.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	wcex.hInstance = hInstance;
	wcex.lpfnWndProc = WndProc;
	wcex.lpszClassName = windowClass;
	wcex.lpszMenuName = MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	if (!RegisterClassEx(&wcex))
	{
		MessageBox(NULL, TEXT("RegisterClassEx Failed!"), TEXT("Error"),
			MB_ICONERROR);
		return EXIT_FAILURE;
	}

	HWND hWnd;
	//#define CreateWindowW(lpClassName, lpWindowName, dwStyle, x, y,\
    //nWidth, nHeight, hWndParent, hMenu, hInstance, lpParam)
	if (!(hWnd = CreateWindow(windowClass, windowTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, CW_USEDEFAULT, 600,
		350, NULL, NULL, hInstance, NULL)))
	{
		MessageBox(NULL, TEXT("CreateWindow Failed!"), TEXT("Error"), MB_ICONERROR);
		return EXIT_FAILURE;
	}

	cliwin = hWnd;

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	int connected = 0;
	MSG msg;
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));

	while (GetMessage(&msg, NULL, 0, 0) > 0)
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);

		//First time connection setup
		if (buttonpress) {
			iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
			if (iResult != 0) {
				printf("WSAStartup failed with error: %d\n", iResult);
				return 1;
			}

			ZeroMemory(&hints, sizeof(hints)); // A calloc
			hints.ai_family = AF_UNSPEC;
			hints.ai_socktype = SOCK_STREAM;
			hints.ai_protocol = IPPROTO_TCP;

			// Resolve the server address and port
			iResult = getaddrinfo(ip, DEFAULT_PORT, &hints, &result);
			if (iResult != 0) {
				printf("getaddrinfo failed with error: %d\n", iResult);
				WSACleanup();
				return 1;
			}
			// Attempt to connect to an address until one succeeds
			for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
				// Create a SOCKET for connecting to server
				ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
					ptr->ai_protocol);
				if (ConnectSocket == INVALID_SOCKET) {
					printf("socket failed with error: %ld\n", WSAGetLastError());
					WSACleanup();
					return 1;
				}

				// Connect to server.
				iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
				if (iResult == SOCKET_ERROR) {
					closesocket(ConnectSocket);
					ConnectSocket = INVALID_SOCKET;
					continue;
				}
				printf("Connected to socket at IP: %s",ip);
				connected = 1;
				buttonpress = 0;
				break;
			}

			freeaddrinfo(result);

			if (ConnectSocket == INVALID_SOCKET) {
				printf("Unable to connect to server!\n");
				WSACleanup();
				return 1;
			}

		}
		//We are connected send the byte stream

		//else if (connected == 1) {
			//Do nothing
		//}

	}

	
	return 0;

}
	/*
	MSG msg;
	HACCEL hAccelTable;
	int iResult; //To take the result of function calls
	WSADATA wsaData; //Sockets.
	struct addrinfo *result = NULL, //For socketing address purposes
		*ptr = NULL,
		hints;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_GDI_CAPTURINGANIMAGE, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Create a compound file object, and get
	// a pointer to its IStorage interface.
	//StgCreateDocfile(
	//	L"CompoundFile.cmp",
	//	STGM_READWRITE | STGM_CREATE | STGM_SHARE_EXCLUSIVE,
	//	0,
	//	&youStorage);

	//Socketing adapted from MSDN example

	// Perform application initialization:

	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints)); // A calloc
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("192.168.1.113", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}
	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		printf("Connected to socket at IP: 192.168.1.113");
		break;
	}

	freeaddrinfo(result);

if (ConnectSocket == INVALID_SOCKET) {
	printf("Unable to connect to server!\n");
	WSACleanup();
	return 1;
}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));
	
	//Set up window refresh timer
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
	*/
	//return 0;
//}

///
///   FUNCTION: CaptureAnImage(HWND active)
///
///   PURPOSE: Captures a screenshot of each window and saves them in .jpg files. Creates iStreams for each window and stores in a compound file.
///
///   COMMENTS: 
///
///      Note: This sample will attempt to create a file with the same title as the window, 
///			and a max limit of approximately 50 chars 
///        


int CaptureAnImage(HWND active)
{
	HWND hWnd = cliwin;
	HDC hdcActive;
	HDC hdcMemDC = NULL;
	HBITMAP hbmActive = NULL;
	IStream* youStream = NULL;
	ULONG count;
	ULARGE_INTEGER full;
	INT result;
//	BITMAP bmpActive;

	// Initialize GDI+.
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	ULONG_PTR gdiplusToken;
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
	youStream = SHCreateMemStream(NULL, 0);
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
	/*youStorage->CreateStream(
	title,
	STGM_READWRITE | STGM_SHARE_EXCLUSIVE,
	0,
	0,
	&youStream);
	*/
	
	CLSID *jpgclsid = new CLSID;
	GetEncoderClsid(L"image/jpeg", jpgclsid);
	Gdiplus::Bitmap* sah = Gdiplus::Bitmap::FromHBITMAP(hbmActive, NULL);
	//sah->Save(title, jpgclsid, 0);
	//sah->Save(youStream, jpgclsid, 0);
	//Above two lines are for the iStream implementation.
	//Below is the network socket implementation.
	sah->Save(youStream, jpgclsid, 0);
	//The istream created is in memory, should be able to read from youStream and write to a socket.
	count =  0;
	IStream_Size(youStream, &full);
	char* buffer = (char*)malloc((size_t)full.LowPart);//Doublecheck this line
	ULONG rcoun = 0;
	IStream_Reset(youStream);
	//Consider threading networking area below to increase efficiency and decrease hanging (?)
	//Issue: Would need to create a new socket for each thread if we go this route: likely infeasible for client-side.
	while (count < full.LowPart) {
		result = youStream->Read(&buffer[count], full.LowPart, &rcoun);
		//	result = IStream_Read(youStream, buffer, full.QuadPart);
		if (result != S_OK && result != S_FALSE) {
			System::Console::Write("iStream read error");
			System::Console::WriteLine();
			goto done;
		}
		count = count + rcoun;
	}
	//char start[2] = { 0xFF,0xD8 };
	//char end[2] = { 0xff, 0xd9 };
	char *buf;


	//Send name
	count = 0;
	size_t r;
	buf = (char*)malloc(64);
	wcstombs_s(&r,buf,64,titley,64);
	const char *len = (char*)strlen(buf);
	send(ConnectSocket, len, sizeof(len),NULL);

	while (count < strlen(buf)) {
		count += send(ConnectSocket, &buf[count], strlen(buf)-count, NULL);
	}
	free(buf);

	//Send 2 newlines
	
	len = (char*)full.LowPart;
	send(ConnectSocket, len, sizeof(len), NULL);
	//Send image
	while (count < full.LowPart) {
		//Write to socket and keep track of bytes written in count, update accordingly.

		count += send(ConnectSocket, &buffer[count], full.LowPart, NULL);

	}
	free(buffer);

	//Original code to save each window as a BMP. May need if higher resolution pictures are required.
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
	
	CreateDirectory(L"pictures", NULL);
	wchar_t titley[100];
	wchar_t title[100];
	GetWindowText(active, titley, 50);
	wcsncpy_s(title,100,L"pictures/", 9);
	wcsncat_s(title, 100, titley, 50);
	wcsncat_s(title, 100, L".bmp", 4);

	
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
	System::Drawing::bmpActive.Save(title, System::Drawing::Imaging::ImageFormat::Jpeg);

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

	///
	///   FUNCTION: EnumWindowsProc(HWND hWnd, long lParam)
	///
	///   PURPOSE: Callback function to enumerate through windows.
	///
	///   COMMENTS:
	///
	///			Set up a timer to enumerate through the windows every millisecond, and set a new timer everytime this function is called.
	BOOL CALLBACK EnumWindowsProc(HWND hWnd, long lParam) {
	TCHAR szText[256];
	if (IsWindowVisible(hWnd) && GetWindow(hWnd, GW_OWNER) == NULL) {
		//Visible, has no owners
		if (GetWindowText(hWnd, szText, 256) == 0) // No text in window
			return TRUE;
			//Checking to see if the window has a title bar.
			CaptureAnImage(hWnd);
		}
		//Reset window refresh timer
		UINT_PTR timer = SetTimer(
			cliwin,
			0,
			50,//Milliseconds
			NULL
			);

		return TRUE;
	}


LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	HDC hdc;
	TCHAR greeting[] = _T("\nPress the Connect button to begin streaming images to a server. ");
	TCHAR greeting2[] = _T("\nEnter an IP into the text box to connect to a specific server at port 8000.");
	TCHAR greeting3[] = _T("\nLeave box blank to connect to localhost server.");

	
	switch (msg)
	{
	case WM_CREATE:
	{
		// Create an edit box
		hEdit = CreateWindowEx(WS_EX_CLIENTEDGE,
			L"EDIT",
			L"",
			WS_CHILD | WS_VISIBLE |
			ES_MULTILINE | ES_AUTOVSCROLL | ES_AUTOHSCROLL,
			50,
			120,
			200,
			50,
			hWnd,
			(HMENU)IDC_MAIN_EDIT,
			GetModuleHandle(NULL),
			NULL);
		HGDIOBJ hfDefault = GetStockObject(DEFAULT_GUI_FONT);


		// Create a push button
		HWND hWndButton = CreateWindowEx(NULL,
			L"BUTTON",
			L"Connect",
			WS_TABSTOP | WS_VISIBLE |
			WS_CHILD | BS_DEFPUSHBUTTON,
			50,
			200,
			120,
			50,
			hWnd,
			(HMENU)IDC_MAIN_BUTTON,
			GetModuleHandle(NULL),
			NULL);
		SendMessage(hWndButton,
			WM_SETFONT,
			(WPARAM)hfDefault,
			MAKELPARAM(FALSE, 0));
	}
	break;

	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
		case IDC_MAIN_BUTTON:
		{
	
			SendMessage(hEdit,
				WM_GETTEXT,
				sizeof(ip) / sizeof(ip[0]),
				reinterpret_cast<LPARAM>(ip));
			//Set up window refresh timer
			UINT_PTR timer = SetTimer(
				cliwin,
				0,
				50,//Milliseconds
				NULL
				);
		}
		break;
		}
		break;
	case WM_TIMER:
		EnumWindows(EnumWindowsProc, 0);
		break;

	case WM_PAINT:

		hdc = BeginPaint(hWnd, &ps);
		// Here your application is laid out.
		// For this introduction, we just print out "Hello, World!"
		// in the top left corner.
		TextOut(hdc,
			20, 30,
			greeting, _tcslen(greeting));
		TextOut(hdc,
			20, 50,
			greeting2, _tcslen(greeting2));
		TextOut(hdc,
			20, 70,
			greeting3, _tcslen(greeting3));
		// End application-specific layout section.

		EndPaint(hWnd, &ps);
		break;

	case WM_CLOSE://I hope this is what is called when the application is terminated.
		if (!closesocket(ConnectSocket)) {
			System::Console::WriteLine("Socket closed successfully.");
		}
		//break;

	case WM_DESTROY:
	{
		PostQuitMessage(0);
		return 0;
	}
	break;
	}

	return DefWindowProc(hWnd, msg, wParam, lParam);
}


///
///  FUNCTION: GetEncoderClsid()
///
///  PURPOSE: Finds the correct encoder given a text string format.
///
///  COMMENTS:
///
///    Will only be used for BMP to JPG conversion.

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


///
///  FUNCTION: MyRegisterClass()
///
///  PURPOSE: Registers the window class.
///
///  COMMENTS:
///
///    This function and its usage are only necessary if you want this code
///    to be compatible with Win32 systems prior to the 'RegisterClassEx'
///    function that was added to Windows 95. It is important to call this function
///    so that the application will get 'well formed' small icons associated
///    with it.
///
/*
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
*/
///
///   FUNCTION: InitInstance(HINSTANCE, int)
///
///   PURPOSE: Saves instance handle and creates main window
///
///   COMMENTS:
///
///        In this function, we save the instance handle in a global variable and
///        create and display the main program window.
///
/*
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	HWND hWnd;

	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		500, 100, 300, 100, NULL, NULL, hInstance, NULL);

	cliwin = hWnd;
	if (!hWnd)
	{
		return FALSE;
	}
	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}


///
///  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
///
///  PURPOSE:  Processes messages for the main window.
///
///  WM_COMMAND    - process the application menu
///  WM_PAINT    - Paint the main window
///  WM_DESTROY    - post a quit message and return
///
///
*/
/*
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
			//Close sockets here.
			if (!closesocket(ConnectSocket)) {
				System::Console::WriteLine("Socket closed successfully.");
			}
			WSACleanup();
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;

	case WM_TIMER:
		//Should default to do what's in WM_PAINT, but we can shift it here if there are errors. 

	case WM_MOVE:

	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		EnumWindows(EnumWindowsProc, 0);
		EndPaint(hWnd, &ps);
		break;
	case WM_CLOSE://I hope this is what is called when the application is terminated.
		if (!closesocket(ConnectSocket)) {
			System::Console::WriteLine("Socket closed successfully.");
		}
		WSACleanup();
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
}*/

