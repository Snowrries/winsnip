  // written by: Allen Tung
  // tested by: Allen Tung, Anthony Wong
  // debugged by: Allen Tung, Anthony Wong
  // reviewed by: Yue Yang

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
#define IDC_CANCEL_BUTTON 103		// Cancel button identifier
#define MAX_THREADS 128
HWND hEdit;
INT cancel = 0;

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
// Global Variables:
HINSTANCE hInst;                        // current instance
TCHAR szTitle[MAX_LOADSTRING];          // The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];    // the main window class name
HWND cliwin;
//IStorage* youStorage = NULL; //Currently unnecessary
SOCKET ConnectSocket = INVALID_SOCKET;

HWND	pDataArray[MAX_THREADS];
DWORD   dwThreadIdArray[MAX_THREADS];
windptr* ThreadArray;

/// Forward declarations of functions included in this code module:
//ATOM                MyRegisterClass(HINSTANCE hInstance);
//BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
BOOL CALLBACK		EnumWindowsProc(HWND hWnd, LPARAM lParam);
INT					GetEncoderClsid(const WCHAR* format, CLSID* pClsid);  // helper function

char ip[25] = { 0 };
int buttonpress = 0;
int connected = 0;


///Main function. First argument in command line should be the IP address of the server. 
int APIENTRY _tWinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPTSTR    lpCmdLine,
	int       nCmdShow)
{
	HACCEL hAccelTable;

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

	
	MSG msg;
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GDI_CAPTURINGANIMAGE));
	ThreadArray = (windptr*)malloc(MAX_THREADS * sizeof(windptr));



	while (GetMessage(&msg, NULL, 0, 0) > 0)
	{
		
		TranslateMessage(&msg);
		DispatchMessage(&msg);

	}

	
	return 0;

}
///   FUNCTION: ConnectToServer()
///
///   PURPOSE: Connect to a server given a global IP.
///
///   COMMENTS: 
///
///			Will be called by every single thread to create single sockets for each window.
///			Returns 0 on success, -1 on failure
///			Called on buttonpress == 1 and connected == 0

int ConnectToServer() {

	int iResult; //To take the result of function calls
	WSADATA wsaData; //Sockets.
	struct addrinfo *result = NULL, //For socketing address purposes
		*ptr = NULL,
		hints;
	//First time connection setup
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0) {
			printf("WSAStartup failed with error: %d\n", iResult);
			buttonpress = 0;
			return -1;
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
			buttonpress = 0;
			return -1;
		}
		// Attempt to connect to an address until one succeeds
		for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
			// Create a SOCKET for connecting to server
			ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
				ptr->ai_protocol);
			if (ConnectSocket == INVALID_SOCKET) {
				printf("socket failed with error: %ld\n", WSAGetLastError());
				WSACleanup();
				continue;
			}

			// Connect to server.
			iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
			if (iResult == SOCKET_ERROR) {
				closesocket(ConnectSocket);
				ConnectSocket = INVALID_SOCKET;
				continue;
			}
			printf("Connected to socket at IP: %s", ip);
			connected = 1;
			break;
		}

		freeaddrinfo(result);

		if (ConnectSocket == INVALID_SOCKET) {
			printf("Unable to connect to server!\n");
			WSACleanup();
			return -1;
		}
		buttonpress = 0;
		return 0;

	}

///
///   FUNCTION: CaptureAnImage(HWND active)
///
///   PURPOSE: Captures a screenshot of each window and saves them in .jpg files. Creates iStreams for each window and stores in a compound file.
///
///   COMMENTS: 
///
///      Note: This sample will attempt to create a file with the same title as the window, 
///			and a max limit of approximately 50 chars 
///			If ConnectToServer fails, function will return -1 for failure. 
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
		//MessageBox(hWnd, L"BitBlt has failed", L"Failed", MB_OK);
		int a = GetLastError();
		//For error checking. Some strange error occurs here after long runtime, not sure of the reason. May be server side.
		goto done;
	}
	if (!hbmActive)
	{
		MessageBox(hWnd, L"CreateCompatibleBitmap Failed", L"Failed", MB_OK);
		goto done;
	}
	//CreateDirectory(L"pictures", NULL);
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
	//char *buf;


	//Send name
	//count = 0;
	//size_t r;
	//buf = (char*)malloc(64);
	//wcstombs_s(&r,buf,64,titley,64);
	//const char *len = (char*)strlen(buf);
//	send(ConnectSocket, len, sizeof(len),NULL);

	//while (count < strlen(buf)) {
	//	count += send(ConnectSocket, &buf[count], strlen(buf)-count, NULL);
	//}
	//free(buf);
	/*
	count = 0;
	char* bufx = (char*)malloc(sizeof(hWnd)+1);
	memcpy(&bufx, hWnd, sizeof(hWnd));
	//loop while there is more data:
	//Send hWnd as unique identifier
	while (count < sizeof(hWnd)) {
		count += send(ConnectSocket, bufx, sizeof(hWnd), NULL);
	}
	free(bufx);
	*/
	//Send 2 newlines
	count = 0;
	//while (count < sizeof("\n\n")) {
//		count += send(ConnectSocket, "\n\n", 2, NULL);
//	}
	
	//len = (char*)full.LowPart;
	//send(ConnectSocket, len, sizeof(len), NULL);
	//Send image
	while (count < full.LowPart) {
		//Write to socket and keep track of bytes written in count, update accordingly.

		count += send(ConnectSocket, &buffer[count], full.LowPart, NULL);

	}
	free(buffer);

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
	///   FUNCTION: EnumWindowsProc(HWND hWnd, LPARAM lParam)
	///
	///   PURPOSE: Callback function to enumerate through windows.
	///
	///   COMMENTS:
	///
	///		Create a new thread for each valid window upon a socket will be connected, and a jpeg stream made.
	///		Return value of true to continue enumerating.
	///		Save the handle into the pointer given in HANDLE*. Will be &lParam[appropriateIndex], so no concern is necessary.

	BOOL CALLBACK EnumWindowsProc(HWND hWnd, LPARAM lParam) {
		TCHAR szText[256];
		DWORD WINAPI MyThreadFunction(LPVOID lpParam);
		DWORD ThreadId;
		windptr* pontr = (windptr*)lParam;
		int iof;//instance of free handlespot
		int flag = 0; //Only take first free handlespot

		if (IsWindowVisible(hWnd) && GetWindow(hWnd, GW_OWNER) == NULL) {
			//Visible, has no owners
			if (GetWindowText(hWnd, szText, 256) == 0) // No text in window
				return TRUE;
				//Checking to see if the window has a title bar.

			if (!wcscmp(szText, L"Program Manager")) {
				return TRUE;
			}
			else if (!wcscmp(szText, L"Windows Shell Experience Host")) {
				return TRUE;
			}
			for (int i = 0; i < MAX_THREADS; i++) {
				if (ThreadArray[i].hWnd == hWnd) {
					return TRUE;
				}
				if ( !flag && !IsWindow(ThreadArray[i].hWnd)) {
					iof = i;
					flag = 1;
				}
			}
			flag = 0;
			ThreadArray[iof].hWnd = hWnd;
			ThreadArray[iof].hand = CreateThread(
				NULL,                   // default security attributes
				0,                      // use default stack size  
				MyThreadFunction,       // thread function name
				hWnd,				    // argument to thread function 
				0,                      // use default creation flags 
				&ThreadId);				// returns the thread identifier 
			if (ThreadArray[iof].hand == NULL)
				{return TRUE;}
		}
		
		/*Reset window refresh timer
		UINT_PTR timer = SetTimer(
			cliwin,
			0,
			50,//Milliseconds
			NULL
			);
		*/
		return TRUE;
	}

	///
	///
	///   FUNCTION: MyThreadFunction(LPVOID lpParam)
	///
	///   PURPOSE: Threading function
	///
	///   COMMENTS:
	///			Sets up a new socket connection and streams the window feed. lpParam is a HWND. 
	///			If ConnectToServer fails, function will return -1 for failure. 
	///        



	DWORD WINAPI MyThreadFunction(LPVOID lpParam)
	{
		
		if (ConnectToServer() < 0) {
			return -1;
		}	
		while (!cancel) {
			CaptureAnImage((HWND)lpParam);
			Sleep(50);//Wait 50 milliseconds to refresh
		}
		//System::Threading::Thread::Abort();
		return 0;
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

		// Create a push button
		HWND hWndButtonStop = CreateWindowEx(NULL,
			L"BUTTON",
			L"End",
			WS_TABSTOP | WS_VISIBLE |
			WS_CHILD | BS_DEFPUSHBUTTON,
			200,
			200,
			120,
			50,
			hWnd,
			(HMENU)IDC_CANCEL_BUTTON,
			GetModuleHandle(NULL),
			NULL);
		SendMessage(hWndButtonStop,
			WM_SETFONT,
			(WPARAM)hfDefault,
			MAKELPARAM(FALSE, 0));
		break;
	}


	case WM_COMMAND:
		switch (LOWORD(wParam))
		{
			case IDC_MAIN_BUTTON:
				{
					cancel = 0;
					SendMessage(hEdit,
						WM_GETTEXT,
						sizeof(ip) / sizeof(ip[0]),
						reinterpret_cast<LPARAM>(ip));
					char* buf = "localhost";
					if (ip[0] == '\0') {
						memcpy(ip, buf, 10);// Copy localhost with null byte into ip
					}
					//Set up window refresh timer
					UINT_PTR timer = SetTimer(
						cliwin,
						0,
						50,//Milliseconds
						NULL
						);
					buttonpress = 1;
					//Sets flag so socketing occurs
					break;
				}

			case IDC_CANCEL_BUTTON:
				{
					if (!closesocket(ConnectSocket)) {
						System::Console::WriteLine("Socket closed successfully.");
					}
					connected = 0;
					buttonpress = 0;
					cancel = 1; //Setting this will stop the stream while loop.
					break;
				}
		}

	case WM_TIMER:
		{
		EnumWindows(EnumWindowsProc, 0);
		break;
		}	
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
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

	case WM_CLOSE:
	//I hope this is what is called when the application is terminated.
		if (!closesocket(ConnectSocket)) {
			System::Console::WriteLine("Socket closed successfully.");
		}	
		//break;
	
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	
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




