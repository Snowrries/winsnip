#pragma once

#include "resource.h"
struct winpacket {
	HWND winhand;
	//string name

};

struct HANDLEstrt
{
	HANDLE hand;
	HWND hWnd;
};
typedef struct HANDLEstrt windptr;