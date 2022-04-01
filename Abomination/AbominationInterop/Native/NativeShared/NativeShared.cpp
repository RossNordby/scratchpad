#define PathToLibrary "..\\..\\AbominationInterop\\bin\\Release\\net7.0\\win10-x64\\publish\\AbominationInterop.dll"

#include "windows.h"
#include <stdlib.h>
#include <stdio.h>

void callGreetings(char* path, char* funcName);


int main()
{
	// Sum two integers
	callGreetings((char*)PathToLibrary, (char*)"Goingtr");

}

void callGreetings(char* path, char* funcName)
{
	HINSTANCE handle = LoadLibraryA(path);
	if (handle != NULL)
	{
		typedef int(*myFunc)(void);
		myFunc MyImport = (myFunc)GetProcAddress(handle, funcName);

		MyImport();
	}
}
