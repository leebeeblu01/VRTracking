#include "stdafx.h"
#include "ViveTracking.h"
#pragma comment(lib, "openvr_api")

//Windows keyboard input
#include <conio.h>

int _tmain(int argc, _TCHAR* argv[])
{
	ViveTracking *viveTracking = new ViveTracking();

	if (viveTracking)
	{
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Press 'q' to quit\n");
		printf_s(buf);

		while (viveTracking->RunProcedure())
		{
			// Windows quit routine 
			if (_kbhit())
			{
				char ch = _getch();
				if ('q' == ch)
				{
					char buf[1024];
					sprintf_s(buf, sizeof(buf), "User pressed 'q' - exiting...");
					printf_s(buf);
					Sleep(500);
					break;
				}
				
				if ('r' == ch) {
					vr::VRSystem()->ResetSeatedZeroPose();
					char buf[1024];
					sprintf_s(buf, sizeof(buf), "User pressed 'r' - reseting...");
					printf_s(buf);
					Sleep(500);
					continue;
				}
				
			}
			
		}
		delete viveTracking;
	}
	return 0;
}