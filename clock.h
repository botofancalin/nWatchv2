#ifndef _CLOCK_H
#define _CLOCK_H

#include "global_inc.h"

#define ID_WINDOW_0    (GUI_ID_USER + 0x00)
#define ID_TEXT_0    (GUI_ID_USER + 0x02)
#define ID_TEXT_1    (GUI_ID_USER + 0x03)
#define ID_TEXT_2    (GUI_ID_USER + 0x04)
#define ID_TEXT_3    (GUI_ID_USER + 0x05)
#define ID_TEXT_4    (GUI_ID_USER + 0x06)
#define ID_TEXT_5    (GUI_ID_USER + 0x07)

WM_HWIN Createclock(void);
static void _cbDialog(WM_MESSAGE * pMsg);
void Clock( void * pvParameters);

#endif
