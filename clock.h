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
#define ID_TEXT_6    (GUI_ID_USER + 0x08)
#define ID_TEXT_7    (GUI_ID_USER + 0x09)
#define ID_TEXT_8    (GUI_ID_USER + 0x0A)

#define GUI_ID_U   0x850

#define ID_WINDOW_0  (GUI_ID_U + 0x10)
#define ID_TEXT_0    (GUI_ID_U + 0x11)
#define ID_TEXT_1    (GUI_ID_U + 0x12)
#define ID_TEXT_2    (GUI_ID_U + 0x13)
#define ID_TEXT_3    (GUI_ID_U + 0x14)
#define ID_TEXT_4    (GUI_ID_U + 0x15)
#define ID_TEXT_5    (GUI_ID_U + 0x16)

#define GUI_ID_1   0x850

#define ID_WINDOW_0 (GUI_ID_1 + 0x17)
#define ID_BUTTON_0 (GUI_ID_1 + 0x18)
#define ID_BUTTON_1 (GUI_ID_1 + 0x19)
#define ID_BUTTON_2 (GUI_ID_1 + 0x1A)
#define ID_BUTTON_3 (GUI_ID_1 + 0x1B)
#define ID_CHECKBOX_0 (GUI_ID_1 + 0x1C)
#define ID_TEXT_0 (GUI_ID_1 + 0x1D)
#define ID_TEXT_1 (GUI_ID_1 + 0x1E)
#define ID_TEXT_2 (GUI_ID_1 + 0x1F)
#define ID_TEXT_3 (GUI_ID_1 + 0x20)

#define ID_WINDOW_1 (GUI_ID_1 + 0x21)

WM_HWIN Createclock(void);
WM_HWIN Createplan(void);
WM_HWIN CreateAlarm(void);
void Clock( void * pvParameters);

#endif
