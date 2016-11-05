#include "heading.h"
#include "menu.h"

float last_volt=150;
extern WM_HWIN hWinclock;

static GUI_CONST_STORAGE GUI_COLOR _ColorsfMhGv[] = {
  0xFFFFFF, 0x000000
};

static GUI_CONST_STORAGE GUI_LOGPALETTE _PalfMhGv = {
  2,  // Number of entries
  1,  // Has transparency
  &_ColorsfMhGv[0]
};

static GUI_CONST_STORAGE unsigned char _acfMhGv[] = {
  ________, ________, ________,
  ________, ____XX__, ________,
  ________, ___XXXX_, ________,
  ________, __XXXXX_, ________,
  ________, _XXXXXX_, ________,
  ________, XXXXXX__, ________,
  _______X, XXXXX___, ________,
  ______XX, XXXX____, ________,
  _____XXX, XXX_____, ________,
  ____XXXX, XX______, ________,
  ___XXXXX, X_______, ________,
  __XXXXXX, ________, ________,
  _XXXXXXX, ________, ________,
  __XXXXXX, X_______, ________,
  ___XXXXX, X_______, ________,
  ____XXXX, XX______, ________,
  _____XXX, XXX_____, ________,
  ______XX, XXXX____, ________,
  _______X, XXXXX___, ________,
  ________, XXXXXX__, ________,
  ________, _XXXXXX_, ________,
  ________, __XXXXX_, ________,
  ________, ___XXX__, ________,
  ________, ____X___, ________,
  ________, ________, ________
};

GUI_CONST_STORAGE GUI_BITMAP bmfMhGv = {
  19, // xSize
  25, // ySize
  3, // BytesPerLine
  1, // BitsPerPixel
  _acfMhGv,  // Pointer to picture data (indices)
  &_PalfMhGv   // Pointer to palette
};


static const GUI_WIDGET_CREATE_INFO _aDialogCreateb[] =
{
  { WINDOW_CreateIndirect, "bar", ID_WINDOW_0, 0, 0, 320, 25,0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Button", ID_BUTTON_0, 0, 0, 54, 25, 0, 0x0, 0 },
  { IMAGE_CreateIndirect, "Image", ID_IMAGE_0, 280, 4, 27, 14, 0, 0, 0 },
  { IMAGE_CreateIndirect, "Image", ID_IMAGE_1, 120, 0, 30, 25, 0, 0, 0 },
  { IMAGE_CreateIndirect, "Image", ID_IMAGE_2, 180, 0, 30, 25, 0, 0, 0 },
  { TEXT_CreateIndirect, "Text", ID_TEXT_0, 260, 4, 80, 20, 0, 0x64, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

static int _ButtonSkin(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo)
{
  int Index, xSize, ySize, IsPressed;
  WM_HWIN hWin;
  const GUI_BITMAP * pBm;
  GUI_COLOR Color;

  hWin = pDrawItemInfo->hWin;
  switch (pDrawItemInfo->Cmd)
  {
	  case WIDGET_ITEM_DRAW_BACKGROUND:
	  {
		  if( Menu_Handle == NULL )
		  {
			  IsPressed = BUTTON_IsPressed(pDrawItemInfo->hWin);
			  if (IsPressed)
			  {
				  Color = GUI_BLUE | 0xC0000000;
				  GUI_SetBkColor(Color);
				  GUI_Clear();
			  }
			  	  GUI_DrawBitmap(&bmfMhGv, 0,0);
		   }
		break;
	  }
  }
  return 0;
}



static void _cbDialogb(WM_MESSAGE * pMsg)
{
  const void * pData;
  WM_HWIN      hItem;
  U32          FileSize;
  int  Id, NCode;
  static int IsPressed;

  switch (pMsg->MsgId)
  {
  case WM_NOTIFY_PARENT:
  {
      Id    = WM_GetId(pMsg->hWinSrc);      // Id of widget
      NCode = pMsg->Data.v;                 // Notification code
  		switch (NCode)
  		{
  		case WM_NOTIFICATION_CLICKED:
  		  IsPressed = 1;
  		  break;
  		case WM_NOTIFICATION_RELEASED:
  		  if (IsPressed)
  		  {
//  			xTaskCreate(Menu,(char const*)"Menu",1024,NULL,7, &Menu_Handle);
  			IsPressed = 0;
  		  }
  		  break;
  		case WM_NOTIFICATION_MOVED_OUT:
  	//        IsPressed = 0;
  		  break;
  		}
  		break;
  }
  case WM_INIT_DIALOG:
  {

	hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE_0);
	IMAGE_SetBitmap(hItem,&_bmBatteryEmpty_27x14);
//
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
    TEXT_SetFont(hItem, &GUI_Font16_ASCII);
    TEXT_SetText(hItem, "");

    break;
  }
  case WM_PAINT:
  {
	  taskENTER_CRITICAL();
	  LCD_BMP("0:dzien.bmp");
	  taskEXIT_CRITICAL();

	    ADC_SoftwareStartConv(ADC1);
	    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	    float result= ADC_GetConversionValue(ADC1);
	    float volt = result /3070 * 100;  //240

	    if(volt<last_volt)
	    {
	    	last_volt=volt;
	    }
//		volt-=140;
//	    volt=(volt/41)*100;

	    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
	    TEXT_SetText(hItem,itoa(last_volt,t,10));

	    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
	    BUTTON_SetSkin(hItem, _ButtonSkin);
//	    GUI_DrawBitmap(&bmArrowRigth_15x24, 5, 0);

	    if(last_volt>=80 && last_volt <=100)
		{
			GUI_SetColor(GUI_GREEN);
			GUI_FillRect(284,6,304,15);
		}
		else if(last_volt>=60 && last_volt <80)
		{
			GUI_SetColor(0x0011ee9e);
			GUI_FillRect(288,6,304,15);
		}
		else if(last_volt>=40 && last_volt <60)
		{
			GUI_SetColor(0x0011eeec);
			GUI_FillRect(292,6,304,15);
		}
		else if(last_volt>=20 && last_volt <40)
		{
			GUI_SetColor(0x00118cee);
			GUI_FillRect(296,6,304,15);
		}
		else if(last_volt<20)
		{
			GUI_SetColor(GUI_RED);
			GUI_FillRect(300,6,304,15);
		}

	  break;
  }
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}


WM_HWIN Createbar(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreateb, GUI_COUNTOF(_aDialogCreateb), _cbDialogb, 0, 0, 0);
  return hWin;
}

void Heading_Task( void * pvParameters)
{
	WM_HWIN bar=Createbar();

	while(1)
	{
		WM_HWIN hButton = WM_GetDialogItem(bar, ID_BUTTON_0);
		if(BUTTON_IsPressed(hButton))
		{
			WM_DeleteWindow(hWinclock);
			xTaskCreate(Menu,(char const*)"Menu",1024,NULL,7, &Menu_Handle);
		}
		vTaskDelay(50);
		WM_InvalidateWindow(bar);
		WM_BringToTop(bar);
//		GUI_Exec();
	}
}
