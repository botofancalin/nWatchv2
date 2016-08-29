#include <global_inc.h>

inline void cpu_off(void)
{
//	vTaskEndScheduler();
	taskENTER_CRITICAL();
	for(int i=0;i<160;i++)
	{
		IWDG_ReloadCounter();
		LCD_line(i,0,i,240,BLACK,1);
		LCD_line(320-i,0,320-i,240,BLACK,1);
//		GUI_SetColor(GUI_WHITE);
//		GUI_DrawLine(i,0,i,320);
//		GUI_DrawLine(240-i,0,240-i,320);
		backlight(160-i);
		delay(1);
	}

	GPIOB->BSRRH|=GPIO_BSRR_BS_8;
	while(1){IWDG_ReloadCounter();};
	taskEXIT_CRITICAL();
}
