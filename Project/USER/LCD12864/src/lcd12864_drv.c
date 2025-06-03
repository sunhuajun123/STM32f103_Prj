#include "lcd12864_drv.h"
#include "lcd12864_font.h"
#include "MC_Type.h"


unsigned char ucLcdBuffer[8][128];
unsigned short ClockDir;//˳ʱ��ʱ��Ϊ1
uint8_t menu = 0;
uint8_t fgDispMode = 0,fgLcdCS1 = 0;
NumToArrayBufDef N2ABuf;
TYPE_BYTE tmpbyte;
/****************************************************************************** 
* Function Name    	: LCD12864_HardwareInit
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864����IOӲ����ʼ��
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void LCD12864_HardwareInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |\
      RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN | LCD_EN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_DB7_PIN | LCD_CS1_PIN | LCD_CS2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_DB1_PIN | LCD_DB2_PIN | LCD_DB3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = LCD_DB4_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LCD_DB0_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LCD_DB5_PIN | LCD_DB6_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/****************************************************************************** 
* Function Name    	: WriteDataLCD
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,д����
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void WriteDataLCD(unsigned char WDLCD, unsigned char select)
{
    switch(select)
    {
        /*���Ұ����ѡ��*/
        case 0:{LCDCS1_ON(); LCDCS2_OFF(); break;}
        case 1:{LCDCS1_OFF(); LCDCS2_ON(); break;}
        default: break;
    }
    LCDEN_OFF();            /*�رղ���ͨ�Žӿ�*/
    LCDRS_ON();             /*д����ʹ��*/
    tmpbyte.byte = WDLCD;   /*����ӳ�䵽byte�ṹ*/
    if (tmpbyte.bit.b0){LCDDB0_ON();}else{LCDDB0_OFF();}/*��Ҫд������ӳ�䵽�������ݽӿ�*/
    if (tmpbyte.bit.b1){LCDDB1_ON();}else{LCDDB1_OFF();}
    if (tmpbyte.bit.b2){LCDDB2_ON();}else{LCDDB2_OFF();}
    if (tmpbyte.bit.b3){LCDDB3_ON();}else{LCDDB3_OFF();}
    if (tmpbyte.bit.b4){LCDDB4_ON();}else{LCDDB4_OFF();}
    if (tmpbyte.bit.b5){LCDDB5_ON();}else{LCDDB5_OFF();}
    if (tmpbyte.bit.b6){LCDDB6_ON();}else{LCDDB6_OFF();}
    if (tmpbyte.bit.b7){LCDDB7_ON();}else{LCDDB7_OFF();}
    
    Delay1Us();
    Delay1Us();
    LCDEN_ON();
    Delay1Us();
    LCDEN_OFF();/*E�ź��½������沢������*/
}

/****************************************************************************** 
* Function Name    	: WriteCommandLCD
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,дָ��
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void WriteCommandLCD(unsigned char WDLCD, unsigned char select)
{
    switch(select)
    {
        /*���Ұ����ѡ��*/
        case 0:{LCDCS1_ON(); LCDCS2_OFF(); break;}/*��*/
        case 1:{LCDCS1_OFF(); LCDCS2_ON(); break;}/*��*/
        default: break;
    }
    LCDEN_OFF();            /*�رղ���ͨ�Žӿ�*/
    LCDRS_OFF();            /*дָ��ʹ��*/
    tmpbyte.byte = WDLCD;
    if (tmpbyte.bit.b0){LCDDB0_ON();}else{LCDDB0_OFF();}/*��Ҫд������ӳ�䵽�������ݽӿ�*/
    if (tmpbyte.bit.b1){LCDDB1_ON();}else{LCDDB1_OFF();}
    if (tmpbyte.bit.b2){LCDDB2_ON();}else{LCDDB2_OFF();}
    if (tmpbyte.bit.b3){LCDDB3_ON();}else{LCDDB3_OFF();}
    if (tmpbyte.bit.b4){LCDDB4_ON();}else{LCDDB4_OFF();}
    if (tmpbyte.bit.b5){LCDDB5_ON();}else{LCDDB5_OFF();}
    if (tmpbyte.bit.b6){LCDDB6_ON();}else{LCDDB6_OFF();}
    if (tmpbyte.bit.b7){LCDDB7_ON();}else{LCDDB7_OFF();}
    
    Delay1Us();
    Delay1Us();
    /*E�����½���д�벢������*/
    LCDEN_ON();
    Delay1Us();
    LCDEN_OFF();
}

/****************************************************************************** 
* Function Name    	: ClearLCD
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,��������DDRAMд��
* Input Param    	: select��������ѡ��   
* Output Param    	: void
*******************************************************************************/ 
void ClearLCD(unsigned char select)
{
    unsigned char i=0,j=0; 
    for (i = 0; i < 8; i++)
    {
        if (fgDispMode == 0)
        {
            WriteCommandLCD(LCDPageSet | i, select);    //����ҳ��ַ��8��Ϊ1ҳ����8ҳ
            WriteCommandLCD(LCDColumnAddrSet, select);  //DDRAM��Y��ַָ�룬��DDRAM���ж�д������Y��ַָ���Զ���1��ָ����һ��DDRAM��Ԫ
        }
        for (j = 0; j < 64; j++)
        {
            if (fgDispMode == 0)
            {
                WriteDataLCD(0, select);                //�ӵ�0ҳ��0�п�ʼ��DDRAM��д0������
            }
            ucLcdBuffer[i][select*64+j] = 0;            //LCDBUFFER��0
        }
    }
}

/****************************************************************************** 
* Function Name    	: ClearAllLcd
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,����������
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void ClearAllLcd(void)
{
    ClearLCD(LCDR);
    ClearLCD(LCDL);
}

/****************************************************************************** 
* Function Name    	: DispLCDBuf
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,��ucLcdBuffer����д��DDRAM����ʾ
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void DispLCDBuf(unsigned char select)
{
    unsigned char i,j;
    for (i = 0; i < 8; i++)
    {
        if (fgDispMode == 0)
        {
            WriteCommandLCD(LCDPageSet | i, select);
            WriteCommandLCD(LCDColumnAddrSet, select);
        }
        for (j = 0; j < 64; j++)
        {
            WriteDataLCD(ucLcdBuffer[i][select*64+j], select);
        }
    }
}

/****************************************************************************** 
* Function Name    	: ShowDispBuf
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,��ucLcdBuffer����д��������
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void ShowDispBuf(void)
{
    DispLCDBuf(LCDR);
    DispLCDBuf(LCDL);
}

/****************************************************************************** 
* Function Name    	: LcdReset
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,LCD��ʼ��
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void LcdReset(void)
{
    LCDRS_ON();                                 //��������ָ��
    WriteCommandLCD(LCDStartRowAddrSet, LCDR);  //�ұ�����ʾ��ʼ�����ã�DDRAM��0�ж�Ӧ��Ļ�ĵ�1��
    WriteCommandLCD(LCDStartRowAddrSet, LCDL);  //�������ʾ��ʼ�����ã�DDRAM��0�ж�Ӧ��Ļ�ĵ�1��
    ClearAllLcd();                              //����
    
    WriteCommandLCD(LCDPageSet, LCDL);          //�����ҳ��ʾ����
    WriteCommandLCD(LCDColumnAddrSet, LCDL);    //���������ʾ����
    WriteCommandLCD(LCDPageSet, LCDR);          //�ұ���ҳ��ʾ����
    WriteCommandLCD(LCDColumnAddrSet, LCDR);    //�ұ�������ʾ����
    WriteCommandLCD(LCDDispOn, LCDL);           //�������ʾ��
    WriteCommandLCD(LCDDispOn, LCDR);           //�ұ�����ʾ��
}

/****************************************************************************** 
* Function Name    	: SetXY
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,���в���λ������
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void SetXY(unsigned char Cloumn, unsigned char Page)
{
    unsigned char x,y,select=0;
   
    switch(Cloumn & 0x40)
    {
        case 0x00: {select = 0; fgLcdCS1 = 0; break;}//�����
        case 0x40: {select = 1; fgLcdCS1 = 1; break;}//�ұ���
    }
    y = Cloumn&0x3f|0x40;       //������ָ��
    x = Page&0x07|0xb8;         //ҳ����ָ��
    WriteCommandLCD(x, select); //����ҳ����ָ��
    WriteCommandLCD(y, select); //����������ָ��
    WriteCommandLCD(LCDStartRowAddrSet, select);//��ʼ�����ã�DDRAM��0�ж�Ӧ��Ļ��1��
}

/****************************************************************************** 
* Function Name    	: ShowBlank
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD12864,��ʾ�ո�
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void ShowBlank(unsigned char width,unsigned char row,unsigned char line)
{
    unsigned char i;
	if (fgDispMode==0)
    {
        SetXY(line,row);
    }
	for(i=0;i<width;i++) 
	{	
		if (fgDispMode==0)
		{
			if (fgLcdCS1==LCDL) 
			{	
                if (line+i>=64) 
				{	
                    SetXY(line+i,row);
				}
			}
			WriteDataLCD(0,fgLcdCS1);
		}
		ucLcdBuffer[row][line+i]=0;		
	}

}

/****************************************************************************** 
* Function Name    	: NumToArray
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: ���з����������ֲ�ֺ��������
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void NumToArray(signed int number32)
{
	unsigned int num;
	if(number32<0)
	{
		N2ABuf.Fuhao=0x01;
		num=(-1)*number32;
	}
	else
	{
		N2ABuf.Fuhao=0x00;
		num=number32;
	}
	N2ABuf.Buf[0]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[1]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[2]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[3]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[4]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[5]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[6]=(unsigned char)(num%10);
	num/=10;
	N2ABuf.Buf[7]=(unsigned char)(num%10);
	N2ABuf.Buf[8]=(unsigned char)(num/10);
	
}

/****************************************************************************** 
* Function Name    	: ShowChar
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: �ַ���ʾ
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void ShowChar(const unsigned char* CHI, unsigned char width, unsigned char page, unsigned char line)
{
    unsigned char i=0;
    if (!fgDispMode)
    {
        SetXY(line, page);        
    }
    for (i = 0; i < width; i++)
    {
        if (!fgDispMode)
        {
            if (fgLcdCS1 == LCDL)
            {
                if (line+i >= 64)
                {
                    SetXY(line+i, page);
                }
            }
            WriteDataLCD(*(CHI+i), fgLcdCS1);
        }
        ucLcdBuffer[page][line+i] = *(CHI+i);
    }
}

/****************************************************************************** 
* Function Name    	: ShowNum
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: ������ʾ
* Input Param    	: number32:Ҫ��ʾ�����֣�numCnt�����ֵĸ�����point���Ƿ���С��
*                     �㣬row��ҳ��line����   
* Output Param    	: void
*******************************************************************************/ 
void ShowNum(signed int number32, unsigned char numCnt, unsigned char point, unsigned char row ,unsigned char line)
{
	unsigned int temp32;
	unsigned char num0,num1,num2,num3,num4,num5,num6,num7,num8;
	//�����ԭ������ʾ��ֵ��ռ�Ŀռ�
	if(point>0)
	{
        if(number32<0)
        {
            //����Ҫ��ʾ�Ĳ���Ϊ����ʱ�����ʱӦ��С����͸�����ռ����Ҳ���ǽ�ȥ
            ShowBlank((numCnt*6+1*3+6+1),row,(line-numCnt*6-1*3-6));
        }
        else
        {
            //������С��λʱ�����ʱӦ��С������ռ����Ҳ���ǽ�ȥ
            ShowBlank((numCnt*6+1*3+1),row,(line-numCnt*6-1*3));
        }		
	}
	else
	{
        if(number32<0)
        {
            //����Ҫ��ʾ�Ĳ���Ϊ����ʱ�����ʱӦ�Ѹ�����ռ����Ҳ���ǽ�ȥ
            ShowBlank((numCnt*6+6+1),row,(line-numCnt*6-6));
        }
        else
        {
            ShowBlank((numCnt*6+1),row,(line-numCnt*6));
        }		
	}
	
	NumToArray(number32);  //������ת��Ϊ�ַ����飬ͬʱҲ�Ǹ���ȫ�ֱ���N2ABuf.Buf[]
	num0=N2ABuf.Buf[0];
	num1=N2ABuf.Buf[1];
	num2=N2ABuf.Buf[2];
	num3=N2ABuf.Buf[3];
	num4=N2ABuf.Buf[4];
	num5=N2ABuf.Buf[5];
	num6=N2ABuf.Buf[6];
	num7=N2ABuf.Buf[7];
	num8=N2ABuf.Buf[8];
	if (number32<0) 
            temp32=number32*(-1);
	else 
            temp32=number32;

	switch(point)
	{
        case 0:
            ShowChar(&number8x5[num0][0],5,row,(line-1*6+1));
            if((temp32>9)&&(numCnt>1))
                ShowChar(&number8x5[num1][0],5,row,(line-2*6+1));
            if((temp32>99)&&(numCnt>2))
                ShowChar(&number8x5[num2][0],5,row,(line-3*6+1));
            if((temp32>999)&&(numCnt>3))
                ShowChar(&number8x5[num3][0],5,row,(line-4*6+1));
            if((temp32>9999)&&(numCnt>4))
                ShowChar(&number8x5[num4][0],5,row,(line-5*6+1));
            if((temp32>99999)&&(numCnt>5))
                ShowChar(&number8x5[num4][0],5,row,(line-6*6+1));
            if((temp32>999999)&&(numCnt>6))
                ShowChar(&number8x5[num4][0],5,row,(line-7*6+1));
            if(number32<0)
            {
                if((temp32>999999)&&(numCnt>6))
                    ShowChar(&sign_fu[0],5,row,(line-8*6+1));
                else if((temp32>99999)&&(numCnt>5))
                    ShowChar(&sign_fu[0],5,row,(line-7*6+1));
                else if((temp32>9999)&&(numCnt>4))
                    ShowChar(&sign_fu[0],5,row,(line-6*6+1));
                else if((temp32>999)&&(numCnt>3))
                    ShowChar(&sign_fu[0],5,row,(line-5*6+1));
                else if((temp32>99)&&(numCnt>2))
                    ShowChar(&sign_fu[0],5,row,(line-4*6+1));
                else if((temp32>9)&&(numCnt>1))
                    ShowChar(&sign_fu[0],5,row,(line-3*6+1));
                else
                    ShowChar(&sign_fu[0],5,row,(line-2*6+1));
            }
		break;
		case 1:
                    ShowChar(&number8x5[num0][0],5,row,(line-1*6+1));
                    ShowChar(&sign_dot[0],2,row,line-1*6-1*3+1);
                    ShowChar(&number8x5[num1][0],5,row,(line-2*6-1*3+1));
                    if((temp32>99)&&(numCnt>2))
			ShowChar(&number8x5[num2][0],5,row,(line-3*6-1*3+1));
                    if((temp32>999)&&(numCnt>3))
			ShowChar(&number8x5[num3][0],5,row,(line-4*6-1*3+1));
                    if((temp32>9999)&&(numCnt>4))
			ShowChar(&number8x5[num4][0],5,row,(line-5*6-1*3+1));
                    if((temp32>99999)&&(numCnt>5))
			ShowChar(&number8x5[num4][0],5,row,(line-6*6-1*3+1));
                    if((temp32>999999)&&(numCnt>6))
			ShowChar(&number8x5[num4][0],5,row,(line-7*6-1*3+1));
                    if(number32<0)
                    {
			if((temp32>999999)&&(numCnt>6))
                            ShowChar(&sign_fu[0],5,row,(line-8*6-1*3+1));
			else if((temp32>99999)&&(numCnt>5))
                            ShowChar(&sign_fu[0],5,row,(line-7*6-1*3+1));
			else if((temp32>9999)&&(numCnt>4))
                            ShowChar(&sign_fu[0],5,row,(line-6*6-1*3+1));
			else if((temp32>999)&&(numCnt>3))
                            ShowChar(&sign_fu[0],5,row,(line-5*6-1*3+1));
			else if((temp32>99)&&(numCnt>2))
                            ShowChar(&sign_fu[0],5,row,(line-4*6-1*3+1));
			else 
                            ShowChar(&sign_fu[0],5,row,(line-3*6-1*3+1));	
                    }
                    break;
		case 2:
			ShowChar(&number8x5[num0][0],5,row,(line-1*6+1));
			ShowChar(&number8x5[num1][0],5,row,(line-2*6+1));
			ShowChar(&sign_dot[0],2,row,line-2*6-1*3+1);
			ShowChar(&number8x5[num2][0],5,row,(line-3*6-1*3+1));
			if((temp32>999)&&(numCnt>3))
                            ShowChar(&number8x5[num3][0],5,row,(line-4*6-1*3+1));
			if((temp32>9999)&&(numCnt>4))
                            ShowChar(&number8x5[num4][0],5,row,(line-5*6-1*3+1));
			if((temp32>99999)&&(numCnt>5))
                            ShowChar(&number8x5[num5][0],5,row,(line-6*6-1*3+1));
			if((temp32>999999)&&(numCnt>6))
                            ShowChar(&number8x5[num6][0],5,row,(line-7*6-1*3+1));
			if(number32<0)
			{
				if((temp32>999999)&&(numCnt>6))
					ShowChar(&sign_fu[0],5,row,(line-8*6-1*3+1));
				else if((temp32>99999)&&(numCnt>5))
					ShowChar(&sign_fu[0],5,row,(line-7*6-1*3+1));
				else if((temp32>9999)&&(numCnt>4))
					ShowChar(&sign_fu[0],5,row,(line-6*6-1*3+1));
				else if((temp32>999)&&(numCnt>3))
					ShowChar(&sign_fu[0],5,row,(line-5*6-1*3+1));
				else
					ShowChar(&sign_fu[0],5,row,(line-4*6-1*3+1));
			}
			break;
		case 3:
			ShowChar(&number8x5[num0][0],5,row,(line-1*6+1));
			ShowChar(&number8x5[num1][0],5,row,(line-2*6+1));
			ShowChar(&number8x5[num2][0],5,row,(line-3*6+1));
			ShowChar(&sign_dot[0],2,row,line-3*6-1*3+1);
			ShowChar(&number8x5[num3][0],5,row,(line-4*6-1*3+1));
			if((temp32>9999)&&(numCnt>4))
				ShowChar(&number8x5[num4][0],5,row,(line-5*6-1*3+1));
			if((temp32>99999)&&(numCnt>5))
				ShowChar(&number8x5[num5][0],5,row,(line-6*6-1*3+1));
			if((temp32>999999)&&(numCnt>6))
				ShowChar(&number8x5[num6][0],5,row,(line-7*6-1*3+1));
			if((temp32>9999999)&&(numCnt>7))
				ShowChar(&number8x5[num7][0],5,row,(line-8*6-1*3+1));
			if((temp32>99999999)&&(numCnt>8))
				ShowChar(&number8x5[num8][0],5,row,(line-8*6-1*3+1));
			if(number32<0)
			{
				if((temp32>9999999)&&(numCnt>7))
					ShowChar(&sign_fu[0],5,row,(line-9*6-1*3+1));
				else if((temp32>999999)&&(numCnt>6))
					ShowChar(&sign_fu[0],5,row,(line-8*6-1*3+1));
				else if((temp32>99999)&&(numCnt>5))
					ShowChar(&sign_fu[0],5,row,(line-7*6-1*3+1));
				else if((temp32>9999)&&(numCnt>4))
					ShowChar(&sign_fu[0],5,row,(line-6*6-1*3+1));				
				else
					ShowChar(&sign_fu[0],5,row,(line-6*6-1*3+1));
			}
			break;
		default:
			break;
	}
}

/****************************************************************************** 
* Function Name    	: LCD_ShowMenuMain
* Modify/Creat time	: 2025/05/07
* Modified/Creat By	: shj
* Description    	: LCD��ʾ���˵�
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void LCD_ShowMenuMain(void)
{
    if (fgDispMode)
    {
        ClearAllLcd();
        ShowChar(&Eng_P[0],5,0,0);
        ShowChar(&Eng_M[0],5,0,6);
        ShowChar(&Eng_S[0],5,0,6*2);
        ShowChar(&Eng_M[0],5,0,6*3);
        ShowChar(&sign_xinghao[0],5,0,6*4);  
        ShowChar(&Eng_B[0],5,0,6*5);
        ShowChar(&Eng_L[0],5,0,6*6);
        ShowChar(&Eng_D[0],5,0,6*7);
        ShowChar(&Eng_C[0],5,0,6*8);
        ShowChar(&sign_maohao[0],3,0,6*9);
        
        if(ClockDir==0)	
        {
            ShowChar(&Eng_C[0],5,0,6*12);   //clockwise anticlockwise
            ShowChar(&Eng_l[0],5,0,6*13);
            ShowChar(&Eng_o[0],5,0,6*14);
            ShowChar(&Eng_c[0],5,0,6*15);
            ShowChar(&Eng_k[0],5,0,6*16);
            ShowChar(&Eng_W[0],5,0,6*17);
            ShowChar(&Eng_i[0],5,0,6*18);
            ShowChar(&Eng_s[0],5,0,6*19);
            ShowChar(&Eng_e[0],5,0,6*20);
        }
        else
        {
            ShowChar(&Eng_A[0],5,0,6*12);   //clockwise anticlockwise
            ShowChar(&Eng_n[0],5,0,6*13);
            ShowChar(&Eng_t[0],5,0,6*14);
            ShowChar(&Eng_i[0],5,0,6*15);
            ShowChar(&Eng_C[0],5,0,6*16);
            ShowChar(&Eng_l[0],5,0,6*17);
            ShowChar(&Eng_o[0],5,0,6*18);
            ShowChar(&Eng_c[0],5,0,6*19);
            ShowChar(&Eng_k[0],5,0,6*20);	
        }
        ShowChar(&Eng_V[0],5,1,0);  
        ShowChar(&Eng_o[0],5,1,6);
        ShowChar(&Eng_l[0],5,1,6*2);
        ShowChar(&Eng_t[0],5,1,6*3);
        ShowChar(&Eng_a[0],5,1,6*4);
        ShowChar(&Eng_g[0],5,1,6*5);	
        ShowChar(&Eng_e[0],5,1,6*6);
        ShowChar(&sign_maohao[0],3,1,6*7);
        ShowChar(&sign_zheng[0],5,1,6*14);
        ShowNum(24,2,0,1,6*17);
        ShowChar(&Eng_V[0],5,1,6*18);
		
        ShowChar(&Eng_S[0],5,2,0);
        ShowChar(&Eng_E[0],5,2,6);
        ShowChar(&Eng_T[0],5,2,6*2);
        ShowChar(&Eng_S[0],5,2,6*4);
        ShowChar(&Eng_P[0],5,2,6*5);
        ShowChar(&Eng_D[0],5,2,6*6);
        ShowChar(&sign_maohao[0],3,2,6*7);
    //	ShowNum(hSpeed_Reference * 6,5,0,2,6*17);
        ShowChar(&Eng_R[0],5,2,6*18);
        ShowChar(&Eng_P[0],5,2,6*19);
        ShowChar(&Eng_M[0],5,2,6*20);
        
        ShowChar(&Eng_C[0],5,3,0);
        ShowChar(&Eng_U[0],5,3,6);
        ShowChar(&Eng_R[0],5,3,6*2);
        ShowChar(&Eng_S[0],5,3,6*4);
        ShowChar(&Eng_P[0],5,3,6*5);
        ShowChar(&Eng_D[0],5,3,6*6);
        ShowChar(&sign_maohao[0],3,3,6*7);
    //	ShowNum((s16)(ENC_Get_Mechanical_Speed() * 6),5,0,3,6*17);
        ShowChar(&Eng_R[0],5,3,6*18);
        ShowChar(&Eng_P[0],5,3,6*19);
        ShowChar(&Eng_M[0],5,3,6*20);

        ShowChar(&Eng_T[0],5,4,0);
        ShowChar(&Eng_O[0],5,4,6);
        ShowChar(&Eng_R[0],5,4,6*2);
        ShowChar(&Eng_Q[0],5,4,6*3);
        ShowChar(&Eng_U[0],5,4,6*4);
        ShowChar(&Eng_E[0],5,4,6*5);
        ShowChar(&sign_maohao[0],3,4,6*6);

        ShowNum(pFOC_ctr_handle.hTorque_Reference,5,0,4,6*16);
        ShowChar(&Eng_s[0],5,4,6*18);
        ShowNum(16,2,0,4,6*21);

        ShowChar(&Eng_A[0],5,5,0);
        ShowChar(&Eng_l[0],5,5,6);
        ShowChar(&Eng_f[0],5,5,6*2);
        ShowChar(&Eng_a[0],5,5,6*3);
        ShowChar(&Eng_V[0],5,5,6*5);
        ShowChar(&Eng_o[0],5,5,6*6);
        ShowChar(&Eng_l[0],5,5,6*7);
        ShowChar(&Eng_t[0],5,5,6*8);
        ShowChar(&sign_maohao[0],3,5,6*9);
        ShowNum(pFOC_ctr_handle.Stat_Volt_qd.q,5,0,5,6*17);
        ShowChar(&Eng_s[0],5,5,6*18);
        ShowNum(16,2,0,5,6*21);

        ShowChar(&Eng_B[0],5,6,0);
        ShowChar(&Eng_e[0],5,6,6);
        ShowChar(&Eng_t[0],5,6,6*2);
        ShowChar(&Eng_a[0],5,6,6*3);
        ShowChar(&Eng_V[0],5,6,6*5);
        ShowChar(&Eng_o[0],5,6,6*6);
        ShowChar(&Eng_l[0],5,6,6*7);
        ShowChar(&Eng_t[0],5,6,6*8);
        ShowChar(&sign_maohao[0],3,6,6*9);
        ShowNum(pFOC_ctr_handle.Stat_Volt_qd.d,5,0,6,6*17);
        ShowChar(&Eng_s[0],5,6,6*18);
        ShowNum(16,2,0,6,6*21);
	
        if(menu==0)
        {	
            ShowChar(&sign_sanjiao[0],4,0,6*10);
            
            if(ClockDir==0)	
            {
                ShowChar(&Eng_C[0],5,0,6*12);   //clockwise anticlockwise
                ShowChar(&Eng_l[0],5,0,6*13);
                ShowChar(&Eng_o[0],5,0,6*14);
                ShowChar(&Eng_c[0],5,0,6*15);
                ShowChar(&Eng_k[0],5,0,6*16);
                ShowChar(&Eng_W[0],5,0,6*17);
                ShowChar(&Eng_i[0],5,0,6*18);
                ShowChar(&Eng_s[0],5,0,6*19);
                ShowChar(&Eng_e[0],5,0,6*20);
            }
            else
            {
                ShowChar(&Eng_A[0],5,0,6*12);   //clockwise anticlockwise
                ShowChar(&Eng_n[0],5,0,6*13);
                ShowChar(&Eng_t[0],5,0,6*14);
                ShowChar(&Eng_i[0],5,0,6*15);
                ShowChar(&Eng_C[0],5,0,6*16);
                ShowChar(&Eng_l[0],5,0,6*17);
                ShowChar(&Eng_o[0],5,0,6*18);
                ShowChar(&Eng_c[0],5,0,6*19);
                ShowChar(&Eng_k[0],5,0,6*20);	
            }	
        }
        if(menu==1)ShowChar(&sign_sanjiao[0],4,1,6*10);
        if(menu==2)ShowChar(&sign_sanjiao[0],4,2,6*10);
        if(menu==3)ShowChar(&sign_sanjiao[0],4,3,6*10);
        if(menu==4)ShowChar(&sign_sanjiao[0],4,4,6*10);
        if(menu==5)ShowChar(&sign_sanjiao[0],4,5,6*10);
        if(menu==6)ShowChar(&sign_sanjiao[0],4,6,6*10);
	
        fgDispMode =0;
        ShowDispBuf();
    }
    else
    {
        ShowNum(/*hSpeed_Reference*/0 * 6,4,0,2,6*17);
	    ShowNum(/*(s16)(ENC_Get_Mechanical_Speed()*/0 * 6,4,0,3,6*17);
        ShowNum(pFOC_ctr_handle.hTorque_Reference,5,0,4,6*17);
        ShowNum(pFOC_ctr_handle.Stat_Volt_qd.q,5,0,5,6*17);
        ShowNum(pFOC_ctr_handle.Stat_Volt_qd.d,5,0,6,6*17);
	
//        ShowChar(&Eng_T[0],5,7,6*1);
//        ShowChar(&Eng_A[0],5,7,6*2);
//        ShowChar(&Eng_N[0],5,7,6*3);
//        ShowChar(&Eng_K[0],5,7,6*4);
//        ShowChar(&Eng_O[0],5,7,6*5);
//        ShowChar(&Eng_R[0],5,7,6*6);
//	
//        ShowChar(&Eng_E[0],5,7,6*8);
//        ShowChar(&Eng_n[0],5,7,6*9);			 			
//        ShowChar(&Eng_c[0],5,7,6*10);
//        ShowChar(&Eng_o[0],5,7,6*11);
//        ShowChar(&Eng_d[0],5,7,6*12);
//        ShowChar(&Eng_e[0],5,7,6*13);
//        ShowChar(&Eng_r[0],5,7,6*14);			
//	
//        ShowChar(&sign_jiankuo[0],5,7,6*15); 
//        ShowChar(&sign_jiankuo[0],5,7,6*16); 
//        ShowChar(&sign_jiankuo[0],5,7,6*17); 
//        ShowChar(&Eng_a[0],5,7,6*19);	
//        ShowChar(&Eng_d[0],5,7,6*20);
    }
}
