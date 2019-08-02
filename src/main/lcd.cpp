
#include <string.h>
#include <hal.h>
#include "tft.h"

#include "lcd.h"

void drawGraph(const int *gDw,const int *gDh,int count,int gX,int gY,int curcount,color_t normcolor,color_t selectcolor)
{
    color_t drawcolor;
    int x=0,y=0;
    for(int i=0;i<count;i++){
        if(curcount-1 == i) drawcolor = selectcolor; else  drawcolor = normcolor; 
        if(gDh[i]>0){
            TFT_fillTriangle(gX+x,gY-y-1, gX+x+gDw[i],gY-y-1, gX+x+gDw[i],gY-(y+gDh[i])-1,drawcolor);
        } else if (gDh[i]<0) {
            TFT_fillTriangle(gX+x,gY-y-1, gX+x+gDw[i],gY-(y+gDh[i])-1, gX+x,gY-(y+gDh[i])-1,drawcolor);
        }
        if( y>0 ){
            if(gDh[i] < 0){
                if(y+gDh[i] > 0) TFT_fillRect(gX+x,gY-(y+gDh[i]),gDw[i],y+gDh[i],drawcolor);
            } else {
                TFT_fillRect(gX+x,gY-y,gDw[i],y,drawcolor);
            }
        }
        x += gDw[i];
        y += gDh[i];
    }
}

M5ReflowLCD::M5ReflowLCD()
{
    m_ReflowParam.m_PowerLower = 0;
    m_ReflowParam.m_PowerUpper = 0;
    m_MenuCursor = 0;
}

M5ReflowLCD::~M5ReflowLCD()
{
}

void M5ReflowLCD::drawStatus(const char *mode,const char *status)
{
    char tmp_buff[64];
    color_t title_red = (color_t){ 234, 85, 20 };

    _bg = title_red;
    _fg = TFT_BLACK;
    TFT_fillRect(0, 0, 320, 22,title_red);
    TFT_setFont(UBUNTUB21_FONT, NULL);
    sprintf(tmp_buff, "Mode:%s", mode);
	TFT_print(tmp_buff, 0, 0);
    sprintf(tmp_buff, "Status:%s", status);
    TFT_print(tmp_buff, 148, 0);
}

void M5ReflowLCD::drawBottomGuide(const char *left,const char *mid,const char *right)
{
    char tmp_buff[64];
    color_t title_red = (color_t){ 234, 85, 20 };
    _bg = title_red;
    TFT_fillRect(0, 208, 320, 32,TFT_WHITE);
    TFT_fillRect(0, 208, 96, 32,title_red);
    TFT_fillRect(112,208, 96, 32,title_red);
    TFT_fillRect(224,208, 96, 32,title_red);
    TFT_setFont(UBUNTUB21_FONT, NULL);
    sprintf(tmp_buff, "%s", left);
    TFT_print(tmp_buff, 18, 214);
    sprintf(tmp_buff, "%s", mid);
    TFT_print(tmp_buff, 130, 214);
    sprintf(tmp_buff, "%s", right);
    TFT_print(tmp_buff, 246, 214);
}

void M5ReflowLCD::drawReflowBase()
{
    char tmp_buff[64];
    color_t heater_red = (color_t){ 255, 153, 117 };
    color_t centor_gray = (color_t){ 200, 200, 210 };
	_bg = TFT_WHITE;
    _fg = TFT_BLACK;

    TFT_fillRect(0, 23, 320, 188,TFT_WHITE);
    TFT_fillRect(0, 80, 320, 48,centor_gray);

	//TFT_resetclipwin();
    _bg = TFT_WHITE;
    TFT_setFont(UBUNTUB21_FONT, NULL);
    sprintf(tmp_buff, "To:");
    TFT_print(tmp_buff, 144, 26);
    sprintf(tmp_buff, "Ramp:");
    TFT_print(tmp_buff, 144, 54);
    TFT_setFont(OCRASTD36_FONT, NULL);
    sprintf(tmp_buff, "°C");
    TFT_print(tmp_buff, 80, 28);
    TFT_setFont(OCRASTD24_FONT, NULL);
    TFT_print(tmp_buff, 280, 26);//264
    TFT_print(tmp_buff, 280, 54);

    _bg = centor_gray;
    TFT_setFont(UBUNTUB21_FONT, NULL);
    sprintf(tmp_buff, "Stage:");
    TFT_print(tmp_buff, 0, 82);
    sprintf(tmp_buff, "Total:");
    TFT_print(tmp_buff, 0, 103);

    TFT_fillRect(16, 130, 96, 28,heater_red);
    TFT_fillRect(16, 178, 96, 28,heater_red);


}
void M5ReflowLCD::drawReflowParam()
{
    color_t heater_red = (color_t){ 255, 153, 117 };
    color_t centor_gray = (color_t){ 200, 200, 210 };
    char tmp_buff[64];

    _bg = TFT_WHITE;
    TFT_setFont(OCRASTD36_FONT, NULL);

    if(m_ReflowParam.m_TempError){
        sprintf(tmp_buff, "ERR");
    } else {
        int disptemp = (int)m_ReflowParam.m_ExternalTemp;
        if(disptemp > 999) disptemp = 999;
        if(disptemp < -99) disptemp = -99;
        if(disptemp < 0){
            sprintf(tmp_buff, "%+02d", disptemp );
        } else {
            sprintf(tmp_buff, "%03d", disptemp );
        }
    }
    TFT_print(tmp_buff, 0, 28);
    TFT_setFont(OCRASTD24_FONT, NULL);
    if(m_ReflowParam.m_TempError){
        sprintf(tmp_buff, "ERR");
    } else {
        int disptemp = (int)m_ReflowParam.m_TargetTemp;
        if(disptemp > 999) disptemp = 999;
        if(disptemp < -99) disptemp = -99;
        if(disptemp < 0){
            sprintf(tmp_buff, "%+02d", disptemp );
        } else {
            sprintf(tmp_buff, "%03d", disptemp );
        }
    }
    TFT_print(tmp_buff, 228, 26);
    if(m_ReflowParam.m_TempError){
        sprintf(tmp_buff, "ERR");
    } else {
        float dispramp = m_ReflowParam.m_RampRate;
        if(dispramp >  9.9f) dispramp =  9.9f;
        if(dispramp < -9.9f) dispramp = -9.9f;
        sprintf(tmp_buff, "%+3.1f",dispramp);
    }
    TFT_print(tmp_buff, 210, 54);

    _bg = centor_gray;
    TFT_setFont(OCRASTD24_FONT, NULL);
    sprintf(tmp_buff, "%03d/%03d", m_ReflowParam.m_Stage,m_ReflowParam.m_StageCount);
    TFT_print(tmp_buff, 82, 82);
    sprintf(tmp_buff, "%03d", m_ReflowParam.m_ReflowTime);
    TFT_print(tmp_buff, 82, 103);

    _bg = heater_red;
    sprintf(tmp_buff, "%03d%%", m_ReflowParam.m_PowerUpper);
    TFT_print(tmp_buff, 30, 133);
    sprintf(tmp_buff, "%03d%%", m_ReflowParam.m_PowerLower);
    TFT_print(tmp_buff, 30, 181);

    int gY = 200;
    int gX = 128+8;
    color_t graphcolor_selected;
    color_t graph_gray = (color_t){ 220, 220, 240 };
    color_t graph_red = (color_t){ 255, 153, 117 };
    int gDw[5] = {32,64,32,16,32};
    int gDh[5] = {32,0,32,0,-64};
    if(m_isBrink){
        graphcolor_selected = graph_red;
    } else {
        graphcolor_selected = graph_gray;
    }
    drawGraph(gDw,gDh,5,gX,gY,m_ReflowParam.m_Stage,graph_gray,graphcolor_selected);
}

void M5ReflowLCD::drawMenu(MenuRecoard* record,int* params,int recordCnt,int inputDiff)
{
    if(m_MenuSelected)
    {
        params[m_MenuCursor] += inputDiff;
        if(params[m_MenuCursor] < record[m_MenuCursor].lower){
            params[m_MenuCursor] = record[m_MenuCursor].lower;
        }
        if(params[m_MenuCursor] > record[m_MenuCursor].upper){
            params[m_MenuCursor] = record[m_MenuCursor].upper;
        }
    } else {
        m_MenuCursor += inputDiff;
        if(m_MenuCursor < 0){
            m_MenuCursor = 0;
        }
        if(m_MenuCursor >= recordCnt){
            m_MenuCursor = recordCnt-1;
        }
    }

    char tmp_buff[64];
    color_t gray = (color_t){ 220, 220, 230 };
    color_t red = (color_t){ 255, 153, 117 };
	_bg = TFT_WHITE;
    _fg = TFT_BLACK;
    TFT_fillRect(0, 23, 320, 188,TFT_WHITE);

    int lX=8,rX =192, y=22, dY = 26;
    for(int i=0;i<recordCnt;i++){
        if(m_MenuCursor == i){
            _bg = gray;
            if(m_MenuSelected){
                TFT_fillRect(0, y, 192, dY,gray);
                TFT_fillRect(192, y, 128, dY,red);
            } else {
                TFT_fillRect(0, y, 320, dY,gray);
            }
        } else {
            _bg = TFT_WHITE;
        }
        TFT_setFont(UBUNTUB21_FONT, NULL);
        sprintf(tmp_buff,"%s", record[i].menustr);
        TFT_print(tmp_buff, lX, y+2);

        if(m_MenuCursor == i && m_MenuSelected){
            _bg = red;
        }
        TFT_setFont(OCRASTD24_FONT, NULL);
        sprintf(tmp_buff,"%d%s",params[i],record[i].suffixstr);
        TFT_print(tmp_buff, rX, y+1);
        y += dY;
    }
}

void M5ReflowLCD::resetMenu()
{
    m_MenuCursor = 0;
    m_MenuSelected = false;
}

void M5ReflowLCD::MenuSelectToggle()
{
   if(m_MenuSelected)
    {
        m_MenuSelected = false;
    } else {
        m_MenuSelected = true;
    }
}

void M5ReflowLCD::drawBakingBase()
{
    color_t centor_gray = (color_t){ 200, 200, 210 };
	_bg = TFT_WHITE;
    _fg = TFT_BLACK;

    TFT_fillRect(0, 23, 320, 188,TFT_WHITE);
    TFT_fillRect(0, 80, 320, 48,centor_gray);
}

void M5ReflowLCD::drawBakingParam()
{
}

void M5ReflowLCD::drawSettingBase()
{
    color_t centor_gray = (color_t){ 200, 200, 210 };
	_bg = TFT_WHITE;
    _fg = TFT_BLACK;

    TFT_fillRect(0, 23, 320, 188,TFT_WHITE);
    TFT_fillRect(0, 80, 320, 48,centor_gray);
}

void M5ReflowLCD::drawSettingParam()
{
}

