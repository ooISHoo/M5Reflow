#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_spiffs.h"

#include "hal.h"
#include "lcd.h"
#include "control.h"

enum RootMode{
    RootMode_Reflow,
    RootMode_Baking,
    RootMode_EditReflowProfile,
    RootMode_Setting,
    RootMode_Boot
};
enum SubMode{
    SubMode_Ready,
    SubMode_Running,
    SubMode_Error,
    SubMode_RootTransit,
};

RootMode g_CurrentMode;
RootMode g_GotoMode;

SubMode g_CurrentSubMode;
SubMode g_GotoSubMode;

typedef struct{
    int PreHeatTemp;
    int PreHeatTime;
    int ReflowTemp;
    int PeakHoldTime;
    int TimerThresholdTemp;
    int RampPower;
} ReflowParam;

typedef struct{
    int ReflowTemp;
    int PeakHoldTime;
} BakingParam;

typedef struct{
    int TuningSlope;
    int TuningDx;
    int TuningIntegral;
    bool EnableSound;
    int MaxExternalTemp;
    int MaxInternalTemp;
    int ReflowTimeout;
} SettingsParam;

int g_ReflowParamIndex=0;
ReflowParam g_ReflowParam[4];
int g_ReflowParamMenu[7];
BakingParam g_BakingParam;
SettingsParam g_SettingParam;
int g_SettingParamMenu[7];

void ResetParam(){
    for(int i=0;i<4;i++){
        g_ReflowParam[i].PreHeatTemp = 160;
        g_ReflowParam[i].PreHeatTime = 90;
        g_ReflowParam[i].ReflowTemp = 260;
        g_ReflowParam[i].PeakHoldTime = 7;
        g_ReflowParam[i].TimerThresholdTemp = 200;
        g_ReflowParam[i].RampPower = 100;
    }
    g_ReflowParamMenu[0] = 0;

    g_SettingParam.TuningDx = 0;
    g_SettingParam.TuningSlope = 0;
    g_SettingParam.TuningIntegral = 0;
    g_SettingParam.EnableSound = true;
    g_SettingParam.ReflowTimeout = 800;
    g_SettingParam.MaxExternalTemp = 300;
    g_SettingParam.MaxInternalTemp = 60;
}

M5ReflowLCD::MenuRecoard EditProfileMenuItems[]={
    {"Profile no","",0,3},
    {"Preheat temp","°C",120,180},
    {"Preheat time","Sec",60,180},
    {"Reflow temp","°C",190,300},
    {"Peak hold time","Sec",0,20},
    {"Timer threshold","°C",180,250},
    {"Ramp power","%",70,100}
};

void CopyToLcdReflowParam(M5ReflowLCD& g_Lcd){
    int pindex = g_ReflowParamMenu[0];
    g_ReflowParamMenu[1] = g_ReflowParam[pindex].PreHeatTemp;
    g_ReflowParamMenu[2] = g_ReflowParam[pindex].PreHeatTime;
    g_ReflowParamMenu[3] = g_ReflowParam[pindex].ReflowTemp;
    g_ReflowParamMenu[4] = g_ReflowParam[pindex].PeakHoldTime;
    g_ReflowParamMenu[5] = g_ReflowParam[pindex].TimerThresholdTemp;
    g_ReflowParamMenu[6] = g_ReflowParam[pindex].RampPower;
}
void CopyFromLcdReflowParam(M5ReflowLCD& g_Lcd){
    int pindex = g_ReflowParamMenu[0];
    g_ReflowParam[pindex].PreHeatTemp = g_ReflowParamMenu[1];
    g_ReflowParam[pindex].PreHeatTime = g_ReflowParamMenu[2];
    g_ReflowParam[pindex].ReflowTemp = g_ReflowParamMenu[3];
    g_ReflowParam[pindex].PeakHoldTime = g_ReflowParamMenu[4];
    g_ReflowParam[pindex].TimerThresholdTemp = g_ReflowParamMenu[5];
    g_ReflowParam[pindex].RampPower = g_ReflowParamMenu[6];
    g_ReflowParamIndex = pindex;
}

void CopyToLcdSettingParam(M5ReflowLCD& g_Lcd){
    g_SettingParamMenu[0] = g_SettingParam.TuningSlope;
    g_SettingParamMenu[1] = g_SettingParam.TuningDx;
    g_SettingParamMenu[2] = g_SettingParam.TuningIntegral;
    g_SettingParamMenu[3] = g_SettingParam.EnableSound;
    g_SettingParamMenu[4] = g_SettingParam.MaxExternalTemp;
    g_SettingParamMenu[5] = g_SettingParam.MaxInternalTemp;
    g_SettingParamMenu[6] = g_SettingParam.ReflowTimeout;
}

void CopyFromLcdSettingParam(M5ReflowLCD& g_Lcd){
     g_SettingParam.TuningSlope = g_SettingParamMenu[0];
     g_SettingParam.TuningDx = g_SettingParamMenu[1];
     g_SettingParam.TuningIntegral = g_SettingParamMenu[2];
     g_SettingParam.EnableSound = g_SettingParamMenu[3];
     g_SettingParam.MaxExternalTemp = g_SettingParamMenu[4];
     g_SettingParam.MaxInternalTemp = g_SettingParamMenu[5];
     g_SettingParam.ReflowTimeout = g_SettingParamMenu[6];
}

M5ReflowLCD::MenuRecoard SettingMenuItems[]={
    {"param Slp","",-16,16},
    {"param dx","",-16,16},
    {"param int","",-16,16},
    {"Sound","",0,1},
    {"Max ext temp","°C",280,320},
    {"Max internal temp","°C",50,70},
    {"Reflow timeout","Sec",600,1200}
};

static const char Signature[16]={'M','5','R','e','f','l','o','w',' ','R','0','0','-','0','0','4'};
static const char SettingFileName[] = "/spiffs/m5reflow_settings.dat";
void loadfile()
{
    char signature_temp[16];
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    assert(ret==ESP_OK);
    FILE* f = fopen(SettingFileName, "r");
    if (f == NULL) {
        printf("can not open file\n");
        return;
    } 
    fread(signature_temp,sizeof(signature_temp),1,f);
    bool  diff = false;
    for(int i=0;i<sizeof(Signature);i++){
        if(signature_temp[i] != Signature[i]) {
            diff=true;
            break;
        }
    }
    if(diff){
        fclose(f);
        return;
    }

    fread(&g_SettingParam,sizeof(g_SettingParam),1,f);
    for(int i=0;i<4;i++){
        fread(&g_ReflowParam[i],sizeof(ReflowParam),1,f);
    }
    fclose(f);
}

int savefile()
{
    FILE* f = fopen(SettingFileName, "w");
    if (f == NULL) {
        printf("can not open file\n");
        return ESP_FAIL;
    } else {
        printf("ok hello.txt\n");
    }
    
    fwrite(Signature,sizeof(Signature),1,f);
    fwrite(&g_SettingParam,sizeof(g_SettingParam),1,f);
    for(int i=0;i<4;i++){
        fwrite(&g_ReflowParam[i],sizeof(ReflowParam),1,f);
    }
    fclose(f);
    return ESP_OK;
}

M5ReflowLCD g_Lcd;
M5ReflowControl g_Control;

extern "C" void app_main()
{
    printf("Start M5Reflow CPP!\n");
    g_CurrentMode = RootMode_Boot;
    g_GotoMode = RootMode_Reflow;
    int drawCount = 0;
    bool menuUpdate;
    ResetParam();

    auto hal = M5ReflowHAL::GetInstance();
    hal->Init();
    loadfile();
    hal->pushBuzzerNote(1000,160);
    hal->pushBuzzerNote(500,160);

    M5ReflowHAL::Result MAX31855Result;

    g_Control.stop();
    while(true){
        hal->SelectMAX31855(true);
        MAX31855Result = hal->ReadMAX31855();
        hal->SelectMAX31855(false);
        hal->UpdateKeyStatus();
        
        if(MAX31855Result == M5ReflowHAL::Result_Ok){
            g_Control.enterTemp(hal->m_ExternalTemp);
        }
        g_Control.update();

        if(g_GotoMode != g_CurrentMode)
        {
            switch(g_GotoMode){
                case RootMode_Boot:
                break;
                case RootMode_Reflow:
                    hal->SelectLCD(true);
                    g_Lcd.drawReflowBase();
                    hal->SelectLCD(false);
                    break;
                case RootMode_Baking:
                    hal->SelectLCD(true);
                    g_Lcd.drawBottomGuide("Mode"," Edit","Start");
                    g_Lcd.drawBakingBase();
                    hal->SelectLCD(false);
                    break;
                case RootMode_EditReflowProfile:
                    hal->SelectLCD(true);
                    g_ReflowParamMenu[0] = 0;
                    g_Lcd.resetMenu();
                    CopyToLcdReflowParam(g_Lcd);
                    g_Lcd.drawStatus("Reflow","Edit");
                    g_Lcd.drawBottomGuide("Back","","Edit");
                    g_Lcd.drawMenu(EditProfileMenuItems,g_ReflowParamMenu,7,0);
                    hal->SelectLCD(false);
                    break;
                case RootMode_Setting:
                    hal->SelectLCD(true);
                    g_Lcd.resetMenu();
                    CopyToLcdSettingParam(g_Lcd);
                    g_Lcd.drawStatus("Seting","");
                    g_Lcd.drawBottomGuide("Back","Save","Edit");
                    g_Lcd.drawMenu(SettingMenuItems,g_SettingParamMenu,7,0);
                    hal->SelectLCD(false);
                    break;
            }
            g_CurrentMode = g_GotoMode;
            g_CurrentSubMode = SubMode_RootTransit;
            g_GotoSubMode = SubMode_Ready;
        }
        if(g_GotoSubMode != g_CurrentSubMode)
        {
            if(g_CurrentMode == RootMode_Reflow){
                switch(g_GotoSubMode){
                    case SubMode_Ready:
                        g_Lcd.drawStatus("Reflow","Ready");
                        g_Lcd.drawBottomGuide("Mode"," Edit","Start");
                        g_Control.stop();
                        break;
                    case SubMode_Running:
                        g_Lcd.drawStatus("Reflow","Run");
                        g_Lcd.drawBottomGuide("","","Stop");
                        g_Control.stop();
                        g_Control.pushStage(
                            static_cast<float>(g_ReflowParam[g_ReflowParamIndex].PreHeatTemp),
                            0.0f,
                            0.5f,
                            0.2f,
                            static_cast<float>(g_ReflowParam[g_ReflowParamIndex].PreHeatTime) );
                        g_Control.pushStage(
                            static_cast<float>(g_ReflowParam[g_ReflowParamIndex].ReflowTemp),
                            0.0f,
                            1.0f,
                            1.0f,
                            static_cast<float>(g_ReflowParam[g_ReflowParamIndex].PeakHoldTime) );
                        g_Control.pushStage(
                            80.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.f );
                        g_Control.setMonitorThreshold(static_cast<float>(g_ReflowParam[g_ReflowParamIndex].TimerThresholdTemp));
                        g_Control.start();
                        break;
                    case SubMode_Error:
                        g_Lcd.drawStatus("Reflow","Error");
                        g_Lcd.drawBottomGuide("","","Ok");
                        g_Control.stop();
                        break;
                    default:
                        break;
                }
            } else if (g_CurrentMode == RootMode_Baking){


            }
            g_CurrentSubMode = g_GotoSubMode;
        }

        switch(g_CurrentMode){
            case RootMode_Boot:
            break;
            case RootMode_Reflow:
                hal->SelectLCD(true);
                if(MAX31855Result == M5ReflowHAL::Result_Ok){
                    g_Lcd.m_ReflowParam.m_ExternalTemp = hal->m_ExternalTemp;
                    g_Lcd.m_ReflowParam.m_InternalTemp = hal->m_InternalTemp;
                    g_Lcd.m_ReflowParam.m_TempError = false;
                } else {
                    g_Lcd.m_ReflowParam.m_ExternalTemp = 0;
                    g_Lcd.m_ReflowParam.m_InternalTemp = 0;
                    g_Lcd.m_ReflowParam.m_TempError = true;
                }
                g_Lcd.m_ReflowParam.m_PowerUpper = static_cast<float>(g_Control.getOutputPowerRaw(0) * 100.0f);
                g_Lcd.m_ReflowParam.m_PowerLower = static_cast<float>(g_Control.getOutputPowerRaw(1) * 100.0f);
                if(g_Control.getStateCode()==M5ReflowControl::StateCode_Processing){
                    g_Lcd.m_ReflowParam.m_TargetTemp = static_cast<float>(g_Control.getTargetTemp());
                    int stage = g_Control.getStageIndex();
                    stage = (stage - 1) * 2 + 1;
                    if(g_Control.isTargetReached()){
                        stage += 1;
                    }
                    g_Lcd.m_ReflowParam.m_Stage = stage;

                } else {
                    g_Lcd.m_ReflowParam.m_TargetTemp = 0;
                    g_Lcd.m_ReflowParam.m_Stage =0;
                    g_Lcd.m_ReflowParam.m_StageCount = 0;
                }
                g_Lcd.m_ReflowParam.m_StageCount = 5;
                g_Lcd.m_ReflowParam.m_ReflowTime = g_Control.getMonitorTime() / (1000*1000);
                g_Lcd.m_ReflowParam.m_RampRate = g_Control.getTempSlope();
                g_Lcd.drawReflowParam();
                hal->SelectLCD(false);
                switch(g_CurrentSubMode){
                    case SubMode_Ready:
                        {

                            if(hal->m_KeyDown[M5ReflowHAL::Key_L])
                            {
                                g_GotoMode = RootMode_Setting;
    
                            }
                            if(hal->m_KeyDown[M5ReflowHAL::Key_R])
                            {
                                g_GotoSubMode = SubMode_Running;
                            }
                            if(hal->m_KeyDown[M5ReflowHAL::Key_M])
                            {
                                g_GotoMode = RootMode_EditReflowProfile;
                            }
                        }
                        break;
                    case SubMode_Running:
                        {
                            if(hal->m_KeyDown[M5ReflowHAL::Key_R])
                            {
                                g_GotoSubMode = SubMode_Ready;
                            }
                            if(g_Control.getStateCode() != M5ReflowControl::StateCode_Processing){
                                g_GotoSubMode = SubMode_Ready;
                            }
                        }                    
                        break;
                    case SubMode_Error:
                        break;
                    default:
                        break;
                }
                break;
            case RootMode_Baking:
                hal->SelectLCD(true);
                hal->SelectLCD(false);
                if(hal->m_KeyDown[M5ReflowHAL::Key_L])
                {
                    g_GotoMode = RootMode_Setting;
                }
                break;
            case RootMode_EditReflowProfile:
                hal->SelectLCD(true);
                menuUpdate = false;
                if(hal->m_KeyDown[M5ReflowHAL::Key_M])
                {

                }
                if(hal->m_EncoderDiff != 0)
                {
                    menuUpdate = true;
                }
                if(hal->m_KeyDown[M5ReflowHAL::Key_R] || hal->m_KeyDown[M5ReflowHAL::Key_C])
                {
                    g_Lcd.MenuSelectToggle();
                    menuUpdate = true;
                }
                if(menuUpdate)
                {
                    int pindex = g_ReflowParamMenu[0];
                    g_Lcd.drawMenu(EditProfileMenuItems,g_ReflowParamMenu,7,hal->m_EncoderDiff);
                    if(pindex != g_ReflowParamMenu[0]){
                        CopyToLcdReflowParam(g_Lcd);
                        g_Lcd.drawMenu(EditProfileMenuItems,g_ReflowParamMenu,7,0);
                    } else {
                        CopyFromLcdReflowParam(g_Lcd);
                    }
                    if(g_Lcd.m_MenuSelected){
                        g_Lcd.drawBottomGuide("Back","    ","Back");
                    } else {
                        g_Lcd.drawBottomGuide("Back","    ","Edit");
                    }
                }
                hal->SelectLCD(false);
                if(hal->m_KeyDown[M5ReflowHAL::Key_L])
                {
                    g_GotoMode = RootMode_Reflow;
                }
                break;
            case RootMode_Setting:
                hal->SelectLCD(true);
                menuUpdate = false;
                if(hal->m_KeyDown[M5ReflowHAL::Key_M])
                {
                    savefile();
                    g_Lcd.drawBottomGuide("Back","Done","Edit");
                }
                if(hal->m_EncoderDiff != 0)
                {
                    menuUpdate = true;
                }
                if(hal->m_KeyDown[M5ReflowHAL::Key_R] || hal->m_KeyDown[M5ReflowHAL::Key_C])
                {
                    g_Lcd.MenuSelectToggle();
                    menuUpdate = true;
                }
                if(menuUpdate)
                {
                    g_Lcd.drawMenu(SettingMenuItems,g_SettingParamMenu,7,hal->m_EncoderDiff);
                    CopyFromLcdSettingParam(g_Lcd);
                    if(g_Lcd.m_MenuSelected){
                        g_Lcd.drawBottomGuide("Back","Save","Back");
                    } else {
                        g_Lcd.drawBottomGuide("Back","Save","Edit");
                    }
                }
                hal->SelectLCD(false);
                if(hal->m_KeyDown[M5ReflowHAL::Key_L])
                {
                    g_GotoMode = RootMode_Reflow;
                }
            break;
        }
        vTaskDelay(40 / portTICK_PERIOD_MS);
        drawCount++;
        if((drawCount / 10) % 2 ==0){
            g_Lcd.setBrink(true);
        } else {
            g_Lcd.setBrink(false);
        }
    }
}