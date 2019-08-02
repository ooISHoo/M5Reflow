#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"

class M5ReflowControl{
public:
    M5ReflowControl();
    ~M5ReflowControl();
    enum StateCode{
        StateCode_Ready,
        StateCode_Processing,
        StateCode_Error,
    };

    int pushStage(float temp, float slope, float powerL,float powerU, int holdtime);
    int popStage();
    void start();
    void stop();
    StateCode update();
    StateCode getStateCode();
    void enterTemp(float temp);
    void calcSlope(float time);

    void setMonitorThreshold(float temp);
    void setTuningParam(int slope,int dx,int integral);
    uint64_t getMonitorTime();

    int getStageIndex();
    float getTargetTemp();
    bool isTargetReached();

    float getOutputPower();
    float getOutputPowerRaw(int ch);
    float getTempSlope();
private:
    float m_TargetTemp;
    uint64_t m_HoldTime;
    uint64_t m_TargetReachTime;
    uint64_t m_LastUpdateTime;
    bool m_TargetReached;
    float m_MaxPowerU;
    float m_MaxPowerL;

    float m_MonitorThresholdTemp;
    uint64_t m_MonitorReachTime;

    // works
    float m_LastTemp;
    float m_SlopeTemp;
    float m_IntegralTargetDiff;
    float m_Power;
    float m_PowerRaw[2];

    StateCode m_State;
    typedef struct{
        float temp;
        float powerU;
        float powerL;
        float slope;
        int holdtime;
        uint64_t starttime;
    }StageRecoard;

    static const int m_StagesCount = 16;
    StageRecoard m_Stages[m_StagesCount];
    int m_StagesProcess;
    int m_StagesEnter;

    typedef struct{
        float temp;
        uint64_t time;
    }TempRecoard;

    static const int m_TempLogSize = 64;
    TempRecoard m_TempLog[m_TempLogSize];
    int m_TempLogCount;

    int m_TuningSlope;
    int m_TuningDx;
    int m_TuningIntegral;

    void calcPulse();
};


inline M5ReflowControl::StateCode M5ReflowControl::getStateCode(){
    return m_State;
}
inline void M5ReflowControl::setMonitorThreshold(float temp){
    m_MonitorReachTime = 0;
    m_MonitorThresholdTemp = temp;
}
inline uint64_t M5ReflowControl::getMonitorTime(){
    if(m_MonitorReachTime == 0) return 0;
    return esp_timer_get_time() - m_MonitorReachTime;
}


inline int M5ReflowControl::getStageIndex(){
    return m_StagesProcess;
}
inline float M5ReflowControl::getTargetTemp(){
    return m_TargetTemp;
}
inline bool M5ReflowControl::isTargetReached(){
    return m_TargetReached;
}
inline float M5ReflowControl::getOutputPower(){
    return m_Power;
}
inline float M5ReflowControl::getOutputPowerRaw(int ch){
    return m_PowerRaw[ch];
}
inline float M5ReflowControl::getTempSlope(){
    return m_SlopeTemp;
}