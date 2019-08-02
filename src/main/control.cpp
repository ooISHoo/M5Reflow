#include <string.h>
#include <hal.h>
#include <math.h>

#include "control.h"


M5ReflowControl::M5ReflowControl()
{
    m_TuningSlope = 
    m_TuningDx = 
    m_TuningIntegral = 0;
    stop();
}

M5ReflowControl::~M5ReflowControl()
{
}

int M5ReflowControl::pushStage(float temp,float slope,float powerL,float powerU,int holdtime)
{
    if(m_StagesEnter >= m_StagesCount){
        return -1;
    }
    m_Stages[m_StagesEnter].temp = temp;
    m_Stages[m_StagesEnter].slope = slope;
    m_Stages[m_StagesEnter].powerL = powerL;
    m_Stages[m_StagesEnter].powerU = powerU;
    m_Stages[m_StagesEnter].holdtime = holdtime;
    m_StagesEnter++;
    return m_StagesEnter;
}

int M5ReflowControl::popStage()
{
    if(m_StagesProcess >= m_StagesEnter){
        return -1;
    }
    m_TargetTemp = m_Stages[m_StagesProcess].temp;
    m_HoldTime = m_Stages[m_StagesProcess].holdtime;
    m_MaxPowerL = m_Stages[m_StagesProcess].powerL;
    m_MaxPowerU = m_Stages[m_StagesProcess].powerU;
    m_HoldTime *= 1000 * 1000; // sec->usec
    m_TargetReached = false;

    if(m_StagesProcess > 0){
        auto hal = M5ReflowHAL::GetInstance();
        if(m_MaxPowerL > 0 || m_MaxPowerU > 0){
            hal->pushBuzzerNote(490*3,200);
        } else {
            hal->pushBuzzerNote(1800,1000);
            hal->pushBuzzerNote(0,500);
            hal->pushBuzzerNote(1800,1000);
            hal->pushBuzzerNote(0,500);
            hal->pushBuzzerNote(1800,1000);
            hal->pushBuzzerNote(0,500);
        }
    }

    m_StagesProcess ++;
    return m_StagesEnter - m_StagesProcess;
}

void M5ReflowControl::start()
{
    if(m_StagesEnter <= 0){
        return;
    }
    m_State = StateCode_Processing;
    m_StagesProcess = 0;
    m_MonitorReachTime = 0;

    auto hal = M5ReflowHAL::GetInstance();
    hal->pushBuzzerNote(290*2,100);
    hal->pushBuzzerNote(0,1);
    hal->pushBuzzerNote(390*2,100);
    hal->pushBuzzerNote(0,1);
    hal->pushBuzzerNote(490*2,100);
    hal->pushBuzzerNote(0,1000);

    popStage();
}

void M5ReflowControl::stop()
{
    if(m_State==StateCode_Processing)

    m_State = StateCode_Ready;
    m_StagesProcess = 0;
    m_StagesEnter = 0;
    m_MonitorReachTime = 0;
    m_Power = 0;
}

void M5ReflowControl::enterTemp(float temp)
{
    m_TempLog[m_TempLogCount % m_TempLogSize].temp = temp;
    m_TempLog[m_TempLogCount % m_TempLogSize].time = esp_timer_get_time();
    m_LastTemp = temp;
    m_TempLogCount ++;
}

M5ReflowControl::StateCode M5ReflowControl::update()
{
    auto hal = M5ReflowHAL::GetInstance();
    uint64_t now =  esp_timer_get_time();
    switch(m_State){
        case StateCode_Ready:
            m_Power = 0.0;
            break;
        case StateCode_Processing:
            // power control
            if(m_LastUpdateTime==0)break;
            if(m_MaxPowerU > 0 || m_MaxPowerL > 0){
                // 雑なPID実装
                float dt = static_cast<float>(now - m_LastUpdateTime);
                dt = dt / (1000.f * 1000.0f);
                float slopedtemp = m_LastTemp+(m_SlopeTemp * (12.5f + (float)m_TuningSlope * 0.2f));
                float dx = (m_TargetTemp - slopedtemp) * dt *(0.025f + (float)m_TuningDx * 0.005f);
                float integral = m_IntegralTargetDiff * dt * (0.005f + (float)m_TuningIntegral * 0.0002f);
                m_Power += dx + integral;
                if(m_Power < 0) m_Power = 0;
                if(m_Power > 1.0f) m_Power = 1.0f;

                if(m_LastTemp >= m_TargetTemp && !m_TargetReached){
                    hal->pushBuzzerNote(490*3,50);
                    hal->pushBuzzerNote(0,50);
                    hal->pushBuzzerNote(490*3,50);
                    m_TargetReached = true;
                    m_TargetReachTime = now;
                }
                printf("power %f p:%f d:%f\n",m_Power,dx,integral);
            } else {
                m_Power = 0.0f;
                if(m_LastTemp <= m_TargetTemp && !m_TargetReached){
                    m_TargetReached = true;
                    m_TargetReachTime = now;
                }
            }
            if(m_TargetReached) {
                if((esp_timer_get_time() - m_TargetReachTime) > m_HoldTime){
                    if(popStage() < 0){
                        hal->pushBuzzerNote(1800,2000);
                        m_State = StateCode_Ready;
                        m_StagesEnter = 0;
                    }
                }
            }

            // monitor
            if(m_MonitorReachTime == 0 && m_LastTemp >= m_MonitorThresholdTemp){
                m_MonitorReachTime = now;
            }
            break;
        case StateCode_Error:
            m_Power = 0.0;
            break;
    }
    calcSlope(2.0f);
    calcPulse();
    m_LastUpdateTime = now;
    return m_State;
}

static float lastPower=0;
void M5ReflowControl::calcPulse()
{
    auto hal = M5ReflowHAL::GetInstance();
    const float pre_margin = 500.f; // 位相検出後のマージン
    const float post_margin = 500.f; // 位相周期にこれを引いた値が最大パルス遅延
    if(lastPower != m_Power){
        lastPower = m_Power;
        if(m_Power==0.0f){
            hal->stopTriacPulse(0);
            hal->stopTriacPulse(1);
            return;
        }
        float p;
        float interval = static_cast<float>(hal->getSupplyPhaseInterval());
        m_PowerRaw[0] = 
        m_PowerRaw[1] = m_Power;
        if(m_PowerRaw[0] > m_MaxPowerU) m_PowerRaw[0] = m_MaxPowerU;
        if(m_PowerRaw[1] > m_MaxPowerL) m_PowerRaw[1] = m_MaxPowerL;
        for(int i=0;i<2;i++){
            p = asinf(m_PowerRaw[i] * 2.0f - 1.0f);
            p = (p + (M_PI/2.0f)) / M_PI;
            p = interval - (interval * p);
            if(p > interval - post_margin) p = interval - post_margin;
            if(p < pre_margin)  p = pre_margin;
            hal->pushTriacPulseDelay(i,static_cast<int>(p));
            //printf("power %f temp:%f set Delay %f\n",m_Power,m_LastTemp,p);
        }
    }
}

void M5ReflowControl::calcSlope(float time)
{
    int i;
    int logindex = m_TempLogCount-1;
    if(logindex==0){
        m_SlopeTemp = 0;
        return;
    }
    uint64_t firstTime = m_TempLog[logindex % m_TempLogSize].time;
    float firstTemp = m_TempLog[logindex % m_TempLogSize].temp;
    // 指定時間まで巻き戻す
    for(i=0;i<m_TempLogSize-1;i++){
        if(--logindex<0) break;
        if(static_cast<float>(firstTime - m_TempLog[logindex % m_TempLogSize].time) > time * 1000 * 1000){
            break;
        }
    }
    float dt = static_cast<float>(firstTime - m_TempLog[logindex % m_TempLogSize].time);
    dt = dt / (1000.f * 1000.0f);
    float dx = static_cast<float>(firstTemp - m_TempLog[logindex % m_TempLogSize].temp);
    if(dt < 0.1f){
        m_SlopeTemp = 0;
        return;
    }
    m_SlopeTemp = dx / dt;

    // TargetTempまでの積分を求める、timeの端数で誤差が出るがとりあえず無視で
    m_IntegralTargetDiff = 0;
    uint64_t lastTime = m_TempLog[logindex % m_TempLogSize].time;
    for(i=logindex+1;i<m_TempLogCount;i++){
        dt = static_cast<float>(m_TempLog[i % m_TempLogSize].time - lastTime);
        dt = dt / (1000.f * 1000.0f);
        dx = static_cast<float>(m_TargetTemp - m_TempLog[i % m_TempLogSize].temp);

        m_IntegralTargetDiff += dx * dt;
        lastTime = m_TempLog[i % m_TempLogSize].time;
    }
    //printf("slope %d %f %f\n",i,dx,dt);
}