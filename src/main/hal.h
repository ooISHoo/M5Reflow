
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/spi_reg.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"

class M5ReflowHAL{
public:
    M5ReflowHAL();
    ~M5ReflowHAL();
    static M5ReflowHAL* GetInstance();
    enum Key{
        Key_L,
        Key_M,
        Key_R,
        Key_C,
        Key_Count
    };
    enum Result{
        Result_Ok,
        Result_Error,
    };
    bool m_KeyDown[Key_Count];
    bool m_KeyState[Key_Count];
    int m_EncoderDiff;
    void Init();
    Result ReadMAX31855();
    void SelectMAX31855(bool sel);
    void SelectLCD(bool sel);
    void UpdateKeyStatus();
    void pushBuzzerNote(int freq,int time);
    void pushTriacPulseDelay(int ch,int time);
    void stopTriacPulse(int ch);
    int getSupplyPhaseInterval();

    float m_InternalTemp;
    float m_ExternalTemp;
    int m_SupplyPhaseInterval;

    static const gpio_num_t GPIO_INPUT_PHASE_A;
    static const gpio_num_t GPIO_INPUT_PHASE_B;

    static const gpio_num_t GPIO_OUTPUT_TRIAC_T1;
    static const gpio_num_t GPIO_OUTPUT_TRIAC_T2;

    static const gpio_num_t GPIO_INPUT_ENCODER_A;
    static const gpio_num_t GPIO_INPUT_ENCODER_B;
    static const gpio_num_t GPIO_INPUT_ENCODER_D;

    static const gpio_num_t GPIO_INPUT_BUTTON_L;
    static const gpio_num_t GPIO_INPUT_BUTTON_M;
    static const gpio_num_t GPIO_INPUT_BUTTON_R;

    static const gpio_num_t GPIO_OUTPUT_BUZZER;

    static const timer_idx_t TIMER_INDEX_MICRO;
private:
    void InitLCD();
    void InitTriac();
    void InitBuzzer();
    void InitMAX31855();
    void InitEncoder();
    


};

inline int M5ReflowHAL::getSupplyPhaseInterval(){
    return m_SupplyPhaseInterval;
}