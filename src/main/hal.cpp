
#include <string.h>
#include <hal.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"
#include "freertos/ringbuf.h"
#include "tftspi.h"
#include "tft.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"
#include "nvs.h"
// ==========================================================
// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_HSPI_HOST
// ==========================================================
#define MAX31855_MISO   (19)
#define MAX31855_SCK   (18)
#define MAX31855_CS     (5)


// http://esp32.info/docs/esp_idf/html/dc/d9a/driver_2include_2driver_2gpio_8h.html#a08eaa8be58a0ceab6e64bb2ad80f5763
const gpio_num_t M5ReflowHAL::GPIO_INPUT_PHASE_A = GPIO_NUM_35;
const gpio_num_t M5ReflowHAL::GPIO_INPUT_PHASE_B = GPIO_NUM_36;

const gpio_num_t M5ReflowHAL::GPIO_OUTPUT_TRIAC_T1 = GPIO_NUM_22;
const gpio_num_t M5ReflowHAL::GPIO_OUTPUT_TRIAC_T2 = GPIO_NUM_13;

const gpio_num_t M5ReflowHAL::GPIO_INPUT_ENCODER_A = GPIO_NUM_21;
const gpio_num_t M5ReflowHAL::GPIO_INPUT_ENCODER_B = GPIO_NUM_2;
const gpio_num_t M5ReflowHAL::GPIO_INPUT_ENCODER_D = GPIO_NUM_12;

const gpio_num_t M5ReflowHAL::GPIO_INPUT_BUTTON_L = GPIO_NUM_39;
const gpio_num_t M5ReflowHAL::GPIO_INPUT_BUTTON_M = GPIO_NUM_38;
const gpio_num_t M5ReflowHAL::GPIO_INPUT_BUTTON_R = GPIO_NUM_37;

const gpio_num_t M5ReflowHAL::GPIO_OUTPUT_BUZZER = GPIO_NUM_15;

const timer_idx_t M5ReflowHAL::TIMER_INDEX_MICRO = TIMER_0;
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_DIVIDER_MICRO         80 //  Hardware timer clock divider

static M5ReflowHAL g_M5ReflowHAL;

// Triac control
static uint64_t RawPhaseInterval=0;
static uint64_t RawErrorCount=0;
RingbufHandle_t triac_rbuf_handle;
static void haltaskTriac(void* arg)
{
    const gpio_num_t phase_gpio[2]={M5ReflowHAL::GPIO_INPUT_PHASE_A,M5ReflowHAL::GPIO_INPUT_PHASE_B};
    const gpio_num_t triac_gpio[2]={M5ReflowHAL::GPIO_OUTPUT_TRIAC_T1,M5ReflowHAL::GPIO_OUTPUT_TRIAC_T2};

    { // 電源位相周期検出
        int last_phase_levels[2] = {1,1};
        const int edgeCount = 16;
        uint64_t phase_edge_timer_value[2][edgeCount];
        int phase_edge_count[2]={0,0};
        uint64_t start_counter_value;
        uint64_t current_counter_value;
        vTaskDelay(portTICK_PERIOD_MS * 50);
        start_counter_value = esp_timer_get_time();
        while(true){
            current_counter_value = esp_timer_get_time();

            for(int i=0;i<2;i++){
                int phase_level = gpio_get_level (phase_gpio[i]);
                if(last_phase_levels[i]==0 && phase_level==1 && phase_edge_count[i] < edgeCount){
                    phase_edge_timer_value[i][(phase_edge_count[i])++] = current_counter_value;
                    current_counter_value = esp_timer_get_time();
                }
                last_phase_levels[i] = phase_level;
            }

            if(phase_edge_count[0] >= edgeCount && phase_edge_count[1] >= edgeCount){
                break;
            }
            if(current_counter_value - start_counter_value > 1000 * 1000){
                printf("phase timer timeout %d %d\n",phase_edge_count[0],phase_edge_count[1]);
                g_M5ReflowHAL.m_SupplyPhaseInterval = 0;
                while(true){
                    vTaskDelay(portTICK_PERIOD_MS * 100);
                }
            }
        }

        uint64_t phase_avr[2]={0,0};
        for(int i=0;i<2;i++){
            for(int j=0;j<edgeCount-1;j++)
            {
                phase_avr[i] += phase_edge_timer_value[i][j+1] - phase_edge_timer_value [i][j];
            }
            phase_avr[i] /= edgeCount-1;
        }
        RawPhaseInterval = (phase_avr[0] + phase_avr[1]) / 4; // 半周期でスイッチ
        printf("rint:%d\n",static_cast<int>(RawPhaseInterval));

        g_M5ReflowHAL.m_SupplyPhaseInterval = static_cast<int>(RawPhaseInterval);
    }

    { // 制御開始
        enum PulseStage{
            PulseStage_Off,
            PulseStage_Wait,
            PulseStage_Pulse,
            PulseStage_AfterPulse,
        };
        PulseStage pulse_stages[2]={PulseStage_Off,PulseStage_Off};
        PulseStage pulse_stages_receive[2]={PulseStage_Off,PulseStage_Off};

        int last_phase_levels[2] = {1,1};
        uint64_t edge_counter_value=0;
        uint64_t edge_elapse_time=0;
        uint64_t wait_counter_value;
        uint64_t triac_pulsewait[2];
        uint64_t triac_pulsewait_receive[2];
        const uint64_t triac_pulselength = 20;  // トライアック作動パルス
        const uint64_t wait_counter_timeout = 30 * 1000; // ディレイ突入ロジックのタイムアウト、作動途中で位相信号が消失した場合用
        bool sense_phase_edge;

        wait_counter_value = 
        edge_counter_value = esp_timer_get_time();
        while(true){
            sense_phase_edge = false;
            for(int i=0;i<2;i++){
                int phase_level = gpio_get_level (phase_gpio[i]);
                if(last_phase_levels[i]==0 && phase_level==1){
                    sense_phase_edge = true;
                }
                last_phase_levels[i] = phase_level;
            }
            if(sense_phase_edge){
                for(int i=0;i<2;i++){
                    switch(pulse_stages[i]){
                        case PulseStage_Off:
                            gpio_set_level(triac_gpio[i] , 0);
                        break;
                        case PulseStage_Wait: // err
                            RawErrorCount++;
                        break;
                        case PulseStage_Pulse: // err
                            RawErrorCount++;
                            gpio_set_level(triac_gpio[i] , 0);
                        break;
                        case PulseStage_AfterPulse:
                        break;                            
                    }
                    pulse_stages[i] = pulse_stages_receive[i];
                    triac_pulsewait[i] = triac_pulsewait_receive[i];
                }
                edge_counter_value = esp_timer_get_time();
            }
            if(edge_counter_value != 0){
                edge_elapse_time = esp_timer_get_time() - edge_counter_value;
            }
            for(int i=0;i<2;i++){
                switch(pulse_stages[i]){
                    case PulseStage_Off:
                    break;
                    case PulseStage_Wait:
                        if(edge_elapse_time > triac_pulsewait[i]){
                            gpio_set_level(triac_gpio[i] , 1);
                            pulse_stages[i] = PulseStage_Pulse;
                        }
                    break;
                    case PulseStage_Pulse:
                        if(edge_elapse_time > triac_pulsewait[i] + triac_pulselength){
                            gpio_set_level(triac_gpio[i] , 0);
                            pulse_stages[i] = PulseStage_AfterPulse;
                        }
                    break;
                    case PulseStage_AfterPulse:
                    break;
                }
            }

            {
                bool can_delay = true;
                const uint64_t candelaytime = 1200;
                if(edge_elapse_time > RawPhaseInterval - candelaytime ){
                    can_delay = false;
                }
                for(int i=0;i<2;i++){
                    switch(pulse_stages[i]){
                        case PulseStage_Wait:
                            if(edge_elapse_time + candelaytime > triac_pulsewait[i]){
                                can_delay = false;
                            }
                        case PulseStage_Pulse:
                            can_delay = false;
                            break;
                        default:
                            break;
                    }
                }
                if(can_delay || esp_timer_get_time() - wait_counter_value > wait_counter_timeout){
                    if( esp_timer_get_time() - wait_counter_value > wait_counter_timeout){
                        RawErrorCount++;
                        #if 0
                        printf("delay %d  %d:%d %d:%d\n",static_cast<int>(esp_timer_get_time() - wait_counter_value),
                        pulse_stages[0],
                        (int)triac_pulsewait[0],
                        pulse_stages[1],
                        (int)triac_pulsewait[1]);
                        #endif
                    }
                    size_t item_size = sizeof(uint32_t);
                    uint32_t *item = (uint32_t *)xRingbufferReceive(triac_rbuf_handle, &item_size, pdMS_TO_TICKS(0));
                    if(item!=NULL){
                        uint32_t ch = *item >> 24;
                        uint32_t dtime = (*item & 0xffffff);
                        vRingbufferReturnItem(triac_rbuf_handle, (void *)item);
                        if(ch < 2){
                            if(dtime == 0xffffff){
                                triac_pulsewait_receive[ch] = 0xffff;
                                pulse_stages_receive[ch] = PulseStage_Off;
                                gpio_set_level(triac_gpio[0] , 0);
                            } else {
                                triac_pulsewait_receive[ch] = dtime;
                                pulse_stages_receive[ch] = PulseStage_Wait;
                            }
                        }
                    }
                    vTaskDelay(portTICK_PERIOD_MS);
                    wait_counter_value = esp_timer_get_time();
                }
            }
        }
    }
}

// Buzzer & Key
static int RawEncoderDiff;
static int RawEncoderPush;
static void haltaskEncoder(void* arg)
{
    RawEncoderDiff = 0;
    int last_encoderA_level;
    bool isFirst = true;
    for(;;){
        int encoderA_level = gpio_get_level (M5ReflowHAL::GPIO_INPUT_ENCODER_A);
        int encoderB_level = gpio_get_level (M5ReflowHAL::GPIO_INPUT_ENCODER_B);
        RawEncoderPush = gpio_get_level (M5ReflowHAL::GPIO_INPUT_ENCODER_D);
        if(!isFirst){
            if(last_encoderA_level ^ encoderA_level)
            {
                if(encoderA_level==0)
                {
                    if(encoderB_level==0){
                        RawEncoderDiff++;
                    }else{
                        RawEncoderDiff--;
                    }
                }
            }
        } else {
            isFirst = false;
        }
        last_encoderA_level = encoderA_level;

        vTaskDelay(portTICK_PERIOD_MS * 1);
    }
}

RingbufHandle_t buzzer_rbuf_handle;
static void haltaskBuzzer(void* arg)
{
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 60.0;       //duty cycle of PWMxA = 60.0%
    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);

    uint64_t note_len=0;
    int note_tone=1000;
    uint64_t note_start_counter;
    uint64_t current_counter;
    while(true) {
        if(note_len<=0){
            size_t item_size = sizeof(uint32_t);
            uint32_t *item = (uint32_t *)xRingbufferReceive(buzzer_rbuf_handle, &item_size, pdMS_TO_TICKS(0));
            if(item!=NULL){
                note_tone = *item >> 16;
                note_len = (*item & 0xffff) * 1000; // ms->us
                vRingbufferReturnItem(buzzer_rbuf_handle, (void *)item);
                note_start_counter = esp_timer_get_time();
                if(note_tone==0){
                    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                } else {
                    pwm_config.frequency = note_tone;
                    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        if(note_len > 0){
            current_counter = esp_timer_get_time();
            if(current_counter > note_start_counter + note_len){
                note_len = 0;
                mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
            }
        }
    }
}

M5ReflowHAL::M5ReflowHAL()
{

}



M5ReflowHAL::~M5ReflowHAL()
{

}

M5ReflowHAL* M5ReflowHAL::GetInstance()
{
    return &g_M5ReflowHAL;
}

void M5ReflowHAL::Init()
{
    printf("start hal init\n");
    InitTriac();
    InitLCD();
    InitMAX31855();
    InitBuzzer();
    InitEncoder();
    printf("end hal init\n");
    xTaskCreatePinnedToCore(haltaskTriac, "triac_control_task", 16384, NULL, 1, NULL,1);
    xTaskCreatePinnedToCore(haltaskEncoder, "encoder_update_task", 4096, NULL, 1, NULL,0);
    xTaskCreatePinnedToCore(haltaskBuzzer, "buzzer_update_task", 4096, NULL, 1, NULL,0);
    printf("end create task\n");
}

void M5ReflowHAL::InitLCD()
{
    esp_err_t ret;

    // ====================================================================
    // === Pins MUST be initialized before SPI interface initialization ===
    // ====================================================================
    TFT_PinsInit();

    // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

    spi_lobo_device_handle_t spi;
	
    spi_lobo_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 6 * 1024;

    spi_lobo_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 8000000;           // Initial clock out at 8 MHz
    devcfg.mode = 0;                           // SPI mode 0
    devcfg.spics_io_num = -1;                  // we will use external CS pin
    devcfg.spics_ext_io_num = PIN_NUM_CS;               // external CS pin
    devcfg.flags = LB_SPI_DEVICE_HALFDUPLEX;   // ALWAYS SET  to HALF DUPLEX MODE!! for display spi

    vTaskDelay(500 / portTICK_RATE_MS);
	ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
	printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	disp_spi = spi;

	// ==== Test select/deselect ====
    SelectLCD(true);
    SelectLCD(false);

	printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");


    printf("SPI: display init...\r\n");
	TFT_display_init();
    printf("OK\r\n");

	// ---- Detect maximum read speed ----
	max_rdclock = find_rd_speed();
	printf("SPI: Max rd speed = %u\r\n", max_rdclock);

    // ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
	printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_resetclipwin();

    vTaskDelay(500 / portTICK_PERIOD_MS);
    SelectLCD(true);
    _bg = TFT_WHITE;
    _fg = TFT_BLACK;
	TFT_fillScreen(TFT_RED);
    SelectLCD(false);

}
void M5ReflowHAL::SelectLCD(bool sel)
{
    esp_err_t ret;
    if(sel){
        ret = spi_lobo_device_select(disp_spi, 1);
        assert(ret==ESP_OK);
    } else {
        ret = spi_lobo_device_deselect(disp_spi);
        assert(ret==ESP_OK);
    }
}

spi_lobo_device_handle_t spirobo_devhandle_max31855;
void M5ReflowHAL::InitMAX31855()
{
    esp_err_t ret;
    spi_lobo_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.sclk_io_num = PIN_NUM_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 6 * 1024;

    spi_lobo_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 1000000;           // Initial clock out at 8 MHz
    devcfg.mode = 0;                           // SPI mode 0
    devcfg.spics_io_num = -1;                  // we will use external CS pin
    devcfg.spics_ext_io_num = 5;               // external CS pin
    devcfg.flags = LB_SPI_DEVICE_HALFDUPLEX;   // ALWAYS SET  to HALF DUPLEX MODE!! for display spi

	ret = spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spirobo_devhandle_max31855);
    assert(ret==ESP_OK);
    ret = spi_lobo_device_deselect(spirobo_devhandle_max31855);
    assert(ret==ESP_OK);
    return;
}
void M5ReflowHAL::SelectMAX31855(bool sel)
{
    esp_err_t ret;
    if(sel){
        ret = spi_lobo_device_select(spirobo_devhandle_max31855, 1);
        assert(ret==ESP_OK);
    } else {
        ret = spi_lobo_device_deselect(spirobo_devhandle_max31855);
        assert(ret==ESP_OK);
    }
}

void M5ReflowHAL::InitTriac()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<GPIO_INPUT_PHASE_A) | (1ULL<<GPIO_INPUT_PHASE_B);
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<GPIO_OUTPUT_TRIAC_T1) | (1ULL<<GPIO_OUTPUT_TRIAC_T2);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_TRIAC_T1, 0);
    gpio_set_level(GPIO_OUTPUT_TRIAC_T2, 0);

    triac_rbuf_handle = xRingbufferCreate(1024, RINGBUF_TYPE_NOSPLIT);
}

void M5ReflowHAL::pushTriacPulseDelay(int ch,int time)
{
    uint32_t data = ch << 24 | time;
    xRingbufferSend(triac_rbuf_handle, static_cast<void *>(&data), sizeof(data), pdMS_TO_TICKS(0));
}

void M5ReflowHAL::stopTriacPulse(int ch)
{
    uint32_t data = ch << 24 | 0xffffff;
    xRingbufferSend(triac_rbuf_handle, static_cast<void *>(&data), sizeof(data), pdMS_TO_TICKS(0));
}

void M5ReflowHAL::InitBuzzer()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<GPIO_OUTPUT_BUZZER);
    //set as input mode    
    io_conf.mode = GPIO_MODE_OUTPUT;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_OUTPUT_BUZZER);
    mcpwm_pin_config_t pin_config;
    memset(&pin_config, 0, sizeof(pin_config));
    pin_config.mcpwm0a_out_num = GPIO_OUTPUT_BUZZER;
    
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

    buzzer_rbuf_handle = xRingbufferCreate(128, RINGBUF_TYPE_NOSPLIT);
}
void M5ReflowHAL::pushBuzzerNote(int freq,int time)
{
    uint32_t data = freq << 16 | time;
    xRingbufferSend(buzzer_rbuf_handle, static_cast<void *>(&data), sizeof(data), pdMS_TO_TICKS(0));
}

void M5ReflowHAL::InitEncoder()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<GPIO_INPUT_ENCODER_A) | (1ULL<<GPIO_INPUT_ENCODER_B) | (1ULL<<GPIO_INPUT_ENCODER_D);
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
        //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (1ULL<<GPIO_INPUT_BUTTON_L) | (1ULL<<GPIO_INPUT_BUTTON_M) | (1ULL<<GPIO_INPUT_BUTTON_R);
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
        //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

M5ReflowHAL::Result M5ReflowHAL::ReadMAX31855()
{

    esp_err_t ret;
    uint8_t buf[32];

    spi_lobo_transaction_t robo_trans;
    robo_trans.flags = LB_SPI_TRANS_USE_RXDATA;
    robo_trans.command = 0;
    robo_trans.address = 0;
    robo_trans.length = 0;
    robo_trans.rxlength = 32;
    robo_trans.rx_buffer = buf;
    for(int i=0;i<16;i++)
    {
        buf[i] = 0;
        robo_trans.rx_data[i]=0;
    }

    for(int lp=0;lp<1;lp++){
        ret = spi_lobo_transfer_data(spirobo_devhandle_max31855,&robo_trans);
        assert(ret==ESP_OK);
    }
    uint32_t raw = robo_trans.rx_data[3] | (robo_trans.rx_data[2]<<8) | (robo_trans.rx_data[1]<<16) | (robo_trans.rx_data[0]<<24);

    if((raw & 0x10000) != 0){
         m_InternalTemp = 999;
         m_ExternalTemp = 999;
        return Result_Error;
    }
    int16_t external = (raw >> 18) & 0x3fff;
    external = external << 2;
    external = external >> 2; // 符号拡張
    int16_t internal = (raw >> 4) & 0xfff;
    internal = internal << 4;
    internal = internal >> 4;

    m_InternalTemp = static_cast<float>(internal) * 0.0625; // LSB = 0.0625 degrees
    m_ExternalTemp = static_cast<float>(external) * 0.25; // LSB = 0.0625 degrees

    return Result_Ok;
}

void M5ReflowHAL::UpdateKeyStatus()
{
    int buttons[4];
    buttons[0] = gpio_get_level (GPIO_INPUT_BUTTON_L);
    buttons[1] = gpio_get_level (GPIO_INPUT_BUTTON_M);
    buttons[2] = gpio_get_level (GPIO_INPUT_BUTTON_R);
    buttons[3] = RawEncoderPush;
    for(int i=0;i<4;i++){
        if(buttons[i]==0 && m_KeyState[i]==false)
            m_KeyDown[i] = true;
        else
            m_KeyDown[i] = false;

        if(buttons[i]==0)
            m_KeyState[i] = true;
        else
            m_KeyState[i] = false;
    }
    m_EncoderDiff = RawEncoderDiff;
    RawEncoderDiff = 0;
}
