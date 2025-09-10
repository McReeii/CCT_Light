#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <u8g2.h>
#include "u8g2_esp32_hal.h"
#include "esp_lcd_panel_ssd1306.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BATTERY_ADC            ADC_CHANNEL_3
#define BATTERY_ADC_ATTEN      ADC_ATTEN_DB_12
static int adc_raw[2][10];
static int voltage[2][10];
uint16_t batt_voltage;
// static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
// static void example_adc_calibration_deinit(adc_cali_handle_t handle);

#define OLED_SDA_I2C           6
#define OLED_SCL_I2C           7
#define OLED_I2C_HW_ADDR       0x3C

#define NEXT_BTN               0
#define PRV_BTN                1
#define SEL_BTN                5
#define BUTTON_ACTIVE_LEVEL    0

//extern void example_lvgl_demo_ui(lv_disp_t *disp);

#define LEDC_HS_TIMER          LEDC_TIMER_2//5200K
#define LEDC_HS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_HS_CH0_GPIO       (21) 
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (18)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#define LEDC_HS_CH2_GPIO       (23)
#define LEDC_HS_CH2_CHANNEL    LEDC_CHANNEL_2

#define LEDC_LS_TIMER          LEDC_TIMER_1 //3200K
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_LS_CH3_GPIO       (22)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3
#define LEDC_LS_CH4_GPIO       (19)
#define LEDC_LS_CH4_CHANNEL    LEDC_CHANNEL_4     /*!< LEDC channel 4 */
#define LEDC_LS_CH5_GPIO       (20)
#define LEDC_LS_CH5_CHANNEL    LEDC_CHANNEL_5     /*!< LEDC channel 5 */

#define LEDC_TEST_CH_NUM       (3)
#define LEDC_TEST_DUTY         (1000 ) //Duty cycle
#define LEDC_TEST_FADE_TIME    (1000) //4 secound to fade

bool LedMasterSel =            0; //For selaction beetween 3200K and 5200k FOR MODE-1 [0 = 3200k, 1 = 5200k]
uint16_t LowLedCDuty =         0; //Mode-1 LedC Low Kelvin duty
uint16_t HighLedCDuty =        0; //Mode-1 ledC High Kelvin duty

uint16_t LowKelvinDuty =       0; //Mode-2 LedC Low Kelvin duty
uint16_t HighKelvinDuty =      0; //Mode-2 LedC High Kelvin duty
uint8_t GlobalLedDuty =        0;
                                 
uint16_t SparkleFade =         10; //Mode-3 LedC led fade time for sparkle smoothness control
uint16_t SparkleSpeed =        20; //Mode-3 LedC led channel change delay time for sparkle speed control

uint16_t UserINKelvin =        5600;
uint16_t GlobalKelvinDuty =    0;
uint16_t CurrentKelvin =       4200;

uint8_t BattGauge;
uint8_t mode =                 1;
bool btn_pressed =             0;
bool CCT_ERR =                 false;

static uint32_t map(uint32_t data_in, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
    return (data_in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static const char*  OLED_TAG =  "Display";

/*
 * This callback function will be called when fade operation has ended
 * Use callback only if you are aware it is being called inside an ISR
 * Otherwise, you can use a semaphore to unblock tasks
 */
static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t) user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

static void SerialPrintLog(void)
{



}

static void LedDutyLogic(void)
{
    HighKelvinDuty = map(UserINKelvin, 3100, 5700, 0, 1000 );
    LowKelvinDuty = 1000-HighKelvinDuty;

    //btn_pressed =1;
    //if(HighKelvinDuty < 70 || LowKelvinDuty < 70){CCT_ERR = true; printf("7. CCT Error!\n");}
}

// static void BatteryLvl(void){
// }

//------------------------------------------------BUTTON_CALLBACK------------------------------------------
static void sel_button_event_cb(void *arg,void *usr_data)
{   printf("6. Set_Btn Pressed!\n");

    mode ++;
    if(mode > 3){mode = 1;}

    btn_pressed = 1;
    SerialPrintLog();
}

static void sel_button_double_click_event_cb(void *arg,void *usr_data)
{   printf("7. Sel Button Double Click!\n");
    switch (mode)
    {
    case 1:
    LedMasterSel = !LedMasterSel;

    break;
    case 2:
    //code
    break;
    case 3:
    LedMasterSel = !LedMasterSel;

    break;

    default:
    break;

    }
    SerialPrintLog();
}

static void next_button_event_cb(void *arg,void *usr_data)
{   printf("4. Next_Btn Pressed!\n");
    
    switch (mode)
    {
    case 1:
    if(LedMasterSel == 1){
        if(LowLedCDuty > 0){
            LowLedCDuty = LowLedCDuty - 100;
        }
    }
    if(LedMasterSel == 0){
        if(HighLedCDuty > 0){
            HighLedCDuty = HighLedCDuty - 100;
        }
    }
    break;
    case 2:
    if(UserINKelvin > 3200){
        UserINKelvin = UserINKelvin - 100;
    }
    break;
    case 3:
    if(LedMasterSel == 0){
        if(SparkleFade > 10){
            SparkleFade = SparkleFade - 10;
        }
    }
    if(LedMasterSel == 1){
        if(SparkleSpeed > 20){
            SparkleSpeed = SparkleSpeed - 20;
        }
    }
    break;

    default:
    break;
    }

    SerialPrintLog();
    btn_pressed = 1;
}

static void prv_button_event_cb(void *arg,void *usr_data)
{   printf("5. Prv_Btn Pressed!\n");

    switch (mode)
    {
    case 1:
    if(LedMasterSel == 1){
        if(LowLedCDuty < 1000){
            LowLedCDuty = LowLedCDuty + 100;
        }
    }
    if(LedMasterSel == 0){
        if(HighLedCDuty < 1000){
            HighLedCDuty = HighLedCDuty + 100;
        }
    }
    break;
    case 2:
    if(UserINKelvin < 5600){
        UserINKelvin = UserINKelvin + 100;
    }
    break;
    case 3:
    if(LedMasterSel == 0){
        if(SparkleFade < 1000){
            SparkleFade = SparkleFade + 10;
        }
    }
    if(LedMasterSel == 1){
        if(SparkleSpeed < 1000){
            SparkleSpeed = SparkleSpeed + 20;
        }
    }
    break;
    
    default:
    break;
    }


    SerialPrintLog();
    btn_pressed = 1;
}

//------------------------------------------------CREATE_BUTTON------------------------------------------
void sel_button_init(uint32_t button_num)
{
    button_config_t sel_btn_cfg = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };
    button_gpio_config_t sel_btn_gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = true,
    };

    button_handle_t sel_btn;
    esp_err_t sel_ret = iot_button_new_gpio_device(&sel_btn_cfg, &sel_btn_gpio_cfg, &sel_btn);
    assert(sel_ret == ESP_OK);

    sel_ret |= iot_button_register_cb(sel_btn, BUTTON_SINGLE_CLICK, NULL, sel_button_event_cb, NULL);
    sel_ret |= iot_button_register_cb(sel_btn, BUTTON_PRESS_REPEAT, NULL, sel_button_double_click_event_cb, NULL);


    ESP_ERROR_CHECK(sel_ret);
}
void next_button_init(uint32_t button_num)
{
    button_config_t next_btn_cfg = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };
    button_gpio_config_t next_btn_gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = true,
    };

    button_handle_t next_btn;
    esp_err_t next_ret = iot_button_new_gpio_device(&next_btn_cfg, &next_btn_gpio_cfg, &next_btn);
    assert(next_ret == ESP_OK);

    next_ret = iot_button_register_cb(next_btn, BUTTON_PRESS_DOWN, NULL, next_button_event_cb, NULL);

    ESP_ERROR_CHECK(next_ret);
}
void prv_button_init(uint32_t button_num)
{
    button_config_t prv_btn_cfg = {
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    };
    button_gpio_config_t prv_btn_gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = true,
    };

    button_handle_t prv_btn;
    esp_err_t prv_ret = iot_button_new_gpio_device(&prv_btn_cfg, &prv_btn_gpio_cfg, &prv_btn);
    assert(prv_ret == ESP_OK);

    prv_ret = iot_button_register_cb(prv_btn, BUTTON_PRESS_DOWN, NULL, prv_button_event_cb, NULL);

    ESP_ERROR_CHECK(prv_ret);
}

//------------------------------------------------MAIN_APP------------------------------------------
void app_main(void)
{
    int test = 5276; //intiger created
    char test_char[sizeof(test)*8+1]; //char buffer created hold value of above created intiger
    snprintf(test_char, sizeof(test_char), "%d", test); //converts intiger to string and writes in above crated char buffer

    //------------------------------------BUTTON_INIT----------------------------

    next_button_init(NEXT_BTN);
    prv_button_init(PRV_BTN);
    sel_button_init(SEL_BTN);

   //------------------------------------U8G2_INIT----------------------------
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = OLED_SDA_I2C;
    u8g2_esp32_hal.bus.i2c.scl = OLED_SCL_I2C;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
  
    u8g2_t u8g2;  // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2, U8G2_R0,
        // u8x8_byte_sw_i2c,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
  
    ESP_LOGI(OLED_TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2);  // send init sequence to the display, display is in
                              // sleep mode after this,
  
    ESP_LOGI(OLED_TAG, "u8g2_SetPowerSave");
    u8g2_SetPowerSave(&u8g2, 0);  // wake up display
    u8g2_SetDrawColor(&u8g2,1);
    u8g2_SetFontMode(&u8g2,1);
    ESP_LOGI(OLED_TAG, "u8g2_ClearBuffer");
    u8g2_SetContrast(&u8g2,0x10);

    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawBox(&u8g2, 0, 0, 2, 32);
    u8g2_DrawFrame(&u8g2, 0, 0, 128, 32);

    u8g2_SetFont(&u8g2, u8g2_font_helvR14_tn);
    //u8g2_SetFont(&u8g2, u8g2_font_open_iconic_weather_2x_t);
    //u8g2_SetFont(&u8g2, u8g2_font_profont22_tn);
    
    u8g2_DrawStr(&u8g2, 5, 22, test_char);
    u8g2_SendBuffer(&u8g2);

    //---------------------------------------------ADC_INIT-------------------------------

                //-------------ADC1 Init---------------
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

                //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC, &config));

                //-------------ADC1 Calibration Init---------------//
    // adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    // adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    // bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, BATTERY_ADC, BATTERY_ADC_ATTEN, &adc1_cali_chan0_handle);


    //---------------------------------------------LEDC INIT-------------------------------
    int ch;
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 40000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);

#endif
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel_5600K[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER,
            .flags.output_invert = 1
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER,
            .flags.output_invert = 1
        },
        {
            .channel    = LEDC_HS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH2_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER,
            .flags.output_invert = 1
        },
    };

    ledc_channel_config_t ledc_channel_3200K[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1
        },
        {
            .channel    = LEDC_LS_CH4_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH4_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1
        },
        {
            .channel    = LEDC_LS_CH5_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH5_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 1
        },
    };

    // Set LED Controller with previously prepared configuration //5600K
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel_5600K[ch]);
    }

    // Set LED Controller with previously prepared configuration //3200K
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel_3200K[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
        .fade_cb = cb_ledc_fade_end_event
    };
    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(LEDC_TEST_CH_NUM, 0);

    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_cb_register(ledc_channel_5600K[ch].speed_mode, ledc_channel_5600K[ch].channel, &callbacks, (void *) counting_sem);
    }

    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_cb_register(ledc_channel_3200K[ch].speed_mode, ledc_channel_3200K[ch].channel, &callbacks, (void *) counting_sem);
    }

    //btn_pressed = 1;
    //-----------------------------------------------------------
    while (1){
        LedDutyLogic();
        // char test_char[sizeof(mode)*8+1]; //char buffer created hold value of above created intiger
        // snprintf(test_char, sizeof(test_char), "%d", mode); //converts intiger to string and writes in above crated char buffer
    
        // char c_mode[sizeof(mode)*8+1]; //char buffer created hold value of above created intiger
        // snprintf(c_mode, sizeof(c_mode), "%d", mode); //converts intiger to string and writes in above crated char buffer
        
        char c_UserINKelvinyt[sizeof(LowKelvinDuty)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_UserINKelvinyt, sizeof(c_UserINKelvinyt), "%d", LowKelvinDuty); //converts intiger to string and writes in above crated char buffer
      
        char c_UserINKelviny[sizeof(UserINKelvin)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_UserINKelviny, sizeof(c_UserINKelviny), "%d", UserINKelvin); //converts intiger to string and writes in above crated char buffer
       
 
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFontDirection(&u8g2,0);
        u8g2_SetFont(&u8g2, u8g2_font_helvR14_tr);
        u8g2_DrawLine(&u8g2,45,0,45,32);
        u8g2_DrawLine(&u8g2,105,0,105,32);
        switch (mode)
        {
        case 1:
        char c_HighLedCDuty[sizeof(HighLedCDuty)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_HighLedCDuty, sizeof(c_HighLedCDuty), "%d", HighLedCDuty/10); //converts intiger to string and writes in above crated char buffer

        char c_LowLedCDuty[sizeof(LowLedCDuty)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_LowLedCDuty, sizeof(c_LowLedCDuty), "%d", LowLedCDuty/10); //converts intiger to string and writes in above crated char buffer
        
        u8g2_DrawStr(&u8g2, 0, 32, c_HighLedCDuty);
        u8g2_DrawStr(&u8g2, 52, 32, c_LowLedCDuty);

        u8g2_SetFont(&u8g2, u8g2_font_helvR10_tr);    
        if(LedMasterSel == 0){u8g2_DrawStr(&u8g2, 52, 14, "5600K");}
        else {u8g2_DrawStr(&u8g2, 52, 14, "3200K");}


        u8g2_SetFont(&u8g2, u8g2_font_twelvedings_t_all);
        if(LedMasterSel == 0){u8g2_DrawGlyph(&u8g2,32,30,0x003C);}// "<" icon
        else {u8g2_DrawGlyph(&u8g2,85,30,0x003C);}
        if (CCT_ERR == true) {u8g2_DrawGlyph(&u8g2,27,14,0x0021);}//err icon


        u8g2_SetFont(&u8g2, u8g2_font_open_iconic_weather_2x_t);
        u8g2_DrawGlyph(&u8g2,0,16,0x0045); //mode icon
        break;

        case 2:
        char c_GlobalLedDuty[sizeof(GlobalLedDuty)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_GlobalLedDuty, sizeof(c_GlobalLedDuty), "%d", GlobalLedDuty); //converts intiger to string and writes in above crated char buffer
        
        u8g2_DrawStr(&u8g2, 0, 32, c_GlobalLedDuty);
        u8g2_DrawStr(&u8g2, 52, 32, c_UserINKelviny);

        u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);
        u8g2_DrawStr(&u8g2, 52, 12, "KELVIN");

        u8g2_SetFont(&u8g2, u8g2_font_twelvedings_t_all);
        if (CCT_ERR == true) {u8g2_DrawGlyph(&u8g2,27,14,0x0021);}//err icon

        u8g2_SetFont(&u8g2, u8g2_font_open_iconic_weather_2x_t);
        u8g2_DrawGlyph(&u8g2,0,16,0x0041);
        
        break;

        case 3:

        char c_SparkleFade[sizeof(SparkleFade)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_SparkleFade, sizeof(c_SparkleFade), "%d", SparkleFade/10); //converts intiger to string and writes in above crated char buffer
        
        char c_SparkleSpeed[sizeof(SparkleSpeed)*8+1]; //char buffer created hold value of above created intiger
        snprintf(c_SparkleSpeed, sizeof(c_SparkleSpeed), "%d", SparkleSpeed/10); //converts intiger to string and writes in above crated char buffer
       
        if (LedMasterSel == 1){u8g2_DrawStr(&u8g2, 0, 32, c_SparkleSpeed);}
        else{u8g2_DrawStr(&u8g2, 0, 32, c_SparkleFade);}
        
        u8g2_DrawStr(&u8g2, 52, 32, c_UserINKelviny );

        u8g2_SetFont(&u8g2, u8g2_font_helvR10_tr);
        if (LedMasterSel == 1){u8g2_DrawStr(&u8g2, 52, 14, "SPEED");}
        else {u8g2_DrawStr(&u8g2, 52, 14, "FADE");}

        u8g2_SetFont(&u8g2, u8g2_font_twelvedings_t_all);
        if (CCT_ERR == true) {u8g2_DrawGlyph(&u8g2,27,14,0x0021);}//err icon

        u8g2_SetFont(&u8g2, u8g2_font_open_iconic_weather_2x_t); 
        u8g2_DrawGlyph(&u8g2,0,16,0x0044);
       
        break;
        
        default:
            break;
        }

        batt_voltage = map(adc_raw[0][1], 1000, 1890, 47, 54);

        // char c_soc_temp[sizeof(adc_raw[0][1])*8+1]; //char buffer created hold value of above created intiger
        // snprintf(c_soc_temp, sizeof(c_soc_temp), "%d", Soc_Temp); //converts intiger to string and writes in above crated char buffer
        

        // u8g2_SetFont(&u8g2, u8g2_font_helvR08_tr);
        // u8g2_DrawStr(&u8g2, 109, 20, Soc_Temp);       


        u8g2_SetFontDirection(&u8g2,1);
        u8g2_SetFont(&u8g2, u8g2_font_battery19_tn);
        u8g2_DrawGlyph(&u8g2,108,0,batt_voltage); //batt icon
        u8g2_SendBuffer(&u8g2);
        
        switch (mode)
        {
        case 1: //Brightness Control
        //if(btn_pressed == 1){
            //TURNS both color LED ON- OFF same time
            printf("1. Global Brighntss = %d\n", LowLedCDuty);
            for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
                ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                        ledc_channel_5600K[ch].channel, HighLedCDuty, LEDC_TEST_FADE_TIME);
                ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                                ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

                ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                        ledc_channel_3200K[ch].channel, LowLedCDuty, LEDC_TEST_FADE_TIME);
                ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                                ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);
            }
            for (int i = 0; i < LEDC_TEST_CH_NUM; i++) {
                xSemaphoreTake(counting_sem, portMAX_DELAY);
            }
        //}
        break;
        case 2: //Kelvin Control
        //if(btn_pressed == 1){
            //TURNS both color LED ON- OFF same time
            printf("2. Global Color Temprature = %d\n", GlobalLedDuty);
            for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
                ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                        ledc_channel_5600K[ch].channel, HighKelvinDuty, LEDC_TEST_FADE_TIME);
                ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                                ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

                ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                        ledc_channel_3200K[ch].channel, LowKelvinDuty, LEDC_TEST_FADE_TIME);
                ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                                ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);
            }
            for (int i = 0; i < LEDC_TEST_CH_NUM; i++) {
                xSemaphoreTake(counting_sem, portMAX_DELAY);
            }
        //}
        break;
        case 3:
            //TURNS both color LED ON- OFF same time
            printf("1. LEDC 5600K-3200K fade up to duty = %d\n", LEDC_TEST_DUTY);
            for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
                ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                        ledc_channel_5600K[ch].channel, HighKelvinDuty, SparkleFade);
                ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                                ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

                ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                        ledc_channel_3200K[ch].channel, LowKelvinDuty, SparkleFade);
                ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                                ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);

                vTaskDelay(SparkleSpeed / portTICK_PERIOD_MS);

                ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                        ledc_channel_5600K[ch].channel, 0, SparkleFade);
                ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                                ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

                ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                        ledc_channel_3200K[ch].channel, 0, SparkleFade);
                ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                                ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);
                
                
            }

            for (int i = 0; i < LEDC_TEST_CH_NUM; i++) {
                xSemaphoreTake(counting_sem, portMAX_DELAY);
            }
        break;
        case 5:
        //TURNS both color LED ON- OFF same time
        printf("1. LEDC 5600K-3200K fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                    ledc_channel_5600K[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                            ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

            ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                    ledc_channel_3200K[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                            ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);
        }

        for (int i = 0; i < LEDC_TEST_CH_NUM; i++) {
            xSemaphoreTake(counting_sem, portMAX_DELAY);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);

        printf("2. LEDC 5600K-3200K fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel_5600K[ch].speed_mode,
                                    ledc_channel_5600K[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel_5600K[ch].speed_mode,
                            ledc_channel_5600K[ch].channel, LEDC_FADE_NO_WAIT);

            ledc_set_fade_with_time(ledc_channel_3200K[ch].speed_mode,
                                    ledc_channel_3200K[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel_3200K[ch].speed_mode,
                            ledc_channel_3200K[ch].channel, LEDC_FADE_NO_WAIT);
        }

        for (int i = 0; i < LEDC_TEST_CH_NUM; i++) {
            xSemaphoreTake(counting_sem, portMAX_DELAY);
        }
        break;
        default:
            break;
        }

        //btn_pressed = 0;

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC, &adc_raw[0][1]));
        
        //ESP_LOGI("BATT_ADC", "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BATTERY_ADC, adc_raw[0][1]);
        // if (do_calibration1_chan1) {
        //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        //     ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BATTERY_ADC, voltage[0][1]);
        // }



    }//While(1)
}//App_Ma./