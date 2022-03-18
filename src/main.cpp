// #include <Arduino.h>
// #include "time.hpp"
// // #include <pixy2.hpp>
// // #include "spi.hpp"
// #include "i2c.hpp"


// timeout send_data { msec(1000) }; // timeout zajistuje posilani dat do PC kazdych 500 ms
// bool state_L_G = true;

// void setup() {
//     Serial.begin(115200);
//     pinMode(L_G, OUTPUT);
    
//     send_data.restart();
// }


// void loop() {
//     if (send_data)
//     {
//         send_data.ack();
//         if (state_L_G) state_L_G = false; else  state_L_G = true;
//         digitalWrite(L_G, state_L_G);
//         Serial.println( millis() );
       
//     }
//     delay(10);



// }

// *****************************************************************************
#include <esp_log.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include <pixy2.hpp>
// #include "spi.hpp"
#include "i2c.hpp"

#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_NUM_5 )
void iopins_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

extern "C" void app_main()  // example for connect ESP32 with I2C
{

    iopins_init();

    // The I2C address of the TCS34725 is 0x29: each write message therefore starts with the 0x52 byte and each read message starts by 0x53
    auto linkRes = LinkI2C::withBusInit(I2C_NUM_0, 0x29, GPIO_NUM_18, GPIO_NUM_19);

    int i = 0;
    gpio_set_level(GPIO_NUM_5, 1);        // pin 5 - LED na senzoru TCS34725
    while (true)
    {
        // pojede, dokud se nepřepíše výchozí  nastavení UARTu; totéž printf   
        ESP_LOGI("TAG1", "%i \n", i++);   // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html
        vTaskDelay(pdMS_TO_TICKS(500));   



    }
}
