/************************************************************************************
*   Copyright (c) 2018 by Alan Duncan                                               *
*                                                                                   *
*   This file is part of fountain_control                                           *
*                                                                                   *
*   fountain_control is an ESP32-based system for local and                         *
*   web-based control of pond fountains on our rural residential                    *
*   property.                                                                       *
*   Permission is hereby granted, free of charge, to any person obtaining a copy    *
*   of this software and associated documentation files (the "Software"), to deal   *
*   in the Software without restriction, including without limitation the rights    *
*   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell       *
*   copies of the Software, and to permit persons to whom the Software is           *
*   furnished to do so, subject to the following conditions:                        *
*                                                                                   *
*   The above copyright notice and this permission notice shall be included in all  *
*   copies or substantial portions of the Software.                                 *
*                                                                                   *
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR      *
*   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,        *
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE     *
*   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER          *
*   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,   *
*   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE   *
*   SOFTWARE.                                                                       *
************************************************************************************/

#include "outside.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

static const char* MAINTAG = "OUTSIDE";

//  Array of pointers to DS18B20 devices
DS18B20_Info * devices[MAX_DEVICES] = {0};
//  Array of OneWire bus ROM codes
OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
//  The OneWire bus object
OneWireBus * owb;

//  opaque collection of flags
static EventGroupHandle_t owb_event_group;

int num_devices = 0;
float readings[MAX_DEVICES] = { 0 };

// flag bits in the event group
const int SEARCH_COMPLETE_BIT = BIT0;
const int DS18B20_SETUP_COMPLETE_BIT = BIT1;
const int DS18B20_TEMP_READY = BIT2;

void owb_search_task(void *pvParameters ) {
    ESP_LOGI(MAINTAG,"Will search for OWB devices.");
    owb_event_group = xEventGroupCreate();
    if( owb_event_group == NULL ) {
        ESP_LOGE(MAINTAG,"Unable to create owb_event_group");
        return;
    }
    xEventGroupClearBits(owb_event_group, SEARCH_COMPLETE_BIT);

    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);

    owb_use_crc(owb, true);              // enable CRC check for ROM code

    // Find all connected devices
    ESP_LOGI(MAINTAG,"Finding devices");
    
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while( found ) {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    ESP_LOGI(MAINTAG,"Found %d devices", num_devices);

    xEventGroupSetBits(owb_event_group, SEARCH_COMPLETE_BIT);
    vTaskDelete(NULL);
}

void ds18b20_init_devices(void *pvParameters) {
    ESP_LOGI(MAINTAG,"Will init DS18b20 device(s)");
    xEventGroupClearBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT);
    xEventGroupWaitBits(owb_event_group, SEARCH_COMPLETE_BIT, 1, 0, portMAX_DELAY);
    ESP_LOGI(MAINTAG,"OWB search complete wait done");
    for( int i = 0; i < num_devices; ++i ) {
        devices[i] = ds18b20_malloc();;
        ds18b20_init(devices[i], owb, device_rom_codes[i]); // associate with bus and device
        ds18b20_use_crc(devices[i], true);           // enable CRC check for temperature readings
        ds18b20_set_resolution(devices[i], DS18B20_RESOLUTION);
    }
    xEventGroupSetBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT);
    vTaskDelete(NULL);
}

void owb_get_temps(void *pvParameters) {
    //  wait for bus scan to complete
    ESP_LOGI(MAINTAG,"Will get outside temp");
    xEventGroupWaitBits(owb_event_group, DS18B20_SETUP_COMPLETE_BIT, 1, 0, portMAX_DELAY);

    ESP_LOGI(MAINTAG,"Ready to get outside temp");
    ESP_LOGI(MAINTAG,"Number of devices = %d",num_devices);
    // Read temperatures more efficiently by starting conversions on all devices at the same time
    int errors_count[MAX_DEVICES] = {0};
    int sample_count = 0;
    if (num_devices > 0) {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1) {
            xEventGroupClearBits(owb_event_group, DS18B20_TEMP_READY);
            last_wake_time = xTaskGetTickCount();

            ds18b20_convert_all(owb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices[0]);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices; ++i) {
                errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
            }

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            for (int i = 0; i < num_devices; ++i) {
                if (errors[i] != DS18B20_OK) {
                    ++errors_count[i];
                }
                char rom_code_s[17];
                owb_string_from_rom_code(device_rom_codes[i], rom_code_s, sizeof(rom_code_s));
                printf("  %d - %s: %.2f    %d errors\n", i, rom_code_s, readings[i], errors_count[i]);
            }
            xEventGroupSetBits(owb_event_group, DS18B20_TEMP_READY);

            //  the pvParameters is a pointer to a float outside_temp
            *(float *)pvParameters = readings[0];

            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
}