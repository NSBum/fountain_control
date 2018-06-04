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

/**
 *  @file main.c
 *  @author Alan K. Duncan (OjisanSeiuchi)
 *  @date 2018-06-04
 *  @brief Header file for the main file of fountain_control.
 *
 *  The main code file and application entry point for the
 *  fountain_control project.
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

/*  system */
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

//  drivers
#include "driver/gpio.h"
#include "driver/i2c.h"

//  network
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include "string.h"
#include <stdlib.h>
#include <stdio.h>

//  si7021 i2c temp/humidity component
#include "si7021.h"
#include "tm1637.h"
#include "json.h"
#include "outside.h"

//  wifi task flags
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

//  global temp and humidity
float temp,hum, outside_temp;

int relay_state;

extern EventGroupHandle_t owb_event_group;

static const char* TAG = "APP";

#define RELAY_CONTROL               (CONFIG_RELAY_CONTROL)

//  configuration parameters
#define EXAMPLE_ESP_WIFI_SSID       (CONFIG_ESP_WIFI_SSID)
#define EXAMPLE_ESP_WIFI_PASS       (CONFIG_ESP_WIFI_PASSWORD)
#define EXAMPLE_MAX_STA_CONN        (CONFIG_MAX_STA_CONN)
#define I2C_SDA                     (CONFIG_I2C_SDA)
#define I2C_SCL                     (CONFIG_I2C_SCL)
#define GPIO_TM1637_1_CLK           (CONFIG_TM1637_1_CLK)
#define GPIO_TM1637_1_DIO           (CONFIG_TM1637_1_DIO)
#define GPIO_INPUT_IO_1             5

#define ESP_INTR_FLAG_DEFAULT 0

//  TM1637-based 4 digit, seven-segment display
tm1637_led_t *led[2];

SemaphoreHandle_t buttonSemaphore = NULL;

//  http header
const static char http_html_hdr[] =
    "HTTP/1.1 200 OK\r\nContent-type: application/json\r\n\r\n";

const static char hdr_404[] = 
    "HTTP/1.1 404 Not Found\r\nContent-type: text/html\r\n\r\n";

void IRAM_ATTR button_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(buttonSemaphore,NULL);
}

void button_task(void *arg) {
    for(;;) {
        if( xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE ) {
            ESP_LOGI(TAG, "Button pressed");
            relay_state = !relay_state;
            gpio_set_level(RELAY_CONTROL, relay_state);
        }
    }
}

void init_button_interrupt(void) {
    buttonSemaphore = xSemaphoreCreateBinary();
    gpio_pad_select_gpio(GPIO_INPUT_IO_1);
    gpio_set_direction(GPIO_INPUT_IO_1, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_NEGEDGE);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, button_isr_handler, NULL);
}

esp_err_t init_i2c(void) {
    esp_err_t ret;

    // setup i2c controller
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100000;
    ret = i2c_param_config(I2C_NUM_0, &conf);
    if( ret != ESP_OK ) return ret;

    // install the driver
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}
      
//    event handler for wifi task
static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            printf("got ip\n");
            printf("netmask: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.netmask));
            printf("gw: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.gw));
            printf("\n");
            fflush(stdout);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialise_wifi(void) {
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void http_server_netconn_serve(struct netconn *conn) {
    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    /* Read the data from the port, blocking if nothing yet there.
    We assume the request (the part we care about) is in one netbuf */
    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**)&buf, &buflen);

        // strncpy(_mBuffer, buf, buflen);

        /* Is this an HTTP GET command? (only check the first 5 chars, since
        there are other formats for GET, and we're keeping it very simple )*/
        printf("buffer = %s \n", buf);
        if( buflen >= 5 && strstr(buf,"GET /") != NULL ) {
            printf("buf[5] = %c\n", buf[5]);

            //  Turn on the fountain
            if( buf[5]=='1' ) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
                ESP_LOGE(TAG,"Relay on");
                gpio_set_level(RELAY_CONTROL, 1);
                relay_state = true;
                /* Send our HTML page */
                char *str = create_json_response_relay(true);
                netconn_write(conn, str, strlen(str), NETCONN_NOCOPY);
            }
            //  Turn off the fountain
            else if( buf[5] == '0' ) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
                ESP_LOGE(TAG,"Relay off");
                gpio_set_level(RELAY_CONTROL, 0);
                relay_state = false;
                char *str = create_json_response_relay(false);
                netconn_write(conn, str, strlen(str), NETCONN_NOCOPY);
            }
            else if( buf[5] == 's' ) {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
                char *str = create_json_response_relay(relay_state);
                netconn_write(conn, str, strlen(str), NETCONN_NOCOPY);
            }
            else if( buf[5] == 'f' ) {
                netconn_write(conn, hdr_404, sizeof(hdr_404)-1, NETCONN_NOCOPY);
            }
            else {
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
                char *str = create_json_response_th(temp,hum, outside_temp);
                netconn_write(conn, str, strlen(str), NETCONN_NOCOPY);
            }
        }

    }
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
    so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

//  http server task
static void http_server(void *pvParameters) {
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

void query_sensor(void *pvParameter) {
    while(1) {
        temp = si7021_read_temperature();
        hum = si7021_read_humidity();
        
        tm1637_set_float(led[0],temp);
        
        printf("%0.2f degrees C, %0.2f%% RH\n", temp, hum);
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

//  application entry point
int app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initialise_wifi();
        
    //  initialize I2C driver/device
    ret = init_i2c();
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG,"I2C driver initialized");

    init_button_interrupt();

    // 
    gpio_set_direction(RELAY_CONTROL, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_CONTROL,0);
    ESP_LOGV(TAG,"Relay off at start");
    relay_state = false;
    
    //  initialize our displays
    led[0] = tm1637_init(GPIO_TM1637_1_CLK,GPIO_TM1637_1_DIO);

    // Allow bus to stabilize a bit before communicating
    vTaskDelay(2000.0 / portTICK_PERIOD_MS);

    xTaskCreate(&owb_search_task,"owbsearch",4096,NULL,5,NULL);
    xTaskCreate(&ds18b20_init_devices,"initdevives",4096,NULL,5,NULL);
    xTaskCreate(&http_server, "http server", 8000, NULL, 5, NULL);
    xTaskCreate(&query_sensor, "sensor_task", 2048, NULL, 5,NULL);
    xTaskCreate(&owb_get_temps,"gettemp",4096,&outside_temp,5,NULL);

    return 0;
}
