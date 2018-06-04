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

#include "cJSON.h"
#include "json.h"

char* create_json_response_th(float temperature, float humidity, float outside) {
    char *string = NULL;
    cJSON *j_temp = NULL;
    cJSON *j_hum = NULL;
    cJSON *j_outside = NULL;
    cJSON *response = cJSON_CreateObject();

    j_temp = cJSON_CreateNumber(temperature);
    if( j_temp == NULL ) {
        goto end;
    }
    cJSON_AddItemToObject(response, "temperature", j_temp);

    j_hum = cJSON_CreateNumber(humidity);
    if( j_hum == NULL ) {
        goto end;
    }
    cJSON_AddItemToObject(response, "humidity", j_hum);

    j_outside = cJSON_CreateNumber(outside);
    if( j_outside == NULL ) {
        goto end;
    }
    cJSON_AddItemToObject(response, "outside_temperature", j_outside);
    
    if( response == NULL ) {
        goto end;
    }
    string = cJSON_Print(response);
    end:
        cJSON_Delete(response);
        return string;
}

char* create_json_response_relay(int state) {
    char *string = NULL;
    cJSON *j_state = NULL;
    cJSON *response = cJSON_CreateObject();

    j_state = cJSON_CreateBool(state);
    if( j_state == NULL ) {
        goto end;
    }
    cJSON_AddItemToObject(response, "status", j_state);
    string = cJSON_Print(response);
    end:
        cJSON_Delete(response);
        return string;
}