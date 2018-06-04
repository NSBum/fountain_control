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
 *  @file outside.h
 *  @author Alan K. Duncan (OjisanSeiuchi)
 *  @date 2018-06-04
 *  @brief Header file for the outside temperature measurement for fountain_control.
 *
 *  The outside temperature measurement subsystem is an add-on feature for the
 *  fountain_control project giving weather station functionality.
 */

#ifndef OUTSIDE_H
#define OUTSIDE_H

#ifdef __cplusplus
extern "C" {
#endif

//  one wire bus/ds18b20
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define MAX_DEVICES                 (8)
#define DS18B20_RESOLUTION          (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD               (1000)   // milliseconds betwen temperature measurements
#define GPIO_DS18B20_0              (CONFIG_ONE_WIRE_GPIO)      //  set in configuration menu

/**
 *  @brief Searches for OneWire devices.
 *
 *  A FreeRTOS task launched at startup that searches for OneWire devices on the
 *  designated OneWire bus. On completion of the search, this function sets the 
 *  `SEARCH_COMPLETE_BIT` of the `owb_event_group` flags.
 *  @param pvParameters unused
 *  @return none
 */
void owb_search_task(void *pvParameters );

/**
 *  @brief Initializes any DS18B20 devices on the OneWire bus
 *
 *  A FreeRTOS task that should be launched on startup that initializes any
 *  DS18B20 devices found on the OneWire bus. The task waits for the `SEARCH_COMPLETE_BIT`
 *  `owb_event_group` flags and on completion sets the `DS18B20_SETUP_COMPLETE_BIT`
 *  of the same flags.
 *  @param pvParameters unused
 *  @return none
 */
void ds18b20_init_devices(void *pvParameters);

/**
 *  @brief Reads temperature(s) from DS18B20 device(s)
 *
 *  A FreeRTOS task that periodically reads temperatures from any
 *  DS18B20 devices found on the OneWire bus.
 *  The task waits for the `DS18B20_SETUP_COMPLETE_BIT` of the `owb_event_group` flags
 *  to be set before iteratively querying the found devices.
 *  @param pvParameters pointer to a float to be populated with the temperature of the first device.
 *  @return none
 */
void owb_get_temps(void *pvParameters);

#endif