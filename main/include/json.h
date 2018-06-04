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
 *  @file json.h
 *  @author Alan K. Duncan (OjisanSeiuchi)
 *  @date 2018-06-04
 *  @brief Header file for the json subsystem for fountain_control.
 *
 *  Provides json for the web server piece of this project. The functions in this
 *  part of the project return json representation of data to be reported back
 *  to web clients.
 */

#ifndef JSON_H
#define JSON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

/**
 *  @brief Returns JSON representation of temperature, humidity and outside temperature
 *  
 *  Returns a string of JSON reporting the inside temperature and humidity along with
 *  the outside temperature.
 *  @param temperature the inside temperature
 *  @param humidity the inside humidity
 *  @param outside the outside temperature measurement.
 *  @returns string of JSON
*/
char* create_json_response_th(float temperature, float humidity, float outside);

/**
 *  @brief Returns JSON representation of the relay state
 *  
 *  Returns a string of JSON reporting the state of the fountain relay.
 *
 *  @param state 0 if the fountain is off or 1 if the fountain is on
 *  @returns string of JSON
*/
char* create_json_response_relay(int state);

#endif