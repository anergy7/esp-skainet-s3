/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _SPEECH_COMMANDS_ACTION_H_
#define _SPEECH_COMMANDS_ACTION_H_

void led_Task(void *arg);

#include "led_strip.h"

void speech_commands_action(int command_id, led_strip_handle_t strip);

void wake_up_action(void);
#endif
