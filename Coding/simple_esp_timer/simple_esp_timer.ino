/* Copyright (c) 2017 pcbreflux. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 *
 */
hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer(){
  Serial.println(String("onTimer() ")+String(millis()));
}

void setup() {
  Serial.begin(115200);

  Serial.println("start timer ");
  // Original code: timer = timerBegin(0, 80, true); // timer 0, divider 80, countUp
  // Equivalent in your API: Sets timer tick frequency. 80MHz / 80 = 1MHz = 1,000,000 Hz
  timer = timerBegin(1000000);

  // Original code: timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered
  // This line is likely correct for your API
  timerAttachInterrupt(timer, &onTimer);

  // Original code: timerAlarmWrite(timer, 1000000, true); // 1000000 * 1 us = 1 s, autoreload true
  // Original code: timerAlarmEnable(timer); // enable
  // Equivalent in your API: Combined function to set alarm value, repeat, count, AND enable.
  // 1,000,000 is the alarm value (in ticks, which are 1us each because frequency is 1MHz)
  // true means autoreload
  // 0 means unlimited repeats
  timerAlarm(timer, 1000000, true, 0);
}

void loop() {
  // nope nothing here
  vTaskDelay(portMAX_DELAY); // wait as much as posible ...
}