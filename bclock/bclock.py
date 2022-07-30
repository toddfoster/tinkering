#!/usr/bin/env python3
"""
Binary Clock for Raspberry Pi
"""

__author__ = "Todd Foster"
__version__ = "0.1.0"
__license__ = """
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

import RPi.GPIO as GPIO
import time

DEBUG = 0

PINOUT = """
     3V3  (1) (2)  5V
   GPIO2  (3) (4)  5V
   GPIO3  (5) (6)  GND
   GPIO4  (7) (8)  GPIO14
     GND  (9) (10) GPIO15
  GPIO17 (11) (12) GPIO18
  GPIO27 (13) (14) GND
  GPIO22 (15) (16) GPIO23
     3V3 (17) (18) GPIO24
  GPIO10 (19) (20) GND
   GPIO9 (21) (22) GPIO25
  GPIO11 (23) (24) GPIO8
     GND (25) (26) GPIO7
   GPIO0 (27) (28) GPIO1
   GPIO5 (29) (30) GND
   GPIO6 (31) (32) GPIO12
  GPIO13 (33) (34) GND
  GPIO19 (35) (36) GPIO16
  GPIO26 (37) (38) GPIO20
     GND (39) (40) GPIO21
  """

# GPIO's are arrays from least significant bit to greatest
lh_sequence = [17, 27, 22, 5, 6, 13, 19, 26]
rh_sequence = [23, 24, 25, 8, 7, 12, 16, 20, 21]

SECONDS_GPIOS = [17, 27, 22, 23, 24, 25]
MINUTES_GPIOS = [8, 7, 12, 16, 20, 21]
HOURS_GPIOS = [5, 6, 13, 19, 26]

def init_gpios(gpios):
    for g in gpios:
        GPIO.setup(g, GPIO.OUT)
        GPIO.output(g, GPIO.LOW)
    
def set_led(gpio, on):
    if DEBUG > 5: print(f"DEBUG: set_led {gpio} to {on}")
    if on:
        GPIO.output(gpio, GPIO.HIGH)
    else:
        GPIO.output(gpio, GPIO.LOW)

def settime (value, gpios):
    for i in range(len(gpios)):
        set_led(gpios[i], bool(value & (1<<i)))

def on_change(thetime):
    hour = int(thetime[11:13])
    minute = int(thetime[14:16])
    second = int(thetime[17:19])
    if DEBUG > 0: print(f"DEBUG Time  = {thetime} = {hour}:{minute}:{second} = {bin(hour)}:{bin(minute)}:{bin(second)}")

    settime (hour, HOURS_GPIOS)
    settime (minute, MINUTES_GPIOS)
    settime (second, SECONDS_GPIOS)

def main():
    init_gpios(HOURS_GPIOS)
    init_gpios(MINUTES_GPIOS)
    init_gpios(SECONDS_GPIOS)

    while True:
        oldtime = time.ctime()
        on_change(oldtime)
        while (oldtime == time.ctime()):
            time.sleep(0.1)

def test():
    init_gpios(HOURS_GPIOS)
    init_gpios(MINUTES_GPIOS)
    init_gpios(SECONDS_GPIOS)

    def cycle(gpios):
        for g in range(len(gpios)):
            if g > 0:
                set_led(gpios[g-1], False)
            set_led(gpios[g], True)
            if g < len(gpios)-1:
                time.sleep(0.2)
        for g in range(len(gpios)-1, -1, -1):
            if g < len(gpios)-1:
                set_led(gpios[g+1], False)
            set_led(gpios[g], True)
            if g > 0:
                time.sleep(0.2)
        time.sleep(0.2)
        set_led(gpios[0], False)

    def all_set(status):
        for i in HOURS_GPIOS:
            set_led(i, status)
        for i in MINUTES_GPIOS:
            set_led(i, status)
        for i in SECONDS_GPIOS:
            set_led(i, status)

    all_set(True)
    time.sleep(2)
    all_set(False)
    time.sleep(1)
    all_set(True)
    time.sleep(0.5)
    all_set(False)
    
    for i in range(3):
        cycle(SECONDS_GPIOS)
    for i in range(3):
        cycle(HOURS_GPIOS)
    for i in range(3):
        cycle(MINUTES_GPIOS)


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    test()
    main()
