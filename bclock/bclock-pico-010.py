from machine import Pin, Timer
import time

rtc = machine.RTC()

# led = Pin(15, Pin.OUT)



"""
Binary Clock for Raspberry Pi Pico W

Re-name to "main.py" to auto-run on boot.

Note that running from thonny automatically sets the real-time clock on the pico. Running without does not.
"""

__author__ = "Todd Foster"
__version__ = "0.1.0"
__license__ = """
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

DEBUG = 1

PI_ZERO_PINOUT = """
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
lh_sequence = [0, 1, 2, 16, 17, 18, 19, 20]
rh_sequence = [3, 4, 5, 6, 7, 8, 9, 10, 11]

SECONDS_GPIOS = [0, 1, 2, 3, 4, 5]
MINUTES_GPIOS = [6, 7, 8, 9, 10, 11]
HOURS_GPIOS = [16, 17, 18, 19, 20]

PINS = [0] * 29

def init_gpios(gpios):
    for g in gpios:
        PINS[g] = Pin(g, Pin.OUT)
        PINS[g].off()

def settime (value, gpios):
    for i in range(len(gpios)):
        PINS[gpios[i]].value(bool(value & (1<<i)))

def on_change(thetime):
    hour = int(thetime[4])
    minute = int(thetime[5])
    second = int(thetime[6])
    if DEBUG > 0: print(f"DEBUG Time  = {thetime} = {hour}:{minute}:{second} = {bin(hour)}:{bin(minute)}:{bin(second)}")

    settime (hour, HOURS_GPIOS)
    settime (minute, MINUTES_GPIOS)
    settime (second, SECONDS_GPIOS)

def main():
    if DEBUG: print ("DEBUG: main")
    init_gpios(HOURS_GPIOS)
    init_gpios(MINUTES_GPIOS)
    init_gpios(SECONDS_GPIOS)

    # TODO: use timer instead of while True:
    #timer = Timer()
    #def blink(timer):
    #    led.toggle()
    #timer.init(freq=1.0, mode=Timer.PERIODIC, callback=blink)

    while True:
        oldtime = rtc.datetime()
        on_change(oldtime)
        while (oldtime == rtc.datetime()):
            time.sleep(0.1)

def test():
    init_gpios(HOURS_GPIOS)
    init_gpios(MINUTES_GPIOS)
    init_gpios(SECONDS_GPIOS)

    def cycle(gpios):
        for g in range(len(gpios)):
            if g > 0:
                PINS[gpios[g-1]].off()
            PINS[gpios[g]].on()
            if g < len(gpios)-1:
                time.sleep(0.2)
        for g in range(len(gpios)-1, -1, -1):
            if g < len(gpios)-1:
                PINS[gpios[g+1]].off()
            PINS[gpios[g]].on()
            if g > 0:
                time.sleep(0.2)
        time.sleep(0.2)
        PINS[gpios[0]].off()

    def all_set(status):
        for i in HOURS_GPIOS:
            PINS[i].value(status)
        for i in MINUTES_GPIOS:
            PINS[i].value(status)
        for i in SECONDS_GPIOS:
            PINS[i].value(status)

    if DEBUG: print ("TEST: All on")
    all_set(True)
    time.sleep(2)
    if DEBUG: print ("TEST: All off")
    all_set(False)
    time.sleep(1)
    if DEBUG: print ("TEST: All on")
    all_set(True)
    time.sleep(0.5)
    if DEBUG: print ("TEST: All off")
    all_set(False)
    
    if DEBUG: print ("TEST: Cycle seconds")
    for i in range(3):
        cycle(SECONDS_GPIOS)
    if DEBUG: print ("TEST: Cycle hours")
    for i in range(3):
        cycle(HOURS_GPIOS)
    if DEBUG: print ("TEST: Cycle minutes")
    for i in range(3):
        cycle(MINUTES_GPIOS)
    if DEBUG: print ("TEST: complete")
    

if __name__ == "__main__":
    if DEBUG: print ("DEBUG on")
    # test()
    main()
