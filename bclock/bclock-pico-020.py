from machine import Pin, Timer
import time

rtc = machine.RTC()

# led = Pin(15, Pin.OUT)



"""
Binary Clock for Raspberry Pi Pico W

Re-name to "main.py" to auto-run on boot.

Note that running from thonny automatically sets the real-time clock on the pico. Running without does not.

0.1.0 : Proof of concept
0.2.0 : Buttons to set hours, minutes, orientation; arbitrary number possible orientations

"""

__author__ = "Todd Foster"
__version__ = "0.2.0"
__license__ = """
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

DEBUG = 1
MODE = "init"

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

PI_PICO_PINOUT = """
     GP0  (1) (40) VBUS
     GP1  (2) (39) VYSS
     GND  (3) (38) GND
     GP2  (4) (37) 3V3_EN
     GP3  (5) (36) 3V3(OUT)
     GP4  (6) (35) ADC_VREF
     GP5  (7) (34) GP28
     GND  (8) (33) GND
     GP6  (9) (32) GP27
     GP7 (10) (31) GP26
     GP8 (11) (30) RUN
     GP9 (12) (29) GP22
     GND (13) (28) GND
    GP10 (14) (27) GP21
    GP11 (15) (26) GP20
    GP12 (16) (25) GP19
    GP13 (17) (24) GP18
     GND (18) (23) GND
    GP14 (19) (22) GP17
    GP15 (20) (21) GP16

GP25 is the LED on the Pico board

"""



class LED_Addresses:
    def increment_orientation(self):
        self.current_orientation += 1
        if self.current_orientation >= len(self.ORIENTATIONS):
            self.current_orientation = 0
        self.seconds = self.ORIENTATIONS[self.current_orientation][0:6]
        self.minutes = self.ORIENTATIONS[self.current_orientation][6:12]
        self.hours   = self.ORIENTATIONS[self.current_orientation][12:18]
    def __init__(self):
        # Orientation: each orientation is an array of 18 values from least significant bit to most significant (i.e., seconds-minutes-hours)
        # Edit these to accomodate different the led's & GIPO's for a particular implementation
        self.ORIENTATIONS = [
        [25, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0],
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 25]
        ]  
        self.current_orientation = 9999
        self.increment_orientation()
    
GPIOS = LED_Addresses()

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

    settime (hour, GPIOS.hours)
    settime (minute, GPIOS.minutes)
    settime (second, GPIOS.seconds)


def main():
    global MODE
    MODE = "main"
    if DEBUG: print ("DEBUG: main")
    init_gpios(GPIOS.hours)
    init_gpios(GPIOS.minutes)
    init_gpios(GPIOS.seconds)

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
    if DEBUG: print ("running tests")
    global MODE
    MODE = "test"
    init_gpios(GPIOS.hours)
    init_gpios(GPIOS.minutes)
    init_gpios(GPIOS.seconds)

    def cycle(gpios):
        delay = 0.1
        for g in range(len(gpios)):
            if g > 0:
                PINS[gpios[g-1]].off()
            PINS[gpios[g]].on()
            if g < len(gpios)-1:
                time.sleep(delay)
        for g in range(len(gpios)-1, -1, -1):
            if g < len(gpios)-1:
                PINS[gpios[g+1]].off()
            PINS[gpios[g]].on()
            if g > 0:
                time.sleep(delay)
        time.sleep(delay)
        PINS[gpios[0]].off()

    def all_set(status):
        for i in GPIOS.hours:
            PINS[i].value(status)
        for i in GPIOS.minutes:
            PINS[i].value(status)
        for i in GPIOS.seconds:
            PINS[i].value(status)

    if DEBUG: print ("TEST: All on")
    all_set(True)
    time.sleep(2)
    if MODE != "test": return
    if DEBUG: print ("TEST: All off")
    all_set(False)
    time.sleep(1)
    if MODE != "test": return
    if DEBUG: print ("TEST: All on")
    all_set(True)
    time.sleep(0.5)
    if DEBUG: print ("TEST: All off")
    all_set(False)
    
    if DEBUG: print ("TEST: Cycle seconds")
    for i in range(3):
        cycle(GPIOS.seconds)
        if MODE != "test": return
    if DEBUG: print ("TEST: Cycle minutes")
    for i in range(3):
        cycle(GPIOS.minutes)
        if MODE != "test": return
    if DEBUG: print ("TEST: Cycle hours")
    for i in range(3):
        cycle(GPIOS.hours)
        if MODE != "test": return
    if DEBUG: print ("TEST: complete")
    


############### BUTTONS ##########
DEBOUNCE_TIME = 200
button_debounce = time.ticks_ms()
BUTTON_HOURS = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
BUTTON_MINS = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
BUTTON_ORIENT = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)

def button_handler(pin):
    global button_debounce, MODE
    if time.ticks_diff(time.ticks_ms(), button_debounce) < DEBOUNCE_TIME:
        button_debounce = time.ticks_ms()
        return
    button_debounce = time.ticks_ms()
    if DEBUG: print("button_handler", pin, pin.value())
    if MODE == "test":
        MODE = "continue"
        return
    if pin == BUTTON_HOURS:
        if DEBUG: print ("  hours button")
        thetime = rtc.datetime()
        hour = int(thetime[4]) + 1
        # Increment hour
        rtc.datetime((thetime[0], thetime[1], thetime[2], thetime[3], hour, thetime[5], thetime[6], thetime[7]))
    if pin == BUTTON_MINS:
        if DEBUG: print ("  min's button")
        thetime = rtc.datetime()
        minute = int(thetime[5]) + 1
        # Increment minutes and set seconds to 0.
        rtc.datetime((thetime[0], thetime[1], thetime[2], thetime[3], thetime[4], minute, 0, 0))
    if pin == BUTTON_ORIENT:
        if DEBUG: print ("  orient button")
        GPIOS.increment_orientation()

def init_buttons():
    BUTTON_HOURS.irq(trigger=machine.Pin.IRQ_RISING, handler=button_handler)
    BUTTON_MINS.irq(trigger=machine.Pin.IRQ_RISING, handler=button_handler)
    BUTTON_ORIENT.irq(trigger=machine.Pin.IRQ_RISING, handler=button_handler)


###################################


if __name__ == "__main__":
    init_buttons()
    if DEBUG:
        print ("DEBUG on")
        test()
    main()

