from machine import Pin, Timer
import time


"""
Binary Clock for Raspberry Pi
"""

__author__ = "Todd Foster"
__version__ = "0.2.1"
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


# Orientation: each orientation is an array of 18 values from least significant bit to most significant (i.e., seconds-minutes-hours)
class LED_Addresses:
    def __init__(self):
        self.pins = [0] * 29
        self.ORIENTATIONS = [
        [25, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0],
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 25]
        ]
        self.current_orientation = 9999
        self.increment_orientation()
        self.init_gpios(self.ORIENTATIONS[0])
        
    def increment_orientation(self):
        self.current_orientation += 1
        if self.current_orientation >= len(self.ORIENTATIONS):
            self.current_orientation = 0
        self.seconds = self.ORIENTATIONS[self.current_orientation][0:6]
        self.minutes = self.ORIENTATIONS[self.current_orientation][6:12]
        self.hours   = self.ORIENTATIONS[self.current_orientation][12:18]
    
    def init_gpios(self, gpios):
        for g in gpios:
            self.pins[g] = Pin(g, Pin.OUT)
            self.pins[g].off()

    def set_bin_int (self, value, gpios):
        for i in range(len(gpios)):
            self.pins[gpios[i]].value(bool(value & (1<<i)))
            
    def set_gpio(self, gpio, value):
        self.pins[gpio].value(value)
        
    def set_all(self, value):
        for g in self.ORIENTATIONS[self.current_orientation]:
            self.set_gpio(g, value)
            
    def set_time(self, hour, minute, second):        
        self.set_bin_int (hour, self.hours)
        self.set_bin_int (minute, self.minutes)
        self.set_bin_int (second, self.seconds)

        
class StateClass:
    def __init__(self, mode):
        self.mode = mode

class ButtonHandler:
    def __init__(self, state, gpios, rtc):
        self.DEBOUNCE_TIME = 200
        self.button_debounce = time.ticks_ms()
        
        self.state = state
        self.gpios = gpios
        self.rtc = rtc
        
        self.BUTTON_HOURS  = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
        self.BUTTON_MINS   = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
        self.BUTTON_ORIENT = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
        
        self.BUTTON_HOURS.irq( trigger=machine.Pin.IRQ_RISING, handler=self.button_handler)
        self.BUTTON_MINS.irq(  trigger=machine.Pin.IRQ_RISING, handler=self.button_handler)
        self.BUTTON_ORIENT.irq(trigger=machine.Pin.IRQ_RISING, handler=self.button_handler)

    def button_handler(self, pin):
        if time.ticks_diff(time.ticks_ms(), self.button_debounce) < self.DEBOUNCE_TIME:
            self.button_debounce = time.ticks_ms()
            return
        self.button_debounce = time.ticks_ms()
        if DEBUG: print("button_handler", pin, pin.value())
        if self.state.mode == "test":
            self.state.mode = "continue"
            return
        if pin == self.BUTTON_ORIENT:
            if DEBUG: print ("  orient button")
            self.gpios.increment_orientation()
            return
        
        thetime = self.rtc.datetime()
        if pin == self.BUTTON_HOURS:
            if DEBUG: print ("  hours button")
            hour = (int(thetime[4]) + 1) % 24
            self.rtc.datetime((thetime[0], thetime[1], thetime[2], thetime[3], hour, thetime[5], thetime[6], thetime[7]))
        if pin == self.BUTTON_MINS:
            if DEBUG: print ("  min's button")
            minute = (int(thetime[5]) + 1) % 60
            # Increment minutes and set seconds to 0.
            self.rtc.datetime((thetime[0], thetime[1], thetime[2], thetime[3], thetime[4], minute, 0, 0))
        

def test(state, gpios):
    if DEBUG: print ("running tests")
    state.mode = "test"

    def cycle(gpios, segment):
        delay = 0.1
        segment_count = len(segment)
        ## TODO use enumerate(segment) instead of in range(count)
        for s in range(segment_count):
            if s > 0:
                gpios.set_gpio(segment[s-1], False)
            gpios.set_gpio(segment[s], True)
            if s < segment_count-1:
                time.sleep(delay)
        for s in range(segment_count-1, -1, -1):
            if s < segment_count-1:
                gpios.set_gpio(segment[s+1], False)
            gpios.set_gpio(segment[s], True)
            if s > 0:
                time.sleep(delay)
        time.sleep(delay)
        gpios.set_gpio(segment[0], False)
        
    if DEBUG: print ("TEST: All on")
    gpios.set_all(True)
    time.sleep(2)
    if state.mode != "test": return
    if DEBUG: print ("TEST: All off")
    gpios.set_all(False)
    time.sleep(1)
    if state.mode != "test": return
    gpios.set_all(True)
    time.sleep(0.5)
    if DEBUG: print ("TEST: All off")
    gpios.set_all(False)
    
    if DEBUG: print ("TEST: Cycle seconds")
    for i in range(3):
        cycle(gpios, gpios.seconds)
        if state.mode != "test": return
    if DEBUG: print ("TEST: Cycle minutes")
    for i in range(3):
        cycle(gpios, gpios.minutes)
        if state.mode != "test": return
    if DEBUG: print ("TEST: Cycle hours")
    for i in range(3):
        cycle(gpios, gpios.hours)
        if state.mode != "test": return
    if DEBUG: print ("TEST: complete")

        
def main():
    state = StateClass("init")
    gpios = LED_Addresses()
    rtc = machine.RTC()
    buttons = ButtonHandler(state, gpios, rtc)

    if DEBUG: print ("DEBUG on")
    test(state, gpios)
        
    state.mode = "main"
    if DEBUG: print ("DEBUG: main")
    
    # TODO: use timer instead of while True:
    #timer = Timer()
    #def blink(timer):
    #    led.toggle()
    #timer.init(freq=1.0, mode=Timer.PERIODIC, callback=blink)

    while True:
        thetime = rtc.datetime()
        hour = int(thetime[4])
        minute = int(thetime[5])
        second = int(thetime[6])
        if DEBUG > 0: print(f"DEBUG Time  = {thetime} = {hour}:{minute}:{second} = {bin(hour)}:{bin(minute)}:{bin(second)}")

        gpios.set_time(hour, minute, second)

        while (thetime == rtc.datetime()):
            time.sleep(0.1)


if __name__ == "__main__":
    main()

