import time
import board
import neopixel

"""
Binary Clock for Adafruit QT Py with Neopixel display
"""

__author__ = "Todd Foster"
__version__ = "0.1.0"
__license__ = """
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

on_board_pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel_pin = board.A3
num_pixels = 16
pixels = neopixel.NeoPixel(pixel_pin, num_pixels, auto_write=False)
pixels.brightness = 0.08

RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
OFF = (0,0,0)

SECONDS = YELLOW
MINUTES = GREEN
HOURS = RED

SECONDS_OFF = (0,0,0)
MINUTES_OFF = (0,0,0)
HOURS_OFF = (0,0,0)
INITIAL_OFF = (0,0,15)

DEBUG = 1
        
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
        

def test(state, on_board_pixel, pixels):
    if DEBUG: print ("running tests")
    state.mode = "test"

    # Test on-board pixel
    for x in range(255):
        on_board_pixel.fill((x,0,0))
    for x in range(255):
        on_board_pixel.fill((255-x,0,0))
        
    for x in range(255):
        on_board_pixel.fill((0,x,0))
    for x in range(255):
        on_board_pixel.fill((0,255-x,0))

    for x in range(255):
        on_board_pixel.fill((0,0,x))
    for x in range(255):
        on_board_pixel.fill((0,0,255-x))



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
    pixels.fill(RED)
    pixels.show()
    time.sleep(1)
    if DEBUG: print ("TEST: All off")
    pixels.fill(OFF)
    pixels.show()
    if state.mode != "test": return
    time.sleep(1)

    if DEBUG: print ("TEST: All on")
    pixels.fill(GREEN)
    pixels.show()
    time.sleep(1)
    if DEBUG: print ("TEST: All off")
    pixels.fill(OFF)
    pixels.show()
    if state.mode != "test": return
    time.sleep(1)

    if DEBUG: print ("TEST: All on")
    pixels.fill(YELLOW)
    pixels.show()
    time.sleep(1)
    if DEBUG: print ("TEST: All off")
    pixels.fill(OFF)
    pixels.show()
    if state.mode != "test": return
    time.sleep(1)

    if DEBUG: print ("TEST: Cycle seconds")
    for r in (0, 1, 2, 4, 8, 16, 32, 32, 16, 8, 4, 2, 1, 0):
        show_time((0,0,r), on_board_pixel, pixels)
        time.sleep(0.25)
    if state.mode != "test": return

    if DEBUG: print ("TEST: Cycle minutes")
    for r in (0, 1, 2, 4, 8, 16, 32, 32, 16, 8, 4, 2, 1, 0):
        show_time((0,r,0), on_board_pixel, pixels)
        time.sleep(0.25)
    if state.mode != "test": return

    if DEBUG: print ("TEST: Cycle hours")
    for r in (0, 1, 2, 4, 8, 16, 32, 32, 16, 8, 4, 2, 1, 0):
        show_time((r,0,0), on_board_pixel, pixels)
        time.sleep(0.25)
    if state.mode != "test": return


def show_time(thetime, on_board_pixel, pixels):
    hour = int(thetime[0])
    minute = int(thetime[1])
    second = int(thetime[2])
    if DEBUG > 0: print(f"DEBUG show_time Time  = {thetime} = {hour}:{minute}:{second} = {bin(hour)}:{bin(minute)}:{bin(second)}")

    INITIAL_PIXELS = [0, 5, 11]

    if second % 2 > 0:
        on_board_pixel.fill(SECONDS)
    else:
        on_board_pixel.fill(SECONDS_OFF)
        
    for i in range(5):
        if (bool(second & (1<<i+1))):
            pixels[i]=SECONDS
        else:
            if i in INITIAL_PIXELS:
                pixels[i]=INITIAL_OFF
            else:
                pixels[i]=SECONDS_OFF
            
    for i in range(5,11):
        if (bool(minute & (1<<i-5))):
            pixels[i]=MINUTES
        else:
            if i in INITIAL_PIXELS:
                pixels[i]=INITIAL_OFF
            else:
                pixels[i]=MINUTES_OFF

    for i in range(11,16):
        if (bool(hour & (1<<i-11))):
            pixels[i]=HOURS
        else:
            if i in INITIAL_PIXELS:
                pixels[i]=INITIAL_OFF
            else:
                pixels[i]=HOURS_OFF
            
    on_board_pixel.show()
    pixels.show()

        
def main():
    state = StateClass("init")
    #gpios = LED_Addresses()
    #buttons = ButtonHandler(state, gpios, rtc)

    if DEBUG: print ("DEBUG on")
    test(state, on_board_pixel, pixels)
        
    state.mode = "main"
    if DEBUG: print ("DEBUG: main")
    
    # TODO: use timer instead of while True:
    #timer = Timer()
    #def blink(timer):
    #    led.toggle()
    #timer.init(freq=1.0, mode=Timer.PERIODIC, callback=blink)

    while True:
        thetime = time.localtime()
        hour = int(thetime[3])
        minute = int(thetime[4])
        second = int(thetime[5])
        if DEBUG > 0: print(f"DEBUG Time  = {hour}:{minute}:{second} = {bin(hour)}:{bin(minute)}:{bin(second)}")

        show_time((hour, minute, second), on_board_pixel, pixels)

        while (thetime == time.localtime()):
            time.sleep(0.1)


if __name__ == "__main__":
    main()
