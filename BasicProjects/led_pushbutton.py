import machine
import time

push = Pin(27, Pin.IN)
led = Pin(26, Pin.OUT)

while True:
    if push.value():
        led.value(1)
    else:
        led.value(0)
    time.sleep(1)