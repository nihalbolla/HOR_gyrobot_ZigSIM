from machine import Pin, time_pulse_us
import time

trig = Pin(19, Pin.OUT)
echo = Pin(21, Pin.IN)
led = Pin(13, Pin.OUT)

def get_distance():
    trig.value(0)
    time.sleep_us(2)
    
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 30000)
    distance = (duration / 2) / 29.1
    return distance

while True:
    dist = get_distance()
    print("Distance:", dist, "cm")

    if dist < 10:
        led.value(1)
    else:
        led.value(0)

    time.sleep(0.2)