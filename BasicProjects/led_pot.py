from machine import Pin, ADC, PWM
import time

pot = ADC(Pin(33))

led = PWM(Pin(14), freq=1000)

while True:
    val = pot.read()          
    duty = int(val / 4)       
    led.duty(duty)
    time.sleep(0.5)