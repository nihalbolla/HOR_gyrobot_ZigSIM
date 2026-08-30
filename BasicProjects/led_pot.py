from machine import Pin, ADC, PWM
from time import sleep

led = PWM(Pin(26), freq=1000)   

pot = ADC(Pin(34))
pot.atten(ADC.ATTN_11DB)        
pot.width(ADC.WIDTH_12BIT)     

while True:
    value = pot.read()          
    

    duty = int((value / 4095) * 1023)
    led.duty(duty)
    
    print("ADC:", value, "Duty:", duty)
    sleep(0.05)