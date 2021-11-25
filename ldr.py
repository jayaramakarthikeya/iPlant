from gpiozero import LightSensor
ldr= LightSensor(4)
import time 
while True:
    time.sleep(5)
    print(ldr.value)
    
    
    