import BlynkLib
from datetime import datetime
# Initialize Blynk
blynk = BlynkLib.Blynk('36s6LhrcXqms9Jb30u532cbdGhB-GurZ')

# Register Virtual Pin
@blynk.VIRTUAL_READ(2)
def my_read_handler():
     currentTime = datetime.now()
     blynk.virtual_write(2, currentTime.strftime("%d/%m/%Y %H:%M:%S"))

print("started!")
while True:
    blynk.run()