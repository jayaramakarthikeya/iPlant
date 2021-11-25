from gpiozero import Robot 
from time import sleep 
import cv2 
import apriltag_detect 
import ultrasonic 
import serial
import RPi.GPIO as GPIO
import time
from num2words import num2words
from subprocess import call

cmd_beg= 'espeak -s160 -ven+f2 '

def measure(i,TRIG,ECHO):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance+1.15, 2)
  
    if distance<=20 and distance>=5:
        print("distance:",distance,"cm")
        i=1
          
    if distance>20 and i==1:
        print("place the object....")
        i=0
    time.sleep(2)
    return i , distance

def active():
    GPIO.setmode(GPIO.BCM)

    TRIG = 22
    ECHO = 18
    i = 0
    distance = 0

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)

    GPIO.output(TRIG, False)
    print("Calibrating.....")
    time.sleep(2)

    print("Place the object......")
    
    try:
        while True:
           i , distance = measure(i,TRIG,ECHO)
           return distance

    except KeyboardInterrupt:
        GPIO.cleanup()


def track_water(robot):
    print("[INFO] loading video...")
    capture = cv2.VideoCapture(-1)
    print("[INFO] detecting AprilTags...")
    text = 'I am thirsty Let me get some water'
    text = text.replace(' ', '_')
    call([cmd_beg+text], shell=True)
    
    len_prev_res = 0
    #distance = 0
    while True:
     #distance = active()
     isTrue , frame = capture.read()
     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     results = apriltag_detect.detector(gray,len_prev_res)
     len_prev_res = len(results)
     
     if len(results) == 1:
        robot.left()
        sleep(0.2)
     #elif distance < 15 and len(results) == 1:
        #robot.stop()
        #sleep(10)
        #robot.right()
        #sleep(2)
     elif len(results) == 0:
        robot.backward()
        sleep(0.02)
        robot.stop()
        sleep(0.1)
     #elif distance < 15 and len(results) == 0:
        #robot.stop()
        #sleep(1)
        
     image = apriltag_detect.detection_res(frame,results)
     #cv2.imshow('Video',image)

     if cv2.waitKey(20) & 0xFF==ord('d'):
      break

    capture.release()
    cv2.destroyAllWindows()
    
def sun_track(robot,ser):
    prev_meter = 0
    light_meter = 0
    
    while True:
     read_serial=ser.readline()
     light_meter = int(read_serial)
     print(light_meter)
     
     if light_meter < 700:
      text = 'I am feeling nice here'
      text = text.replace(' ', '_')
      call([cmd_beg+text], shell=True)   
      robot.stop()
      
     elif light_meter > 700:
      text = 'It is too sunny here'
      text = text.replace(' ', '_')
      call([cmd_beg+text], shell=True) 
      robot.right()
      sleep(0.08)
     elif light_meter > 700 and light_meter > prev_meter:
      robot.left()
      sleep(0.1)
     elif light_meter > 700 and light_meter < prev_meter:
      robot.right()
      sleep(0.08)
     
     prev_meter = light_meter

def main():
    
    robot = Robot(left = (27, 17), right = (22, 23))
    
    ser = serial.Serial('/dev/ttyACM0',9600)

    
    mode = 0
    mode = input("Enter the mode: ")
    
    if int(mode)==23:
        track_water(robot)
    else:
        sun_track(robot,ser)
        

if __name__ == '__main__':
    main()
