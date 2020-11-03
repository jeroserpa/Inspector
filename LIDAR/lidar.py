
import VL53L1X
import RPi.GPIO as GPIO
import time
import sys, os


def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__



XSHUT1 = 31
XSHUT2 = 33

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(XSHUT1, GPIO.OUT)
GPIO.setup(XSHUT2, GPIO.OUT)
GPIO.output(XSHUT1, False)
GPIO.output(XSHUT2, False)

flag = True


GPIO.output(XSHUT1, True)
tof1 = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof1.open()
try:
    while flag:
        start_time = time.time()
        GPIO.output(XSHUT1, True)
       
        tof1.start_ranging(3)                   # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
        distance1_in_mm = tof1.get_distance()    # Grab the range in m
        tof1.stop_ranging()

        GPIO.output(XSHUT1, False)
        GPIO.output(XSHUT2, True)
        
        tof1.start_ranging(3)                   # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
        distance2_in_mm = tof1.get_distance()    # Grab the range in mm
        tof1.stop_ranging()                     # Stop ranging

    
        GPIO.output(XSHUT1, True)
       
        tof1.start_ranging(3)                   # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
        distance3_in_mm = tof1.get_distance()    # Grab the range in m
        tof1.stop_ranging()

        GPIO.output(XSHUT1, False)
        GPIO.output(XSHUT2, True)
        
        tof1.start_ranging(3)                   # Start ranging, 1 = Short Range, 2 = Medium Range, 3 = Long Range
        distance4_in_mm = tof1.get_distance()    # Grab the range in mm
        tof1.stop_ranging()                     # Stop ranging











        print('Sensors 1,2:',distance1_in_mm,distance2_in_mm,distance3_in_mm,distance4_in_mm)

        GPIO.output(XSHUT1, False)
        GPIO.output(XSHUT2, False)
        print("--- %f Hz ---" % (1/(time.time() - start_time)))

except KeyboardInterrupt:
    flag = False
    pass

