import qwiic_vl53l1x
import time
import sys
import RPi.GPIO as GPIO

XSHUT1 = 31
XSHUT2 = 33

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(XSHUT1, GPIO.OUT)
GPIO.setup(XSHUT2, GPIO.OUT)
GPIO.output(XSHUT1, False)
GPIO.output(XSHUT2, False)




GPIO.output(XSHUT2, True)



print("\nSparkFun VL53L1X Example 1\n")
ToF1 = qwiic_vl53l1x.QwiicVL53L1X()
ToF2 = qwiic_vl53l1x.QwiicVL53L1X()
   

ToF1.sensor_init()
ToF1.set_i2c_address(0x22)
GPIO.output(XSHUT1, True)
ToF2.sensor_init()
ToF2.set_i2c_address(0x23)


while True:
    try:
        start_time = time.time()
        ToF1.start_ranging()                         # Write configuration bytes to initiate measurement
        ToF2.start_ranging()                         # Write configuration bytes to initiate measurement
        
        time.sleep(.005)
        distance1 = ToF1.get_distance()   # Get the result of the measurement from the sensor
        distance2 = ToF2.get_distance()   # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF1.stop_ranging()
        ToF2.stop_ranging()

        print("Distance(mm): %s" % distance1,distance2)
        print("--- %f Hz ---" % (1/(time.time() - start_time)))

    except Exception as e:
        print(e)

GPIO.output(XSHUT1, False)
GPIO.output(XSHUT2, False)