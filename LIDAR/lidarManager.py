
import qwiic_vl53l1x
import time
import sys
import Jetson.GPIO as GPIO

import os

#Cable connection : 29,21,15,M19M,13,11,7,23   M marks main branch
class lidarManager:

    def __init__(self,XSHUT = [15,21,29,19,23,11,13,7]):  #15,21,29,19,23,11,13,7
        self.XSHUT = XSHUT # 13,15,21,23     pins of conected sensors, it also defines the number of sensors to be used
    
        self.first_address = 0x30 #first I2C address to asign
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD) #numbering of ports
        self.init_outputs()
        #instantiate sensor objects
        self.ToF = []
        for sensor in range(0,len(self.XSHUT)):
            self.ToF.append(qwiic_vl53l1x.QwiicVL53L1X())


    def init_outputs(self):
        for pin in self.XSHUT:
            GPIO.setup(pin, GPIO.OUT) #put as output
            GPIO.output(pin, False) #turn off

    def set_input(self):
        for pin in self.XSHUT:
            GPIO.setup(pin, GPIO.IN) #put as output
            

    def start_sensors(self):
        #turn on  and  set addresses   
        for sensor in range(0,len(self.XSHUT)):
            GPIO.output(self.XSHUT[sensor], True) #turn on
            # print("sensor on")
            
            self.ToF[sensor].sensor_init()	#initialize
            time.sleep(.005)
            self.ToF[sensor].set_distance_mode(2)
            self.ToF[sensor].set_i2c_address( self.first_address + sensor*2) #change address


    def stop_sensors(self):
            #turn off   
            for sensor in range(0,len(self.XSHUT)-1):
                GPIO.output(self.XSHUT[sensor], False) #turn off

    def start_ranging(self):
        #start ranging
        for sensor in range(0,len(self.XSHUT)):
                self.ToF[sensor].start_ranging()

    def stop_ranging(self):
        #start ranging
        for sensor in range(0,len(self.XSHUT)):
                self.ToF[sensor].stop_ranging()
    
    def get_range(self,sensor):
        return self.ToF[sensor].get_distance()


def main():
        while True:
            Manager = lidarManager()
            Manager.init_outputs()
            Manager.start_sensors()
            Manager.set_input()
            Manager.start_ranging()
            print("while")
            

            while True:
                try:
                    string = ""        
                    start_time = time.time()

                    for sensor in range(0,len(Manager.XSHUT)):
                        value = Manager.get_range(sensor)
                        string = string + str(value)+","
                    time.sleep(.05)
                    print("Distance(mm): %s" % string)
                    print("--- %f Hz ---" % (1/(time.time() - start_time)))
                
            
                except:
                    GPIO.cleanup()
                    break
                

            print("Error")
            break
        # GPIO.setmode(GPIO.BOARD) 
        # GPIO.setup(7, GPIO.OUT)
        # GPIO.output(7, False)
        
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)