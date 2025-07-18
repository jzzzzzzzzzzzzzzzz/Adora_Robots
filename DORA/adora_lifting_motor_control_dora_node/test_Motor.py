import serial
import time
import time
from SerialLiftingMotor import SerialLiftingMotor
 
 
if __name__ == "__main__":
    app = SerialLiftingMotor()
     
    while True:
        app.motor_position_set(32768*200)
        time.sleep(20)   
        app.motor_position_set(0)
        time.sleep(20)  
    	
    	
    	
    	
    	
    	
    	
    	
    	
