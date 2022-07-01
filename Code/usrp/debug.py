#load:[DEBUG]
import utime
import uasyncio
from machine import ADC
class mainloader:
    
    def getSpeed():
        return 0
    
    async def start(LCD):
        LCD.clear()
        LCD.putstr('[DEBUG]')
        analogInputPin = ADC(26)
        
        MILLIVOLT_PER_AMPERE = 185
        AREF = 3.3
        DEFAULT_OUTPUT_VOLTAGE = 3.3/2
        
        while True:
            LCD.move_to(0,1)
            isum = ADC.read_u16(analogInputPin)
            LCD.putstr(str(isum) +' ')
            
            sensor_voltage = (isum / 65535) * AREF
            sensor_voltage = (sensor_voltage - DEFAULT_OUTPUT_VOLTAGE ) * 1000
            dc_current = (sensor_voltage / MILLIVOLT_PER_AMPERE)
            rsum = (round(abs(dc_current/2)*15.4,2))
            
            LCD.move_to(0,2)
            LCD.putstr(str(rsum) +' ')
            
            xsum = (round(abs(dc_current/2)*15.4,1))
            
            LCD.move_to(0,3)
            LCD.putstr(str(xsum) +' ')
            await uasyncio.sleep(0.25)
