#load:test program
import utime
import uasyncio
from machine import Pin, PWM
PWM = PWM(Pin(16))
PWM.freq(2000)

class mainloader:

    SPEED = 0

    async def start(LCD):
        print(LCD)
        LCD.clear()
        LCD.putstr('RUNNING!!! ')
        PowerLEDRelay= Pin(6,Pin.OUT)
        RelayPower = Pin(12,Pin.OUT)
        RelayPower.high()
        PowerLEDRelay.high()
        while True:
            LCD.move_to(0,2)
            LCD.putstr('Run Engine ')

            DUTY = 65025
            while DUTY > 0:
                DUTY -= 3251.25
                PWM.duty_u16(round(DUTY))
                mainloader.SPEED += 5
                await uasyncio.sleep(0.25)
                #utime.sleep(0.25)
            await uasyncio.sleep(5)
            #utime.sleep(5)
            LCD.move_to(0,2)
            LCD.putstr('Stop Engine ')

            while DUTY < 65025:
                DUTY += 3251.25
                PWM.duty_u16(round(DUTY))
                mainloader.SPEED -= 5
                await uasyncio.sleep(0.25)
                #utime.sleep(0.25)
            await uasyncio.sleep(5)
            #utime.sleep(5)

    def getSpeed():
        return mainloader.SPEED
