from machine import I2C, Timer
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd
import utime
import gc

class DisplayManager:

    I2C_ADDR     = 0x27
    I2C_NUM_ROWS = 4
    I2C_NUM_COLS = 20

    def __init__(self, sda, scl, freq=400000):
        try:
            self.i2c = I2C(1, sda=sda, scl=scl, freq=freq)
            self.LCD = I2cLcd(self.i2c, DisplayManager.I2C_ADDR, DisplayManager.I2C_NUM_ROWS, DisplayManager.I2C_NUM_COLS)
            self.LCD.putstr("Initialization:")
            self.LCD.move_to(0,1)
            self.LCD.putstr("Display is done")
            self.buffer = []
            self.timer = Timer()
            self.init = True
            #self.lock = False
        except:
            self.init = False
            self.buffer = []
            print("______WARNING______")
            print("DISPALY NOT INITED!")

    def addInBuffer(self,b,v):
        if self.init == True:
            self.buffer.append([b,v])

    def callRendering(self):
        if not self.buffer == [] or self.init == True:
            #if not self.lock:
            self.lock = True
            sbuffer = self.buffer
            self.buffer = []
            for buffer in sbuffer:
                try:
                    if isinstance(buffer[1],str):
                        buffer[0](buffer[1])
                    else:
                         buffer[0](*buffer[1])
                    #sbuffer.remove(buffer)
                    #print(buffer)
                    utime.sleep(0.001)
                except:
                    print('0_0')
                    print('X_<')
                    print('>_X')
                    print('%_%')
                    print('%_%')
                    print(buffer)
                    print('reintdisplay')
                    #self.LCD = I2cLcd(self.i2c, DisplayManager.I2C_ADDR, DisplayManager.I2C_NUM_ROWS, DisplayManager.I2C_NUM_COLS)
                    print('AND&?')
                    break
            #self.lock = False
        #gc.collect()

    def getLCD(self):
        if self.init == False:
            return I2cLcd(self.i2c, DisplayManager.I2C_ADDR, DisplayManager.I2C_NUM_ROWS, DisplayManager.I2C_NUM_COLS , True)
        return self.LCD

    def autoUpdate(self,enable):
        if self.init == True:
            if(enable):
                self.timer.init(mode=Timer.PERIODIC, period=200, callback=lambda t:DisplayManager.callRendering(self))
            else:
                self.timer.deinit()
