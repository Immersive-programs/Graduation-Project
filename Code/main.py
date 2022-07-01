#
# Made Denis and Nikita
#
import utime
import uasyncio
import _thread
import os
import gc
from machine import Pin, PWM, ADC, Timer
from displaymanager import DisplayManager
from rotary import Rotary

# Initialize Globals
global Mode
global SPEED
global Rtimer
global Ltimer
global Engine
global locklaunchprogramm
global DM
global REV
global SpeedCount
global ListCurrent
global lockstart
global lockstop
global lockspeedcount

# Initialize DisplayManager
DM = DisplayManager(sda=Pin(10),scl=Pin(11))

# Initialize Timers
Rtimer = Timer()
Ltimer = Timer()

# Initialize VARIABLES
I2C_ADDR     = 0x27
I2C_NUM_ROWS = 4
I2C_NUM_COLS = 20
MAX_DUTY = 65025
MIN_DUTY_SPEED = 65025
TIME_DUTY_EDIT = 0.25
DUTY = 65025
SPEED = 50
Mode = 0
AutoCountProgram = 0
AutoSelectProgram = 0
SpeedCount = 0
REV = False
Engine = False
RevButtonLock = False
locklaunchprogramm = False
lockstart = False
lockstop = True
lockspeedcount = False
AutoNameProgram =''
AutoNamesProgram = []
ListCurrent = []

# Initialize CURRENT SENSORS
MILLIVOLT_PER_AMPERE = 185
AREF = 3.3
DEFAULT_OUTPUT_VOLTAGE = 3.3/2
analogInputPin = ADC(26)

# Initialize ENCODER
Rotary = Rotary(1,0)

# Initialize FREQUENCY PIN
PWM = PWM(Pin(16))
PWM.freq(30)
PWM.duty_u16(65025)

# Initialize LED
ManualLED = Pin(8,Pin.OUT)
AutoLED = Pin(9,Pin.OUT)

# Initialize Relay
RevLEDRelay= Pin(7,Pin.OUT)
PowerLEDRelay= Pin(6,Pin.OUT)
RelayPower = Pin(12,Pin.OUT)
RevPower = Pin(13,Pin.OUT)

# Initialize BUTTONS
StartButton = Pin(2,Pin.IN,Pin.PULL_DOWN)
RevButton = Pin(3,Pin.IN,Pin.PULL_DOWN)
pinstopbutton = 4
StopButton = Pin(pinstopbutton,Pin.IN,Pin.PULL_DOWN)

# Initialize SWITCHES
pinmodeswitch = 5
ModeSwitch = Pin(pinmodeswitch,Pin.IN,Pin.PULL_DOWN)

#buzzer
buzzer = Pin(19,Pin.OUT)

SpeedPin = Pin(28,Pin.IN,Pin.PULL_DOWN)

async def main():
    global Mode
    Mode = ModeSwitch.value()
    LCD = DM.getLCD()

    try:
        generateLoader()
        DM.addInBuffer(LCD.move_to,(0,2))
        DM.addInBuffer(LCD.putstr,('Loader is done'))
    except:
        print("_____WARNING_____")
        print("LOADER NOT INITED!")
        DM.addInBuffer(LCD.move_to,(0,2))
        DM.addInBuffer(LCD.putstr,('LOADER NOT INITED'))
    DM.callRendering()
    utime.sleep(0.2)

#     _thread.start_new_thread(_Sensor,())
    SpeedPin.irq(handler=regRPS, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    DM.addInBuffer(LCD.move_to,(0,3))
    DM.addInBuffer(LCD.putstr,('Sensors is done'))
    DM.callRendering()
    
    buzzer.high()
    utime.sleep(0.1)
    buzzer.low()
    utime.sleep(0.4)
 
    DM.autoUpdate(True)
    if Mode:
        setAutoMode()
    else:
        setManualMode()
        
    sensortimer = Timer()
    sensortimer.init(mode=Timer.PERIODIC, period=1000, callback=lambda t:putSpeed())

def regRPS(change):
    global lockspeedcount
    global SpeedCount
    if SpeedPin.value() == 1:
        if not lockspeedcount:
            lockspeedcount = True
            SpeedCount += 1
    else:
        lockspeedcount = False
        
def putSpeed():
    global Mode
    global Engine
    global ListCurrent
    global SpeedCount
    global r_thread
    if not Mode:
        LCD = DM.getLCD()
        #isum = 0
        #if len(ListCurrent) > 0:
        isum = ADC.read_u16(analogInputPin)#######sum(ListCurrent) / len(ListCurrent)
        ListCurrent.clear()
        
        sensor_voltage = (isum / 65535) * AREF
        sensor_voltage = (sensor_voltage - DEFAULT_OUTPUT_VOLTAGE ) * 1000
        dc_current = (sensor_voltage / MILLIVOLT_PER_AMPERE)
        if Engine:
            rsum = (round(abs(dc_current/2)*15.4,2))
        else:
            rsum = 0.0
#             rsum = (round(abs(dc_current/2)*15.4,1))
#             if rsum <= 0.4:
            
        #DM.addInBuffer(LCD.putstrxy,(' '+str(rsum), 18-len(str(rsum)), 3))
        
        #DM.addInBuffer(LCD.putstrxy,((str(SpeedCount)+'|RPS '), 7, 3))
        speed = str(SpeedCount)+'/RPS '
        current = str(rsum)
        space = (12-len(speed)-len(current))
        if len(current) < 5:
            DM.addInBuffer(LCD.putstrxy,(speed + (' ' * space) + current, 7, 3))
        else:
            DM.addInBuffer(LCD.putstrxy,(speed, 7, 3))
        SpeedCount = 0
        
    gc.collect()

# def _Sensor():
#     global ListCurrent
#     while True:
#         ListCurrent.append(ADC.read_u16(analogInputPin))  
#         utime.sleep_ms(250)
        
def setManualMode():
    global lockstart
    global lockstop
    lockstart = False
    lockstop = True
    
    LCD = DM.getLCD()

    PWM.duty_u16(MAX_DUTY)

    RelayPower.low()
    PowerLEDRelay.low()

    ManualLED.high()
    AutoLED.low()

    DM.addInBuffer(LCD.clear,())
    Rotary.clear_handler()

    setLable(0)

    DM.addInBuffer(LCD.move_to,(0,1))
    DM.addInBuffer(LCD.putstr,('Speed:'))

    setLineSpeedPos(1,SPEED/100)

    DM.addInBuffer(LCD.move_to,(0,3))
    DM.addInBuffer(LCD.putstr,('RSpeed:0/RPS    0.0A'))

    DM.addInBuffer(LCD.move_to,(13,1))
    if REV:
        RevLEDRelay.high()
        DM.addInBuffer(LCD.putstr,('REVERSE'))
    else:
        RevLEDRelay.low()
        DM.addInBuffer(LCD.putstr,('       '))

    Rotary.add_handler(rotary_changed)
    StartButton.irq(handler=startEngine, trigger=Pin.IRQ_FALLING)
    RevButton.irq(handler=revEngine, trigger=Pin.IRQ_FALLING)
    ModeSwitch.irq(handler=switchMode, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    StopButton.irq(handler=stopEngine, trigger=Pin.IRQ_FALLING)

def setAutoMode():
    LCD = DM.getLCD()
    PWM.duty_u16(MAX_DUTY)
    RevLEDRelay.low()
    RelayPower.low()
    ManualLED.low()
    AutoLED.high()
    DM.addInBuffer(LCD.clear,())
    setLable(4)
    StopButton.irq(handler=None)
    Rotary.add_handler(handler=None)
    StartButton.irq(handler=None)
    RevButton.irq(handler=None)
    Rotary.clear_handler()

    global AutoCountProgram
    global AutoSelectProgram
    global AutoNameProgram
    global AutoNamesProgram
    AutoNamesProgram = []
    odir = os.listdir('usrp')
    length = len(odir)
    AutoCountProgram = length
    for u in range(length):
        o = open('usrp/'+odir[u])
        line = o.readline().split(':')
        if line[0] == "#load":
            AutoNamesProgram.append(line[1])
            DM.addInBuffer(LCD.move_to,(0,u+1))
            DM.addInBuffer(LCD.putstr,(str(u)+':'+line[1]))
            DM.addInBuffer(LCD.move_to,(18,u+1))
            if u == AutoSelectProgram:
                AutoNameProgram = line[1]
                DM.addInBuffer(LCD.putstr,('|◿'))
            else:
                DM.addInBuffer(LCD.putstr,('|◛'))
        o.close()

    global locklaunchprogramm
    locklaunchprogramm = False

    Rotary.add_handler(auto_rotary_changed)
    StartButton.irq(handler=startingProgram, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
    ModeSwitch.irq(handler=switchMode, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

def startingProgram(change):
    LCD = DM.getLCD()
    if StartButton.value():
        Rotary.clear_handler()

        global locklaunchprogramm
        if not locklaunchprogramm:
            DM.addInBuffer(LCD.clear,())
            DM.addInBuffer(LCD.move_to,(0,0))
            DM.addInBuffer(LCD.putstr,('Launching program:'))
            DM.addInBuffer(LCD.move_to,(0,1))
            DM.addInBuffer(LCD.putstr,(AutoNameProgram))
            locklaunchprogramm = True

        DM.addInBuffer(LCD.move_to,(0,2))
        DM.addInBuffer(LCD.putstr,('Release START button'))
        DM.callRendering()
    else:
        StopButton.irq(handler=bswitchMode, trigger=Pin.IRQ_FALLING)
        DM.addInBuffer(LCD.move_to,(0,2))
        DM.addInBuffer(LCD.putstr,(' Hold START button: '))
        DM.addInBuffer(LCD.move_to,(0,3))
        DM.addInBuffer(LCD.putstr,('      3 second      '))
        StartButton.irq(handler=programStarted, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)

def programStarted(change):
    LCD = DM.getLCD()
    StopButton.irq(handler=None)
    i = 3
    complete = False
    while StartButton.value() and complete == False:
        DM.addInBuffer(LCD.move_to,(0,3))
        i -=1
        DM.addInBuffer(LCD.putstr,('      '+str(i)+' second      '))
        DM.callRendering()
        utime.sleep(1)
        if i == 0:
            complete = True
    if complete:
        StartButton.irq(handler=None)
        DM.addInBuffer(LCD.move_to,(0,2))
        DM.addInBuffer(LCD.putstr,('      STARTING      '))
        DM.addInBuffer(LCD.move_to,(0,3))
        DM.addInBuffer(LCD.putstr,('                    '))
        DM.callRendering()

        buzzer.high()
        utime.sleep(0.05)
        buzzer.low()
        utime.sleep(0.1)
        buzzer.high()
        utime.sleep(0.05)
        buzzer.low()

        uasyncio.run(runProgram())
    else:
        DM.addInBuffer(LCD.move_to,(0,3))
        DM.addInBuffer(LCD.putstr,('      3 second      '))
        DM.callRendering()
        StopButton.irq(handler=bswitchMode, trigger=Pin.IRQ_FALLING)

async def runProgram():
    print('run')
    from TempData import Loader
    global pinstopbutton
    global pinmodeswitch
    global AutoSelectProgram
    Loader.setParameters(pinstopbutton,pinmodeswitch)
    file = os.listdir('usrp')[AutoSelectProgram].split('.')[0]
    uasyncio.run(Loader.start(file,DM.getLCD()))
    speed = Loader.getSpeed(file)
    global Engine
    global DUTY
    if speed != 0:
        Engine = True
        DUTY = MAX_DUTY-((MAX_DUTY*speed)/100)
    print(speed)
    print(DUTY)
    modeSwitched()

def modeSwitched():
    global Mode
    global Engine
    if Engine:
        CriticalEngineStop()
    else:
        Mode = ModeSwitch.value()
        if ModeSwitch.value():
            setAutoMode()
        else:
            setManualMode()

def bswitchMode(change):
    StopButton.irq(handler=None)
    global Engine
    if Engine:
        CriticalEngineStop()
    else:
        modeSwitched()

def switchMode(change):
    global Mode
    global Engine
    if Mode == ModeSwitch.value():
        return
    if Engine:
        CriticalEngineStop()
    else:
        modeSwitched()

def startEngine(change):
    global lockstart
    if not lockstart and StopButton.value() == 0:
        lockstart = True
        Rotary.clear_handler()
        LCD = DM.getLCD()
        print('start')
        RelayPower.high()
        PowerLEDRelay.high()
        global Engine
        Engine = True

        setLable(1)
        DM.callRendering()

        buzzer.high()
        utime.sleep(0.05)
        buzzer.low()
        utime.sleep(0.1)
        buzzer.high()
        utime.sleep(0.05)
        buzzer.low()

        global DUTY
        global MAX_DUTY
        global PWM
        DUTY = MIN_DUTY_SPEED
        stop = False
        while DUTY > MAX_DUTY-(MAX_DUTY*(SPEED/100)):
            DUTY -= 3251.25
            PWM.duty_u16(round(DUTY))
            utime.sleep(TIME_DUTY_EDIT)
            if StopButton.value() == 1:
                stop = True
                break;
        if not stop:
            Rotary.add_handler(manual_rotary_changed)
            global lockstop
            lockstop = False
            setLable(2)
            print('startED')
        else:
            stopEngine(False)
            
def stopEngine(change):
    global lockstop
    if change == False or not lockstop:
        #if change != False:
            #Ltimer.init(mode=Timer.ONE_SHOT, period=250, callback=lambda t:unlockB())
        lockstop = True
        print('stop')

        Rotary.clear_handler()

        setLable(3)
        DM.callRendering()

        global DUTY
        global MAX_DUTY
        global PWM
        while DUTY < MAX_DUTY:
            DUTY += 3251.25
            PWM.duty_u16(round(DUTY))
            utime.sleep(TIME_DUTY_EDIT)
        
        RelayPower.low()
        PowerLEDRelay.low()
        Rotary.add_handler(rotary_changed)

        setLable(0)
               
        
        global lockstart
        global Engine
        lockstart = False
        Engine = False       
               
        #global Engine
        #Engine = False
        buzzer.high()
        utime.sleep(0.1)
        buzzer.low()
        
        print('stopED')
        
# def unlockB():
#     global Engine
#     Engine = False
#     global lockstart
#     lockstart = False

def revEngine(change):
    global Engine
    global RevButtonLock
    if Engine == False and not RevButtonLock:
        LCD = DM.getLCD()
        RevButtonLock = True

        buzzer.high()
        utime.sleep(0.03)
        buzzer.low()
        utime.sleep(0.04)

        global REV
        global RevLEDRelay
        DM.addInBuffer(LCD.move_to,(13,1))
        if REV:
            RevLEDRelay.low()
            RevPower.low()
            REV = False
            DM.addInBuffer(LCD.putstr,('       '))
        else:
            RevLEDRelay.high()
            RevPower.high()
            REV = True
            DM.addInBuffer(LCD.putstr,('REVERSE'))
        Ltimer.init(mode=Timer.ONE_SHOT, period=250, callback=lambda t:RBL())

def RBL():
    print(0)
    global RevButtonLock
    RevButtonLock = False

def CriticalEngineStop():
    print('Critical')
    LCD = DM.getLCD()
    DM.addInBuffer(LCD.clear,())
    DM.addInBuffer(LCD.move_to,(0,1))
    DM.addInBuffer(LCD.putstr,('Engine is stopping!'))
    DM.callRendering()
    Rotary.clear_handler()
    StopButton.irq(handler=None)
    StartButton.irq(handler=None)
    RevButton.irq(handler=None)
    ModeSwitch.irq(handler=None)
    global DUTY
    global MAX_DUTY
    global PWM
    while DUTY < MAX_DUTY:
        DUTY += 3251.25
        PWM.duty_u16(round(DUTY))
        utime.sleep(0.25)
    RelayPower.low()
    PowerLEDRelay.low()
    global Engine
    Engine = False
    buzzer.high()
    utime.sleep(0.12)
    buzzer.low()
    modeSwitched()

def rotary_changed(change):
    global SPEED
    global Rtimer
    if change == Rotary.ROT_CW:
        if SPEED > 0:
            SPEED -= 5
            setLineSpeedPos(1,SPEED/100)
            #Rtimer.deinit()
    elif change == Rotary.ROT_CCW:
        if SPEED < 100:
            SPEED += 5
            setLineSpeedPos(1,SPEED/100)
            #Rtimer.deinit()
    #Rtimer.init(mode=Timer.ONE_SHOT, period=150, callback=lambda t:setLineSpeedPos(1,SPEED/100))

def manual_rotary_changed(change):
    global Rtimer
    if change == Rotary.ROT_CW:
        global DUTY
        global MAX_DUTY
        global PWM
        global SPEED
        if SPEED > 0:
            SPEED -= 5
            DUTY += 3251.25
            PWM.duty_u16(round(MAX_DUTY-(MAX_DUTY*(SPEED/100))))
            setLineSpeedPos(1,SPEED/100)
            #Rtimer.deinit()
    elif change == Rotary.ROT_CCW:
        global DUTY
        global MAX_DUTY
        global PWM
        global SPEED
        if SPEED < 100:
            SPEED += 5
            DUTY -= 3251.25
            PWM.duty_u16(round(MAX_DUTY-(MAX_DUTY*(SPEED/100))))
            setLineSpeedPos(1,SPEED/100)
            #Rtimer.deinit()
    #Rtimer.init(mode=Timer.ONE_SHOT, period=150, callback=lambda t:setLineSpeedPos(1,SPEED/100))

def auto_rotary_changed(change):
    LCD = DM.getLCD()
    if change == Rotary.ROT_CW:
        global AutoCountProgram
        global AutoSelectProgram
        global AutoNameProgram
        if AutoSelectProgram > 0:
            AutoSelectProgram -= 1
            for u in range(AutoCountProgram):
                DM.addInBuffer(LCD.move_to,(18,u+1))
                if u == AutoSelectProgram:
                    AutoNameProgram = AutoNamesProgram[u]
                    DM.addInBuffer(LCD.putstr,('|◿'))
                else:
                    DM.addInBuffer(LCD.putstr,('|◛'))
                DM.callRendering()
    elif change == Rotary.ROT_CCW:
        global AutoNameProgram
        global AutoCountProgram
        global AutoSelectProgram
        if AutoSelectProgram < AutoCountProgram-1:
            AutoSelectProgram += 1
            for u in range(AutoCountProgram):
                DM.addInBuffer(LCD.move_to,(18,u+1))
                if u == AutoSelectProgram:
                    AutoNameProgram = AutoNamesProgram[u]
                    DM.addInBuffer(LCD.putstr,('|◿'))
                else:
                    DM.addInBuffer(LCD.putstr,('|◛'))
                DM.callRendering()

def setLable(state):
    LCD = DM.getLCD()
    lable = ''
    if state == 0:
        lable = 'Manual | Engine:OFF '
    elif state == 1:
        lable = 'Manual |Engine:START'
    elif state == 2:
        lable = 'Manual | Engine:ON  '
    elif state == 3:
        lable = 'Manual|Engine:toSTOP'
    elif state == 4:
        lable = 'Automatic | Select:'
    DM.addInBuffer(LCD.putstrxy,(lable,0,0))

def setLineSpeedPos(pos,count):
    LCD = DM.getLCD()
    DM.addInBuffer(LCD.putstrxy,(str(round(count*100))+'%  ',7,pos))
    fill = (I2C_NUM_COLS*count)
    empt = (I2C_NUM_COLS - fill)
    DM.addInBuffer(LCD.putstrxy,(('◿')*round(fill) + ('◛')*round(empt),0,pos+1))

def generateLoader():
    odir = os.listdir('usrp')
    check = {'\n'}
    try:
        o = open('TempData.py')
        check = o.readline().replace('#','').split(':')
        o.close()
    except OSError:
            print ('x')
    check.remove('\n')
    if check != odir:
        length = len(odir)
        p = '#'
        for u in range(length):
            o = open('usrp/'+odir[u])
            if o.readline().split(':')[0] == "#load":
                if length-1 != u:
                    p += odir[u]+':'
                else:
                    p += odir[u]+':\n'
            o.close()
        f = open('TempData.py', 'w')
        f.write(p)
        f.write('import sys\n')
        f.write('import uasyncio\n')
        f.write('sys.path.insert(1, "usrp")\n')
        f.write('from machine import Pin\n')
        for i in range(length):
            o = open('usrp/'+odir[i])
            name = odir[i].split('.')[0]
            if o.readline().split(':')[0] == "#load":
                f.write('from '+name+' import mainloader as mainloader'+str(i)+'\n')
            o.close()
        f.write('class Loader:\n')
        f.write('    Task = None\n')
        f.write('    StopButton = None\n')
        f.write('    SwitchButton = None\n')
        f.write('    SwitchValue = None\n')
        f.write('    def setParameters(stop,switch):\n')
        f.write('        Loader.StopButton = Pin(stop,Pin.IN,Pin.PULL_DOWN)\n')
        f.write('        Loader.SwitchButton = Pin(switch,Pin.IN,Pin.PULL_DOWN)\n')
        f.write('        Loader.SwitchValue = Loader.SwitchButton.value()\n')
        f.write('    async def start(loadname,lcd):\n')
        for i in range(length):
            o = open('usrp/'+odir[i])
            if o.readline().split(':')[0] == "#load":
                name = odir[i].split('.')[0]
                f.write('        if loadname == "'+name+'":\n')
                f.write('            Loader.Task = uasyncio.create_task(mainloader'+str(i)+'.start(lcd))\n')
                f.write('            while not Loader.Task.done():\n')
                f.write('                if Loader.StopButton.value() or Loader.SwitchValue != Loader.SwitchButton.value():\n')
                f.write('                    Loader.Task.cancel()\n')
                f.write('                    break\n')
                f.write('                await uasyncio.sleep(0.25)\n')
            o.close()
        f.write('    def getSpeed(loadname):\n')
        for i in range(length):
            o = open('usrp/'+odir[i])
            if o.readline().split(':')[0] == "#load":
                name = odir[i].split('.')[0]
                f.write('        if loadname == "'+name+'":\n')
                f.write('            return mainloader'+str(i)+'.getSpeed()\n')
            o.close()
        f.close()

uasyncio.run(main())
