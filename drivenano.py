print("Jetson Nano Robot Starting Up....")
import threading, multiprocessing
import time
import serial
from datetime import datetime
import sys, termios, tty, os
import adafruit_pca9685
from adafruit_servokit import ServoKit
import adafruit_bno055
import board
import busio
import keyboard
import json
import usb
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.compat import iteritems
import pyrealsense2 as rs


global objr,objl,objc,sensor, client, ax1, ax2, ax3, ax4, decode_errors,decode_control,kit,eyemaxy,eyeminy,eyeinitx,eyeinity,eyeminx,eyemaxx,eyevalx,eyevaly,minl,maxr,thrmax,thrmin,turninit,thrinit,turnval,thrval,bt0, avoid, camera, tcpconn, c, systems, uptime, ser, starttime, looptime, pcatry, pcatrytime, serialtry, usbserialtry,serialtrytime, pcago, serialgo, usbserialgo,quitnow
 
thrmax=100
thrmin=60
turninit = 104
 
if len(sys.argv) < 2:
    print('No Arguments, Using Defaults.')
    print(thrmax,thrmin,turninit)
else:
    try:
        print('Using the following Argument/s:')
        if len(sys.argv) == 2:
            thrmax = int(sys.argv[1])
        elif len(sys.argv) == 3:
            thrmax = int(sys.argv[1])
            thrmin = int(sys.argv[2])
        elif len(sys.argv) == 4:
            thrmax = int(sys.argv[1])
            thrmin = int(sys.argv[2])
            turninit = int(sys.argv[3])
        print(thrmax,thrmin,turninit)
    except Exception as e:
        print(e)


def getjoy(queue, stopped):
    global systems,decode_control,decode_errors
    while 1:
        if systems['modbus']:
            try:
                global ax1, ax2, ax3, ax4, bt0, decode_control
                
                address = 128
                count   =  5
                result  = client.read_holding_registers(address, count,  unit=1)
                decoder = BinaryPayloadDecoder.fromRegisters(result.registers)
                decoded = { 'AXIS1': decoder.decode_16bit_uint(), 'AXIS2': decoder.decode_16bit_uint(),'AXIS3': decoder.decode_16bit_uint(), 'AXIS4': decoder.decode_16bit_uint(), 'bt0': decoder.decode_16bit_uint()}
                for name, value in iteritems(decoded):
                    #print ("%s\t" % name, value)
                    if name=='AXIS1':
                        ax1= value
                    if name=='AXIS2':
                        ax2= value      
                    if name=='AXIS3':
                        ax3= value
                    if name=='AXIS4':
                        ax4= value
                    if name=='bt0':
                        bt0=value
                    decode_control = decode_control + 1
            except Exception as e:
                print("Get Joy Error",e)
                systems['modbus'] = False
                decode_errors = decode_errors + 1

def odetect(queue, stopped):
    global objr,objl,objc
    try:
        pipeline = rs.pipeline()
        pipeline.start()
        objr = 2
        objl = 2
        objc = 2
        while True:
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            if not depth: continue
            mindist = 2
            mindistr = 2
            mindistl = 2
            for y in range(100,400,20):
                for x in range(100,320,20):
                    dist = depth.get_distance(x, y)
                    if 0 < dist and dist < mindistl:
                        mindistl=dist
                for x in range(321,540,20):
                    dist = depth.get_distance(x, y)
                if 0 < dist and dist < mindistr:
                    mindistr=dist
                    
            if mindistr< .5 and mindistl< .5:
                print("OBJECT CENTER DISTANCE:",mindistr if mindistr<mindistl else mindistl)
                objc = mindistr if mindistr<mindistl else mindistl
            else:
                if mindistr< .5:
                    print("OBJECT RIGHT DISTANCE:",mindistr)
                    objr = mindistr
                if mindistl< .5:
                    print("OBJECT LEFT DISTANCE:",mindistl)
                    objl = mindistl

                # if y%20 is 19:
                    # line = ""
                    # for c in coverage:
                        # line += " .:nhBXWW"[c//25]
                    # coverage = [0]*64
                   #print(dist)
                    #print(line)
        exit(0)
    except Exception as e:
        print(e)
        pass




def robotlog():
    global uptime, systems,decode_control,decode_errors, sensor
    timenow=str(datetime.now())
    ut=str(uptime)
    temp = sensor.temperature
    tempf = (temp * 9/5) + 32
  
    
    logtext="JetNanoLog - Time: "+timenow+" --Uptime: "+ut+" --Decode Errors: "+str(decode_errors)+" --Decode Msgs: "+str(decode_control)+" -- Temp F: "+str(tempf)
    
    jltext = open("/home/main/logs/jetlog.txt", "a")
    jltext.write(logtext+"\n")
    jltext.write("------Systems Available------"+"\n")
    
    for name in systems:
        systxt =name+" - Offline" if systems[name]==False else name+" - Online"
        jltext.write(systxt+"\n")
    
    jltext.write("-------BNO055 Sensor Readings -----\n")    
    acc="Accelerometer (m/s^2): {}".format(sensor.acceleration)
    mag="Magnetometer (microteslas): {}".format(sensor.magnetic)
    gy="Gyroscope (rad/sec): {}".format(sensor.gyro)
    euang="Euler angle: {}".format(sensor.euler)
    quat="Quaternion: {}".format(sensor.quaternion)
    lacc="Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration)
    grav="Gravity (m/s^2): {}".format(sensor.gravity)
    pc="Primary Control:"+"Connected - ModBus" if systems['modbus'] else "Modbus Unavailable - Using Serial" if systems['serial'] else "No Control Available"
    jltext.write(acc+"\n")
    jltext.write(mag+"\n")
    jltext.write(gy+"\n")
    jltext.write(euang+"\n")       
    jltext.write(quat+"\n")
    jltext.write(lacc+"\n")
    jltext.write(grav+"\n")
    jltext.write(pc+"\n")     
    
    jltext.close()



def checkmodbus():
    global systems, client
    if not systems['modbus']:
        try:
            client = ModbusClient('192.168.0.200', port=502)
            client.connect()
            address = 128
            count   =  5
            result  = client.read_holding_registers(address, count,  unit=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result.registers)
            decoded = { 'AXIS1': decoder.decode_16bit_uint(), 'AXIS2': decoder.decode_16bit_uint(),'AXIS3': decoder.decode_16bit_uint(), 'AXIS4': decoder.decode_16bit_uint(), 'bt0': decoder.decode_16bit_uint()}
            systems['modbus'] = True
        except Exception as e:
            print("Primary Modbus Connection Error")
            
        if not systems['modbus']:
            client = ModbusClient('192.168.50.200', port=502)
            try:
                client.connect()
                address = 128
                count   =  5
                result  = client.read_holding_registers(address, count,  unit=1)
                decoder = BinaryPayloadDecoder.fromRegisters(result.registers)
                decoded = { 'AXIS1': decoder.decode_16bit_uint(), 'AXIS2': decoder.decode_16bit_uint(),'AXIS3': decoder.decode_16bit_uint(), 'AXIS4': decoder.decode_16bit_uint(), 'bt0': decoder.decode_16bit_uint()}
                systems['modbus'] = True
            except Exception as e:
                print("Secondary Modbus Connection Error")


def systemsavailable(queue, stopped):
    #print("Systems Available")
    global uptime, systems,decode_control,decode_errors, sensor
    while 1:
        print(" ")
        print("------Systems Available------Uptime: ", uptime)
        
        for name in systems:
            print(name+" - Offline" if systems[name]==False else name+" - Online")
            
        print("Uptime: ", uptime)
        if systems['gyro']:

            print("----Gyro Readings ----")
        temp = sensor.temperature
        tempf = (temp * 9/5) + 32
        print("Temperature: {} degrees C".format(sensor.temperature))
        print("Temperature: {} degrees F".format(tempf))
        """
        print(
            "Temperature: {} degrees C".format(temperature())
        )  # Uncomment if using a Raspberry Pi
        """
        print("Accelerometer (m/s^2): {}".format(sensor.acceleration))
        print("Magnetometer (microteslas): {}".format(sensor.magnetic))
        print("Gyroscope (rad/sec): {}".format(sensor.gyro))
        print("Euler angle: {}".format(sensor.euler))
        print("Quaternion: {}".format(sensor.quaternion))
        print("Linear acceleration (m/s^2): {}".format(sensor.linear_acceleration))
        print("Gravity (m/s^2): {}".format(sensor.gravity))
        print()    

        print("Primary Control:", "Connected - ModBus" if systems['modbus'] else "Modbus Unavailable - Using Serial" if systems['serial'] else "No Control Available")
        print("Total Decode Errors:", decode_errors)
        print("Control Messages:", decode_control)
        print("Control Messages per second:", decode_control/3)
        systems['control'] = False if decode_control<4 else True
        print(" ---")
        # if systems['pca'] and not systems['control']:
            # steering(0,0,0,0)
        #decode_errors = 0
        robotlog()
        decode_control = 0
        checkmodbus()
        
        time.sleep(3)


def steering(inax1,inax2,inax3,inax4):
    global bs1,kit,minl,maxr,turninit,turnval,systems, thrmax,thrmin,thrinit,thrval
    OldjRange = 50944
    NewjRange = (maxr - minl)  
    turnval=(((inax3) * NewjRange) / OldjRange) + minl
    OldtRange = 50944
    NewtRange = (thrmax- thrmin)  
    thrval=(((inax4) * NewtRange) / OldtRange) + thrmin
    if(turnval<turninit):
        bs1=turninit+(turninit-turnval)
    else:
        bs1=turninit-(turnval-turninit)
    if systems['pca']:
        kit.servo[0].angle = thrval
        kit.servo[1].angle = turnval
        kit.servo[2].angle = bs1
    else:
        if inax3 != 0 or inax4 !=0 or inax1 != 0 or inax2 !=0:
            print("STR: ",turnval,"THR: ",thrval)
  
def OpenUSBSerialPort(port=""):
    print ("Open port %s" % port)
    fio2_ser = None
    try:
        fio2_ser = serial.Serial(port,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity =serial.PARITY_ODD)
    except serial.SerialException as msg:
        print( "Error opening USB serial port %s" % msg)
    except:
        exctype, errorMsg = sys.exc_info()[:2]
        print ("%s  %s" % (errorMsg, exctype))
    return fio2_ser

def OpenSerialPort(port=""):
    global systems
    print ("Open port %s" % port)
    fio2_ser = None
    try:
        fio2_ser = serial.Serial(
            port = "/dev/ttyTHS1",
            baudrate = 9600,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
        )
        systems['serial'] = True
    except serial.SerialException as msg:
        print( "Error opening serial port %s" % msg)
    except:
        exctype, errorMsg = sys.exc_info()[:2]
        print ("%s  %s" % (errorMsg, exctype))
    return fio2_ser


def Read_Data(queue, serialPort, stopped):
    global decode_errors,systems
    while not stopped.is_set(): 
        if serialPort:
            fio2_data = ''       
            try:                    
                fio2_data = serialPort.readline()
            except:
                exctype, errorMsg = sys.exc_info()[:2]
                print ("Error reading port - %s" % errorMsg+" : "+serialPort.port)
                time.sleep(4)
                serialPort.close()
            if len(fio2_data) > 0:
                queue.put(fio2_data)
            else:
                queue.put("Read_Data() no Data")
    print ("Read_Data finished.")

def Disp_Data(queue, stopped):
    global decode_errors,decode_control,systems,ax1,ax2,ax3,ax4,bt0, v1c, v2c
    print ("Disp_Data started")
    while not stopped.is_set():
        dcd=False
        if queue.empty() == False and  (queue.get() != "Read_Data() no Data"):        
            fio2_data = queue.get()
            #print(fio2_data)
            try:
                dd=fio2_data.decode("utf-8")
                #print(dd)
                teensy = dd[0:1]
                if teensy =="T":
                    voltage1=round(float(dd[2:6])/100,2)
                    voltage2=round(float(dd[7:11])/100,2)
                    v1c = int(convertscales(voltage1, 13, 0, 100, 0))
                    v2c = int(convertscales(voltage2, 13, 0, 100, 0))
                else:
                    if not systems['modbus'] and systems['serial']:
                        ax1=float(dd[9:15])
                        ax2=float(dd[16:22])
                        ax3=float(dd[24:30])
                        ax4=float(dd[32:38])
                        bt0=float(dd[41:42])
                        decode_control = decode_control + 1
                        dcd=True
                        print(ax1,ax2,ax3,ax4,bt0)
                    #time.sleep(.1)
            except Exception as e:
                print("Error Decoding this:"+str(fio2_data))
                decode_errors = decode_errors + 1
                dcd=False


systems={'modbus':False, 'serial':False, 'usbserial':False, 'pca':False, 'uptime':False, 'gyro':False, 'camera':False, 'realsense':False, 'control':False}
sysmod= systems['modbus']
systemlist = list(systems)
busses = usb.busses()
for bus in busses:
   devices = bus.devices
   for dev in devices:
        product = usb.util.get_string(dev.dev, dev.dev.iProduct)
        if "RealSense" in product:
            systems['realsense'] = True


minl=40
maxr=120
# thrmax=100
# thrmin=60
# turninit = 86
thrinit = 75
turnval=turninit
thrval=thrinit
eyeinitx = 80
eyeinity = 80
eyeminx = 0
eyemaxx = 180
eyeminy = 0
eyemaxy = 180
eyevalx=eyeinitx
eyevaly=eyeinity
ax1=0
ax2=0
ax3=0
ax4=0
bt0=0
avoid = False
pcatry=1
serialtry=1
usbserialtry=1
quitnow=False
pcago=False
serialgo=False
usbserialgo=False
starttime = datetime.now()
executionTime=0
decode_errors=0
decode_control = 0
tcpconn = False
camera=False
cameraport='/dev/video0'

try:
    ser = serial.Serial(
        port = "/dev/ttyTHS1",
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS,
    )
    
except Exception as e:
    print("SERIAL CONNECTION ERROR")
    systems['serial'] = False

try:
    print("Initializing IO System - ServoKit")
    i2c = busio.I2C(board.SCL, board.SDA)
except Exception as e:
    print("I2C CONNECTION ERROR")

try:
    print("Trying IO System - PCA Try #:"+str(pcatry))
    kit = ServoKit(channels=16, address=0x40)
    print("Initializing IO System - I2C board")
    pca = adafruit_pca9685.PCA9685(i2c)
    print("Initializing IO System - PCA9685")
    pca.frequency = 100
    print("Initializing IO System Freq")

    kit.servo[1].angle = turninit
    pcago= True
    systems['pca'] = True
except Exception as e:
    print("PCA CONNECTION ERROR")

try:
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    systems['gyro'] = True
except Exception as e:
    print("GYRO CONNECTION ERROR")
    
checkmodbus()

uptimenow=datetime.now()
uptime = uptimenow-starttime

zloop = 0   
lax1 = 0
lax2 = 0
lax3 = 0
lax4 = 0
            
if __name__ == "__main__":
    serialPort = OpenSerialPort('/dev/ttyTHS1')
    queue = multiprocessing.Queue()
    stopped = threading.Event()
    p1 = threading.Thread(target=getjoy, args=(queue, stopped,))
    p2 = threading.Thread(target=systemsavailable, args=(queue, stopped,))
    p3 = threading.Thread(target=Read_Data, args=(queue, serialPort, stopped,))
    p4 = threading.Thread(target=Disp_Data, args=(queue, stopped,))
    p5 = threading.Thread(target=odetect, args=(queue, stopped,))
    p1.start()
    p2.start()
    p3.start()
    p4.start()
    p5.start()
    keeploop = 1
    loopcnt = 1
    systems['uptime'] = True
    while not stopped.is_set():
        loopcnt += 1
        #print ("main() %d" % loopcnt)
        try:
            if lax1 == ax1 and lax2 == ax2 and lax3 == ax3 and lax4 == ax4:
                if zloop < 2:
                    steering(ax1,ax2,ax3,ax4)
                    lax1 = ax1
                    lax2 = ax2
                    lax3 = ax3
                    lax4 = ax4
                    zloop = zloop + 1
            else:
                steering(ax1,ax2,ax3,ax4)
                lax1 = ax1
                lax2 = ax2
                lax3 = ax3
                lax4 = ax4
            #steering(ax1,ax2,ax3,ax4)
            uptimenow=datetime.now()
            uptime = uptimenow-starttime
            time.sleep(.01)
            keeploop += 1
        except KeyboardInterrupt: #Capture Ctrl-C
            print ("Captured Ctrl-C")
            loopcnt=1
            keeploop = 1
            stopped.set()
    stopped.set()
    print("Stopped")
    p1.join()
    p2.join()
    p3.join()
    p4.join()
    p5.join()
    print("Done")

