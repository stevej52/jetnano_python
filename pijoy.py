#Import libraries
##################
import time
import pygame
import keyboard
import serial
import subprocess as sp
from pyModbusTCP.client import ModbusClient

global systems

systems={'modbus':False, 'serial':False,'bluetooth':False}


MODBUS_SERVER_IP="192.168.0.200"
c = ModbusClient(host=MODBUS_SERVER_IP, port=502, auto_open=True)
c.host(MODBUS_SERVER_IP)
c.port(502)


if c.write_multiple_registers(133, [1, 0]):
    systems['modbus'] = True
else:
    systems['modbus'] = False

if not systems['modbus']:
    MODBUS_SERVER_IP="127.0.0.1"
    c = ModbusClient(host=MODBUS_SERVER_IP, port=502, auto_open=True)
    c.host(MODBUS_SERVER_IP)
    c.port(502)
    if c.write_multiple_registers(133, [1, 0]):
        systems['modbus'] = True
    # else:
        # import os
        # os.system("sudo python3 sync_server.py")  
        #time.sleep(3)
        # MODBUS_SERVER_IP="127.0.0.1"
        # c = ModbusClient(host=MODBUS_SERVER_IP, port=502, auto_open=True)
        # c.host(MODBUS_SERVER_IP)
        # c.port(502)
        # if c.write_multiple_registers(133, [1, 0]):
            # systems['modbus'] = True


def systemsavailable():
    print("Systems Available")
    global systems
    for name in systems:
        print(name+" - Offline" if systems[name]==False else name+" - Online")
    

def waitbt():
    global systems
    stdoutdata = sp.getoutput("hcitool con")
    while "EC:83:50:2C:A1:C2" not in stdoutdata.split():
        print("Waiting For Bluetooth Device...")
        time.sleep(1)
        stdoutdata = sp.getoutput("hcitool con")

    print("-Connecting...-")
    systems['bluetooth'] = True
    time.sleep(5)


def rescaleAxis(axisPosition,axisIniPosition):

  rescaledPosition=0
  
  lowSegmentLen=1+axisIniPosition
  upperSegmentLen=1-axisIniPosition
  
  if axisPosition<axisIniPosition:
    rescaledPosition=((axisPosition-axisIniPosition)/lowSegmentLen)
  else:
    rescaledPosition=((axisPosition-axisIniPosition)/upperSegmentLen)
  return rescaledPosition
  
 
waitbt()

pygame.joystick.init()
joy0=pygame.joystick.Joystick(0)
joy0.init()
pygame.display.init()

try:
    ser = serial.Serial(
        port = "/dev/ttyS0",
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize = serial.EIGHTBITS
    )
    print("Serial status: " + str(ser.isOpen()))
    systems['serial'] = True
except Exception as e:
    print("Serial ERROR")
    systems['serial'] = False





#initialize Joystick position
pygame.event.pump()
axis0Ini=joy0.get_axis(0)
axis1Ini=joy0.get_axis(1)
axis2Ini=joy0.get_axis(2)
axis3Ini=joy0.get_axis(3)


systemsavailable()


loopcount=1

print("--Joystick Control Active--")
while True:
  try:
    if keyboard.is_pressed('q'):  # if key 'q' is pressed 
      print('You Pressed q Key! The program stopped.')
      break
    else:
          loopcount=loopcount+1
          pygame.event.pump()
          axis0=joy0.get_axis(0)
          axis0=rescaleAxis(axis0,axis0Ini)
          axis1=joy0.get_axis(1)
          axis1=rescaleAxis(axis1,axis1Ini)
          axis2=joy0.get_axis(2)
          axis2=rescaleAxis(axis2,axis2Ini)
          axis3=joy0.get_axis(3)
          axis3=rescaleAxis(axis3,axis3Ini)
          bt0=joy0.get_button(0)
          
     #     print([round(axis0,2),round(axis1,2),round(axis2,2),round(axis3,2),round(bt0)])
          strloop = str(loopcount).zfill(7)
          sendstring = ":"+strloop+":"+str(round(axis0,2)).zfill(5)+" * "+str(round(axis1,2)).zfill(5)+" * "+str(round(axis2,2)).zfill(5)+" * "+str(round(axis3,2)).zfill(5)+" * "+str(round(bt0))+'\n'
          #print(sendstring)
          if systems['modbus']:
              if c.write_multiple_registers(128, [int((axis0+1)*100),int((axis1+1)*100),int((axis2+1)*100),int((axis3+1)*100),int(bt0)]):
                pass
              else:
                print("Modbus Error 1")
                systems['modbus'] = False
          else:
            if systems['serial'] and not systems['modbus']:
                print("Sending..."+str(loopcount))
                ser.write(sendstring.encode())
                time.sleep(.09)
                
          
          
          
          stdoutdata = sp.getoutput("hcitool con")
          if "EC:83:50:2C:A1:C2" not in stdoutdata.split():
              print("--Joystick Control Disabled--")
              if c.write_multiple_registers(133, [1, 0]):
                systems['modbus'] = True
              systems['bluetooth'] = False
              pygame.joystick.quit()
              waitbt()
              pygame.joystick.init()
              joy0=pygame.joystick.Joystick(0)
              joy0.init()
              pygame.display.init()
              print("--Joystick Control Active--")
  except:
    break

