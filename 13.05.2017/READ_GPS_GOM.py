import serial
import time
import datetime
import math
from socketIO_client import SocketIO, BaseNamespace
#from sensor_supersonic import Sensor_SuperSonic
import json
from threading import Thread
import os

from picamera import PiCamera
import base64
import cv2
from picamera.array import PiRGBArray
from socketIO_client import SocketIO, BaseNamespace
from PIL import Image

image = ''
str_image =''
camera = PiCamera()
camera.brightness = 40
camera.resolution = (240, 160)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(240, 160))

time.sleep(30)


#print "Connect 3G"
#os.system('sudo wvdial MF190S')
#print "Connected 3G"

#time.sleep(10)
print "Connected"
ser = serial.Serial('/dev/ttyUSB0',115200)

#ob sensor supersonic
#Supersonic = Sensor_SuperSonic()

owner = '58fd70d933948af8b77428a1'
ID = '5903a847f43fc4731ec69062'
domain = '103.221.220.199'

LON_int = 1810000000
LAT_int = 910000000
ALT_int = 0

LON_STOP = 0
LAT_STOP = 0


flag_begin = 0
listLAT = []
listLON = []

count = 0
maxcount = 0

x=0
y=0
magXMax=0
magXMin=0
magYMax=0
magYMin=0

clock_hold = 0  #check when finish get position hold
begin_go = 0    #check begin send gps fake
check_Hold = 0  #check position hold when finish = 100
check_Fly = 0   #check Quadcopter is fly?

LAT_TE = 0
LON_TE = 0

check_End = 1 #Finish go to poit and will go home
time_sleep=0  #time sleep

he = 0

compassPayload = bytearray()
gpsPayload = bytearray() #data of gps and compass
GPS = bytearray()

TIME = bytearray()
LON = bytearray()  #array save byte of LONGTUDE
LAT = bytearray()  #array save byte of LATITUDE
ALT = bytearray()  #array save byte of ALTITUDE

LON_H = 0   #LONG of position hold
LAT_H = 0   #LAT of positon Hold
ALT_H = 0

Lon_to_byte = bytearray() 
Lat_to_byte = bytearray()

check_finish = 0 #finished go to point
#------------------------------------------------------------------"
#CAMERA
#------------------------------------------------------------------"


def CAMERA():
    global image
    global camera
    global rawCapture
    global frame
    global str_image
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        s = time.time()
        #Get Frame from camera
        image = frame.array
        print type(image)     
        #Add string because Web reading
        str_image = 'data:image/jpeg;base64,'
        #Decode from numpy to image jpeg
        cv2.imwrite('.jpg',image)
        #encode image jpeg because base64 of image small
        str_image += base64.b64encode(open('.jpg','rb').read())

        #Create thread send Image
        thr_send=Thread(target=sendImage)
        thr_send.start()
        #Destroy cv2
        cv2.destroyAllWindows()
        #Next frame
        rawCapture.truncate(0)
        f = time.time()
        print f-s
def sendImage():
    global image
    global str_image
    api_namespace.emit('stream', str_image)
#------------------------------------------------------------------"
    #FUNCTION
#------------------------------------------------------------------"
def handleControl(*args):
    global flag_begin
    global check_End
    print('Handle control')
    data = args[0]
    print(data.get('controlKey'))
    if (data.get('controlKey')=='Start'):
        flag_begin = 111
        check_End = 1
    elif (data.get('controlKey')=='Stop'):
        flag_begin = 100
    elif (data.get('controlKey')=='gohome'):
        check_End = 0
        count = 0
        maxcount = 0
        LON_STOP = 0
        LAT_STOP = 0
    print(data.get('value'))
    print flag_begin
#------------------------------------------------------------------"
#------------------------------------------------------------------"
#convert int to 4 byte
def int_from_byte(b3,b2,b1,b0):
    return (((((b3 << 8)  + b2) << 8) +b1) << 8 +b0)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#Sent byte to Naza
def sendbyte(byte_array):
    ser.write(byte_array)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#XOR byte and Mask of GPS      
def decode(idx):
    return (gpsPayload[idx] ^ gpsPayload[58])

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#XOR byte and Mask of COMPASS
def decodeshot(idx,mask):
    return (compassPayload[idx] ^ mask)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#convert int to 4 byted
def int_to_byte(x):
    a = bytearray()
    a.append((x>>24) & 0xff)
    a.append((x>>16) & 0xff)
    a.append((x>>8) & 0xff)    
    a.append((x) & 0xff)
    return a

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# conver bytesarray to int
def bytes_to_int(a):
    #4-3-2-1
    return reduce(lambda s,x: s*256 + x,a)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# function set interval
def set_interval(func, sec):
    def func_wrapper():
        set_interval(func, sec)
        func()
        # print('sendLocation')
    t = threading.Timer(sec, func_wrapper)
    t.start()
    return t

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# function muster
def muster():
    lng = 105.7688720
    lat = 10.0307620
    heading = 0
    api_namespace.emit('droneMuster', json.dumps({'id' : ID, 'owner':
        owner, 'location' : {'lng' : lng, 'lat' : lat}, 'heading': heading}))

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# function send location
def sendLocation():
    global LON_int
    global LAT_int
    global he
    he =180
    while (True):
        if LAT_int >= -900000000 and LAT_int <= 900000000:
            if LON_int >= -1800000000 and LON_int <= 1800000000:
                print "====================================================== SEND GPS TO SERVER"
                api_namespace.emit('droneSendLocation',  json.dumps({'id' : ID, 'f_id' :
                    "F001", 'location' : {'lng' : LON_int, 'lat' : LAT_int}, 'heading': he}))
        time.sleep(3)
#------------------------------------------------------------------"
#------------------------------------------------------------------"
# function user send control
def userSendControl(*args):
    print(args[0].get('idControl'))


#------------------------------------------------------------------"
#------------------------------------------------------------------"
# function update destination of dronee
def updateDestination(*args):
    print('User send array destination')
    data = args[0]
    global listLON
    listLON = []
    global listLAT
    listLAT = []
    global maxcount
    maxcount= 0
    print data
    for i in data:
        if len(data) > 0:
            t = i
            maxcount +=1
            listLON.append(int(t.get('lng')))
            listLAT.append(int(t.get('lat')))
            print str(t.get('lng'))  + "  " + str(t.get('lat'))
        else:
            print('array null')
#------------------------------------------------------------------"
#------------------------------------------------------------------"
#Calculator gps fake
def Analysis_GPS(inp, position_Hold, position_Active):
        D = 0
        distance_H_A = 0
        #GPS Position hold is H
        H = position_Hold
        #GPS Position know is A
        A = position_Active
        #distance from H to A
        distance_H_A = A - H
       #GPS doi xung cua D qua A
        D = (A - (inp - A))
        #GPS will send to NAZA (diem hien tai ao de NAZA di den diemminh muon)
        return (D - distance_H_A)

#------------------------------------------------------------------"
#get longtitude
#------------------------------------------------------------------"
def GET_LON():
    LON = bytearray()
    for i in range(0,4):
        LON.append(decode(11 - i))
    return bytes_to_int(LON)

#------------------------------------------------------------------"
#get latitude
#------------------------------------------------------------------"
def GET_LAT():
    LAT = bytearray()
    for i in range(0,4):
        LAT.append(decode(15 - i))
    return bytes_to_int(LAT)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#get ALtitude
def GET_ALT():
    ALT = bytearray()
    for i in range(0,4):
        ALT.append(decode(19 - i))
    return bytes_to_int(ALT)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#GET SPEED for GPS
def GET_SPD():
    nVEL = bytearray()
    eVEL = bytearray()
    for i in range(0,4):
        nVEL.append(decode(35 - i))
        eVEL.append(decode(39 - i))
    nVel = (bytes_to_int(nVEL)/100) #Meter
    eVel = (bytes_to_int(eVEL)/100) #Meter
    return math.sqrt(nVel * nVel + eVel * eVel)
#------------------------------------------------------------------"
#ENCODE CODE HEADING
#------------------------------------------------------------------"
def HEADING():
    global compassPayload
    global he
    global magXMax,magXMin,magYMax,magYMin
    mask = compassPayload[8]
    mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3 ) & 0xF0) ^ (((mask & 0x01) << 3) | (mask & 0x01) << 7))
                    
    CompassX = bytearray()
    CompassY = bytearray()
    for i in range(0,2):
        CompassX.append(decodeshot((5 - i), mask))
        CompassY.append(decodeshot((7 - i), mask))

    x = bytes_to_int(CompassX)        
    y = bytes_to_int(CompassY)
                    
    if (x > magXMax):
        magXMax = x
    if (x < magXMin):
        magXMin = x

    if (y > magYMax):
        magYMax = y
    if (y < magYMin):
        magYMin = y
                        
    he = math.atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) *180 / math.pi
    if he < 0:
        he = he + 360.0
#------------------------------------------------------------------"
#READ MODULE GPS NAZA M_LITE
#------------------------------------------------------------------"
def READ_GPS():
    while(1):
        global gpsPayload
        global compassPayload
        global LON_int
        global LAT_int
        global ALT_int
        global flag_begin
        global LON_H
        global LAT_H
        global ALT_H
        global check_Hold
        global LON_STOP
        global LAT_STOP
        global time_sleep
        global check_finish
        global check_End
        global LON_STOP
        global LAT_STOP
        global GPS
        global count
        compassPayload_F = bytearray()
        tmp = (ser.read(1))
        if (tmp.encode("hex") =="55"):
                compassPayload_F.append(tmp)
                codeaa = (ser.read(1))
                
                if (codeaa.encode("hex") == "aa"):
                    compassPayload_F.append(codeaa)
                    codeid = (ser.read(1))
                    compassPayload_F.append(codeid)
                    
                    if codeid.encode("hex") == "20" or codeid.encode("hex") == "10":
                        data_lenght = ser.read(1) 
                        compassPayload_F.append(data_lenght)
                        
                        if (codeid.encode("hex") == "20"):
                            compassPayload = bytearray()
                            for i in range(0,4):
                                compassPayload.append(compassPayload_F[i])
                                
                            for i in range(0,int(compassPayload[3]) + 2):
                                compassPayload.append(ser.read(1))
                            HEADING()
                            sendbyte(compassPayload)
                        elif (codeid.encode("hex") == "10"):
                            
                            gpsPayload = bytearray()
                            for i in range(0,4):
                                gpsPayload.append(compassPayload_F[i])

                            #mgsLen 62 from 0 to 62
                            #read 1 byte, read until read = leng messeger
                            for i in range(0,int(gpsPayload[3]) + 2):
                                gpsPayload.append(ser.read(1))
                                
                            LON_int = GET_LON()
                            LAT_int = GET_LAT()
                            ALT_int = GET_ALT()

                            #Finish read GPS
                            GET_POSITION_HOLD()
                            
                            if (listLAT == [] or listLON == []):
                                print "No data GPS Point"
                            elif check_Hold>=10:
                                print "---------------Dang o diem thu -------------- : " + str(count + 1)
                                GPS = bytearray()
                                #decode
                                if (listLON[count] >= LON_int - 80 and listLON[count] <= LON_int + 80 and check_finish == 0):
                                    if (listLAT[count] >= LAT_int - 80 and listLAT[count] <= LAT_int + 80):
                                        print " ===================>>> Go to the Point finish"
                                        check_finish = 1
                                        #Sent to server event finish go to a Point
                                        api_namespace.on('Finish_Point', True)

                                        time_sleep = 0
                                elif (check_finish == 1):
                                    print "================>>> Sleep on the Point: <<<================ " + str(time_sleep)
                                    time_sleep = time_sleep + 1
                                    time.sleep(0.25)
                                    if time_sleep == 10:
                                        if count < maxcount-1:
                                            count = count + 1
                                        else:
                                            print "**************** No Data Next Point *******************"
                                        check_finish = 0
                                        print " ===================>>>"
                                        print " ===================>>>"
                                        print " ===================>>>"
                                        print " ===================>>>"
                                        print " ===================>>>"
                                        print " ===================>>>"
                                        print "Go to the next Point"
                                    
                                if (check_Hold>=10):
                                    if check_End == 1:
                                        if (listLAT[count] != 0 and listLAT[count] != 0):
                                            #print "Begin send GPS Fake"
                                            if flag_begin == 100:
                                                print "STOP FIGHT"
                                                if LON_STOP == 0 or LAT_STOP == 0:
                                                    LON_STOP = LON_int
                                                    LAT_STOP = LAT_int
                                                LON_D = LON_STOP
                                                LAT_D = LAT_STOP
                                            elif flag_begin == 111:
                                                LON_D = listLON[count]
                                                LAT_D = listLAT[count]
                                            print "Diem den: ------------" + str(LON_D) + "  " +str(LAT_D)
                                            print "DIem Int: ------------"  +str(LON_int) + "  " +str(LAT_int)
                                            print "PosiHold:  -----------" + str(LAT_H) + " " + str(LON_H)
                                            LON_to = Analysis_GPS(LON_D, LON_H, LON_int)
                                            LAT_to = Analysis_GPS(LAT_D, LAT_H, LAT_int)
                                                
                                            print "Diem ao: -------------" + str(LON_to) + "  " +str(LAT_to)
                                            
                                            #encode to byte fron int Lon
                                            Lon_to_byte = bytearray()
                                            
                                            #encode to byte fron int Lat
                                            Lat_to_byte = bytearray()

                                            Lat_to_byte = int_to_byte(LAT_to)
                                            Lon_to_byte = int_to_byte(LON_to)
                                            #calculator checksum again
                                            #Calculator position 62-ArithmeticError
                                            cs1 = 0
                                            cs2 = 0                                              
                                            #change value LONG request values 8 -11 of LON, 15-19 of LAT
                                            for i in range(0,62):
                                                if (i>=8 and i<=11):
                                                    #add vao bytearray GPS reverse, so 4=>1
                                                    GPS.append(Lon_to_byte[11-i] ^ gpsPayload[58])
                                                elif (i>=12 and i<=15):
                                                    GPS.append(Lat_to_byte[15-i] ^ gpsPayload[58])
                                                else:
                                                    GPS.append(gpsPayload[i])
                                                if i>=2 and i<=61:
                                                    cs1 = cs1 + int(GPS[i])
                                                    if cs1>=256:
                                                        cs1 = cs1 - 256
                                                    cs2 = cs2  + cs1
                                                    if cs2>=256:
                                                        cs2= cs2 - 256
                                            GPS.append(cs1)
                                            GPS.append(cs2)
                                    elif (check_End == 0):
                                        print "#####  ---- Go home ---- #####"
                                        FINISH_FLY()
                            print "Co hieu: "  + str(flag_begin)
                            SEND_GPS_TO_NAZA()
        #time.sleep(1)
#------------------------------------------------------------------"
#GET POSITION HOLD
#------------------------------------------------------------------"
def GET_POSITION_HOLD():
    global LON_H
    global LAT_H
    global ALT_H
    global LAT_int
    global LON_int
    global ALT_H
    global check_Hold
    global flag_begin
    global check_Fly
    if (LON_H == 0 and LAT_H == 0):
        LON_H = LON_int
        LAT_H = LAT_int
        ALT_H = ALT_int
    #Function calculator Position Hold
    #if lon_int, lat_int not = 0 and
    if (flag_begin == 111):
    #restart position stop
        LON_STOP = 0
        LAT_STOP = 0
        if (check_Hold <= 9 and LON_int != 0 and LAT_int != 0):
            if (LON_H >= LON_int - 20 and LON_H <= LON_int + 20):
                if (LAT_H >= LAT_int - 20 and LAT_H <= LAT_int + 20):                                    
                    check_Hold = check_Hold + 1
                    sendbyte(gpsPayload)
                    LON_H = LON_int
                    LAT_H = LAT_int
                    ALT_H = ALT_int
                    print "CLock Position Hold in 10s: " + str(check_Hold)
                   
                else:
                    check_Hold = 0
                    LON_H = 0
                    LAT_H = 0
                    ALT_H = 0
                    print "Check again"
            else:
                check_Hold = 0
                LON_H = 0
                LAT_H = 0
                ALT_H = 0
                print "Check again"
        #create fly moi
        #if check_Hold >=10 and check_Fly == 0:
         #   check_Fly == 1
          #  api_namespace.on('Finish_Fly', check_Fly)
        elif LON_int == 0 and LAT_int == 0:
            print "NO DATA FROM GPS"
    else:
        print "No Command begin"
        if flag_begin==100:
            print "----STOP----"

#------------------------------------------------------------------"
#SEND GPS SWITCH CASE
#------------------------------------------------------------------"
def SEND_GPS_TO_NAZA():
    global gpsPayload
    global GPS
    global check_End
    global check_Hold
    global listLAT,listLON
    #Send gps no fix when preskey Stop no point
    if flag_begin == 0 or check_End == 0:
        print "Send data GPS no fix"
        sendbyte(gpsPayload)
    elif (flag_begin == 111 or flag_begin) and check_Hold>=10:
        print "Send data GPS fix"
        sendbyte(GPS)
#------------------------------------------------------------------"
#FINISH FLY
#------------------------------------------------------------------"
def FINISH_FLY():
    global LON_H
    global LAT_H
    global LAT_int
    global LON_int
    global check_Fly
    global Supersonic
    if ( LON_H>= LON_int - 80 and LON_H <= LON_int + 80):
        if (LAT >= LAT_int - 80 and LAT <= LAT_int + 80):
            if check_Fly == 1:
                if Supersonic.Down() <= 50:
                    check_Fly = 2
                    api_namespace.on('Finish_Fly', check_Fly)          
#------------------------------------------------------------------"
#------------------------------------------------------------------"
#------------------------------------------------------------------"
#------------------------------------------------------------------"
#--------------------------------------------------
#           def & conf socketIO
#--------------------------------------------------

# def socketIO
print "Tao ket noi"
socketIO = SocketIO(domain, 3000)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# khai bao duong dan api khi url api khac voi url mac dinh
api_namespace = socketIO.define(BaseNamespace, '/api')

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# join drone into room in server
api_namespace.emit('droneJoin', json.dumps({'id' : ID, 'owner': owner,
'location' : {'lng' : 105.7688720, 'lat' : 10.0307620}, 'heading': 0}))

#------------------------------------------------------------------"
#------------------------------------------------------------------"
# def listener
api_namespace.on('muster', muster)

api_namespace.on('userSendControl', userSendControl)

api_namespace.on('updateDestination', updateDestination)

api_namespace.on('sendControl', handleControl)

#------------------------------------------------------------------"
#------------------------------------------------------------------"
#Multithread
thr=Thread(target=READ_GPS)
thr_send=Thread(target=sendLocation)
thr_image=Thread(target=CAMERA)

thr_image.start()
thr.start()
thr_send.start()

#------------------------------------------------------------------"
#------------------------------------------------------------------"
socketIO.wait()
