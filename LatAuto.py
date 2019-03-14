#             ******************************************************************
#
#               PYTHON SCRIPT TO READ SENSOR DATA AND MOVE ETHER TO SMART CONTRACT
#
#               ******************************************************************

import serial
import time
import string
import math
import RPi.GPIO as GPIO
import json
import web3
import subprocess
import numpy as np
from os import system
from web3 import Web3, HTTPProvider, IPCProvider
from web3.contract import Contract
from web3.auto.gethdev import w3
from web3.middleware import geth_poa_middleware
LED1 = 5
LED2 = 6
count=0
act_pump=False
LED3 = 12
LED4 = 13
blockno = 0
# RPi GPIO PINS
GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_KEY =21
a = []
b = []
c = 0
a1 = []
b1 = []
c1 = 0
d = 0
# set GPIO direction
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_KEY, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(LED3, GPIO.OUT)
GPIO.setup(LED4, GPIO.OUT)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(GPIO_KEY, False)
C_ADDR_1 = '0x2771ecac5523f44c8a7133ecd572b9be0b0a7cd4'
C_ADDR = Web3.toChecksumAddress(C_ADDR_1)
submit_block = True
web3=Web3(IPCProvider("/home/pi/10sec_floodblock/n1/geth.ipc"))
web3.middleware_stack.inject(geth_poa_middleware, layer=0)


testcontract =web3.eth.contract(address=C_ADDR,abi=[{"constant":False,"inputs":[],"name":"SendLat","outputs":[],"payable":True,"stateMutability":"payable","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"GPSLat","outputs":[{"name":"","type":"bool"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"meanval","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"uint256"}],"name":"GPS","outputs":[{"name":"","type":"address"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[],"name":"SendWL","outputs":[{"name":"","type":"uint256"}],"payable":True,"stateMutability":"payable","type":"function"},{"constant":False,"inputs":[],"name":"kill","outputs":[],"payable":False,"stateMutability":"nonpayable","type":"function"},{"constant":True,"inputs":[{"name":"","type":"uint256"}],"name":"Alltfence","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[{"name":"incoming","type":"int256[12]"}],"name":"AlternativeFence","outputs":[],"payable":False,"stateMutability":"nonpayable","type":"function"},{"constant":True,"inputs":[],"name":"latitude","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"longitude","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"pumpnode","outputs":[{"name":"","type":"address"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"Waterlevel","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"Longitude","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"setpump","outputs":[{"name":"","type":"bool"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"uint256"}],"name":"Node","outputs":[{"name":"","type":"address"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[],"name":"calcflood","outputs":[{"name":"","type":"bool"}],"payable":False,"stateMutability":"nonpayable","type":"function"},{"constant":True,"inputs":[],"name":"owner","outputs":[{"name":"","type":"address"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"meanval4","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"PreviousWaterlevel","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"floodlvl","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[{"name":"WLset","type":"uint256"}],"name":"SetWL","outputs":[],"payable":False,"stateMutability":"nonpayable","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"Sent","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"Nodecount2","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"Latitude","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"uint256"}],"name":"fence","outputs":[{"name":"","type":"int256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"Nodecount","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[],"name":"SendLong","outputs":[],"payable":True,"stateMutability":"payable","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"votes","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[{"name":"","type":"address"}],"name":"GPSLong","outputs":[{"name":"","type":"bool"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":True,"inputs":[],"name":"meanval3","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"constant":False,"inputs":[],"name":"Fence","outputs":[{"name":"","type":"bool"}],"payable":False,"stateMutability":"nonpayable","type":"function"},{"constant":True,"inputs":[],"name":"floodcount","outputs":[{"name":"","type":"uint256"}],"payable":False,"stateMutability":"view","type":"function"},{"inputs":[],"payable":False,"stateMutability":"nonpayable","type":"constructor"},{"anonymous":False,"inputs":[{"indexed":False,"name":"accountAddress","type":"address"},{"indexed":False,"name":"amount","type":"uint256"}],"name":"LogWaterLevel","type":"event"},{"anonymous":False,"inputs":[{"indexed":False,"name":"accountAddress","type":"address"},{"indexed":False,"name":"amount","type":"uint256"}],"name":"LogLatitude","type":"event"},{"anonymous":False,"inputs":[{"indexed":False,"name":"accountAddress","type":"address"},{"indexed":False,"name":"amount","type":"uint256"}],"name":"LogLongitude","type":"event"}])

## Start PPPD
def openPPPD():
        # Check if PPPD is already running by looking at syslog output
        output1 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -3", shell=True)
        if "secondary DNS address".encode() not in output1 and "locked".encode() not in output1:
                while True:
                        # Start the "fona" process
                        subprocess.call("sudo logrotate -v -f /etc/logrotate.d", shell = True)
                        time.sleep(2)
                        subprocess.call("sudo pon fona", shell=True)
                        time.sleep(2)
                        subprocess.call("sudo route add default gw 10.64.64.64", shell = True)
                        output2 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -3", shell=True)
                        if "script failed".encode() not in output2:
                                break
        # Make sure the connection is working
        while True:
                output2 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
                output3 = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -3", shell=True)
                if "secondary DNS address".encode() in output2 or "secondary DNS address".encode() in output3:
                        return True

def closePPPD():
        print ("turning off cell connection")
        # Stop the "fona" process
        subprocess.call("sudo poff fona", shell=True)
        # Make sure connection was actually terminated
        while True:
                output = subprocess.check_output("cat /var/log/syslog | grep pppd | tail -1", shell=True)
                if "Exit".encode() in output:
                        return True

def GPSoff():
        ser= serial.Serial('/dev/serial0')
        ser.baudrate = 115200
        ser.write(b'AT+CGPS=0\r\n')
        print("GPS turning off")
        time.sleep(5)
        ser.write(b'AT+CGPSINFOCFG=0,3\r\n')
        print("Information Terminating")
        time.sleep(5)


def point_in_poly(x,y,poly):

    n = len(poly)
    inside = False

    p1x, p1y= poly[0]
    for i in range(n+1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        print(xints)
                        print(i)
                    if p1x == p2x or x<= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

def GPS():
        while True:
            line = ser.readline()
            data = line.split(b",")
            if data [0] == ('$GPGGA').encode():
                lats = float(data[2])
                #print("The Latitude is : %4.6f " % (lats))

                lat_dir = data[3]
                #print("Direction of Latitude is " + str(lat_dir))

                longitude = float(data[4])
                #print("the Longitude is : %5.6f" % (longitude))

                long_dir = data[5]
                #print("The long_dir is " + str(long_dir))

                time_stamp = float(data[1])
                #print("The time of fix is : %6.1f" % (time_stamp))

                lats = float(lats)
                lon = float(longitude)
                lats_d = int (lats)
                lats_degree = int (lats_d/100)
                #print (lats_degree)
                lon_d = int (lon)
                lon_degree = int(lon_d/100)
                #print (lon_degree)
                lat_mm = lats % 100
                lon_mm = lon % 100

                converted_latitude = lats_degree + (lat_mm/60)
                converted_longitude = lon_degree + (lon_mm/60)
                if lat_dir == (b"N"):
                    converted_latitude = float(converted_latitude)
                else:
                    converted_latitude = - float(converted_latitude)
                if long_dir == (b"E"):
                    converted_longitude = float(converted_longitude)
                else:
                    converted_longitude = -float(converted_longitude)
                polygon = [(-1.47942,53.36674),(-1.47936,53.36659),(-1.47901,53.36655),(-1.47886,53.36676),(-1.47914,53.36684),(-1.47942,53.36674)]
                point =  point_in_poly(converted_longitude,converted_latitude,polygon)
                #print(point)

                return converted_longitude, converted_latitude, point


def storeData():
        while True:
                line = ser.readline()
                data = line.split(b",")
                if data[0] == (b"$GPRMC"):
                        if data[2] == (b"A") :
                            print("storing data")
                            latgps = float(data[3])
                            latdeg = int (latgps/100)
                            latmin = latgps % 100
                            convertedlat = latdeg + (latmin/60)
                            if data[4] == (b"S"):
                                    convertedlat = - float(convertedlat)
                            else:
                                    convertedlat = float(convertedlat)
                            longps = float(data[5])
                            londeg = int (longps/100)
                            lonmin = longps % 100
                            convertedlon = londeg + (lonmin/60)
                            if data[6] == (b"W"):
                                    convertedlon = -float(convertedlon)
                            else:
                                    convertedlon = float(convertedlon)
                            with open ("position.kml", "w") as pos:
                                    pos.write("""<kml xmlns="http://www.opengis.net/kml/2.2"
 xmlns:gx="http://www.google.com/kml/ext/2.2"><Placemark>
  <name>GPS data</name>
  <description>Adafruit</description>
  <Point>
    <coordinates>%s,%s,0</coordinates>
  </Point>
</Placemark></kml>"""  % (convertedlon,convertedlat))
                            break

def distance():
        # set Trigger HIGH
        ts = int (time.time()*1000000.0)
        #print (ts)
        GPIO.output(GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()
        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
                StartTime = time.time()
        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
                StopTime = time.time()
        ts1 = int (time.time()*1000000.0)
        #print (ts1)
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with speed of sound (34300 cm/s)
        # and divide by 2 for there & back
        distance = (TimeElapsed * 34300) / 2
        tp = ts1 - ts
        a1.append(tp)
        #print(tp)
        return distance

if __name__ == '__main__':
        try:
                # Open cell connecion
                #openCell = openPPPD()


                # subprocess.call("sudo tincd", shell=True)
                #if openCell == True:
                #        print("connection established")
                # ser= serial.Serial('/dev/ttyS0')
                #ser.baudrate = 115200
                #ser.timeout = 1
                #print("checking for fix")
                #ser.write(b'AT+CGPS=1\r\n')
                #time.sleep(7)
                #ser.write(b'AT+CGPSINFOCFG=1,3\r\n')
                #time.sleep(5)

                #while True:
                    #line = ser.readline()
                    #data = line.split(b",")
                    #if data[0] ==(b"$GPRMC"):
                        #if data[2] == (b"A") :
                            #print("GPS position is fixed")
                    #break
                # Define the polygon based on your location if you want to test it
                while True:
                        blockno = (web3.eth.blockNumber)
                        #print("blockno %d" % blockno)
                        #datastore = storeData()
                        #coord = GPS()
                        #print(coord)
                        #longitude,latitude, point = GPS()
                        #print("The converted format for longitude is " +str(longitude))
                        #print("The converted format for latitude is " +str(latitude))

                        #longitude = float(longitude*10e5)
                        #print(longitude)
                        #longitude = longitude + 359000000
                        #longTx = int(longitude)
                        latTx = 412422830;
                        longTx = 357539960;
                        #print(longTx)

                        #latitude = float(latitude*10e5)
                        #print(latitude)
                        #latitude = latitude +359000000
                        #latTx = int(latitude)
                        #print(latTx)
                        #point = bool(point)
                        # insert Tx on next block
                        #while blockno >= int(web3.eth.blockNumber):
                                #pass
                                #time.sleep(blocktime/5)
                                #count=count+1
                        #print("count % d" % count)

                        testcontract.transact({'from': web3.eth.accounts[1], 'gas': 200000, 'value': int(latTx)}).SendLat()
                        testcontract.transact({'from': web3.eth.accounts[2], 'gas': 200000, 'value': int(longTx)}).SendLong()

                        #print(testcontract.call().latitude())
                        #print(testcontract.call().longitude())

                          #count = count +1
                          #time.sleep(5)
                        #fencetest=bool(testcontract.transact({'from':web3.eth.coinbase}).Fence())
                        fencetest=True
                        if fencetest == True:
                                #dist1=distance()
                                #dist2=distance()
                                #dist3=distance()
                                #dist= (dist1 + dist2 + dist3)/3
                                #distTx=int(dist1)
                                #Nodecount=int(testcontract.call().Nodecount())
                                #print("NodeCount=%d" % Nodecount)
                                #Floodcount=int(testcontract.call().floodcount())
                                #print("FLOODCount=%d" % Floodcount)
                                #setpump=int(testcontract.call().setpump())
                                #print("SetPump: %i" % bool(setpump))
                                #print("Water Level % d" % int(distTx))
                                if blockno % 2==0:
                                        print (blockno)
                                        timestamp = int (time.time()*1000.0)
                                        testcontract.transact({'from': web3.eth.coinbase, 'gas': 200000, 'value': int(5)}).SendWL()
                                        #if distTx <= 10:
                                        print ("time of flood : ")
                                        print (timestamp)
                                        a.append (timestamp)
                                        c = c+1
                                        b.append (c)
                                        while blockno >= int(web3.eth.blockNumber):
                                                 count=count+1
                                        blockno = (web3.eth.blockNumber)
                                if blockno % 2!=0:
                                        count=0
                                        print (blockno)
                                        while blockno >= int(web3.eth.blockNumber):
                                                 count=count+1
                                        blockno = (web3.eth.blockNumber)
                                if blockno % 2==0:
                                        print (blockno)
                                        testcontract.transact({'from': web3.eth.coinbase, 'gas': 200000, 'value': int(25)}).SendWL()
                                        while blockno >= int(web3.eth.blockNumber):
                                                 count=count+1
                                        blockno = (web3.eth.blockNumber)
                                if blockno % 2!=0:
                                        count=0
                                        print (blockno)
                                        while blockno >= int(web3.eth.blockNumber):
                                                 count=count+1
                                        blockno = (web3.eth.blockNumber)
                        else:
                                #testcontract.transact({'from': web3.eth.coinbase, 'gas': 100000000, 'value': int(distTx)}).holddata()
                                print("outside the fence")




        # reset by pressing CTRL + C
        except KeyboardInterrupt:
                data = np.array([a, b])
                data = data.T
                for index in range(len(a)):
                       np.savetxt("latencyTF.txt", data, fmt=['%d', '%d'])
                data1 = np.array([a1])
                data1 = data1.T
                for index1 in range(len(a1)):
                       np.savetxt("period_latch.txt", data1, fmt=['%d'])
                print(".")
                print("Measurement stopped by User")
GPIO.cleanup()
