#GPS simulator for MAIA - sending a virtual trip to UDP port simulating GPS and instruments on board
simversion=65
print("version " + str(simversion))

import time, datetime
from datetime import datetime, timedelta
import socket
from socket import *
from threading import Thread
from time import sleep
import math
import random

UDPaddress='255.255.255.255'
UDPport=2000

# AUTOHELM SEATALK 1 INSTRUMENTS
# Standard NMEA0183 strings
DBT="$IIDBT,0.00,f,0.00,M,," # Depth Below Transducer
VHW="$IIVHW,,,123.45,M,5,N,," # heading (magnetic compass) and water speed 
VWR="$IIVWR,0.00,R,0.00,N,,,," #Relative Wind Speed and Angle
MTW="$IIMTW,0.00,C" #Mean Temperature of Water
HDM="$IIHDM,222.,M" # Heading - Magnetic - Only broadcasted if Auto pilot power is turned on.
HSC="$IIHSC,,,333,M" # Heading Steering Command. Auto pilot heading. Only broadcasted in AUTO mode.

# Standard NMEA0183 strings from GPS
RMC="$GPRMC,hhmmss,A,4916.45,N,12311.12,W,000.5,054.7,yymmdd,020.3,E*68" # Sample GPS string with addition of SOGmax

# Custom NMEA0183 strings from sensors
VOL="$IIVOL,VOL1,VOL2,VOL3,VOL4,VOL5,VOL6,V"# Analogue readings of voltage. To be expanded with level sensors
WEA="$GPWEA,tIN,tOUT,pressure,humidity,tENGINEroom," # string carrying the meteorological information (temps, humidity and pressure)

# Other data strings
VOL1=10.9
VOL2=11.2
VOL3=5.1

submitperiod=1 # time between UDP messages

# Start position as (Google) angles
lat=56.71465 #55.942508
lon=11.50691 #11.870814
latprev=lat
lonprev=lon
dist=0
bear=0
variation=0.000015 #Change in position beween each update (app 5 knots)
looptime=0 #keep track on how much time each loop takes in sec

#Start data

COG=1 # Default Course Over Ground 
SOG=2 # Default Speed Over Ground 
TWD=270 # True wind direction used for finding AWA
TWA=-90 # Angle between course and true wind
TWS=10 # True wind speed (m/s) used for AWS
AWA=1 # Apparent wind angle
AWD="R" #Apparent wind direction (left/right)
AWS=1 #Apparent wind speed
MAG=1 #Magnetic compass reading
AWAcos=None
SOGup=True #Increase speed
SOGmax=9 #Max speed
SOGmin=1 #Min speed
AWSup=True
AWSmax=25 #Max wind velocity knots
AWSmin=1 #Max wind velocity knots
depth=2.0
STW=1 # Speed through water

# WEA data
tempin=18.3
pressure=1003
humidity=78
tempout=22.3

#datenow= str(datetime.now())[2:4] + str(datetime.now())[5:7]  + str(datetime.now())[8:10]      
#print ("date " + datenow)
#timenow=str(datetime.now())[11:13] + str(datetime.now())[14:16] + str(datetime.now())[17:19]
#print ("time " + timenow)

def UDPsubmit(UDPstring):
    UDPbytes = str.encode(UDPstring+ "\n")
    udp = socket(AF_INET, SOCK_DGRAM)
    udp.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    udp.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    udp.sendto(UDPbytes, (UDPaddress, UDPport))
    #print ("UDP sent (" + str(UDPport) + ")" + str(UDPstring))

def true2apparent():
    global AWS,AWA,AWS,AWD,AWAcos,TWA
    # Finding apparent wind speed based on TWD, TWS, course and speed
    TWA=abs(COG-TWD)
    if TWA>180:
        if COG>TWD: 
            AWD="R"
        else:
            AWD="L" 
        TWA=360-TWA
    else:
        if COG>TWD: 
            AWD="L"
        else:
            AWD="R"
    TWA_rad=math.radians(TWA)
    TWD_rad=math.radians(TWD)
    if TWA_rad==0:
        TWA_rad=0.1 # Avoid COS zero 
    AWS=int(math.sqrt(math.pow(TWS,2)+math.pow(SOG,2)+2*TWS*SOG*math.cos(TWA_rad)))
    if AWS<=0:
        AWS=0.1 
        print("Avoiding zero division")
    # Finding apparent wind angle based on TWD, course and speed  
    AWAcos=((TWS*math.cos(TWA_rad)+SOG)/AWS)

#    AWAcos=AWS*math.cos((TWS*math.cos(TWA_rad)+SOG)/AWS)
    if AWAcos>1:
        AWAcos=AWAcos-int(AWAcos)
    if AWAcos<-1:
        AWAcos=AWAcos+abs(int(AWAcos))
    AWArad=math.acos(AWAcos)
    AWA=int(math.degrees(AWArad))
    #AWA=math.degrees(math.acos(AWS*math.cos((TWS*math.cos(TWD_rad)+SOG)/AWS)))
    
def WPTcourse(lat1,lon1,lat2,lon2):
    # calculate course to selected waypoint from current position

    lat1rad=math.radians(float(lat1)) # convert to radians
    lon1rad=math.radians(float(lon1)) # convert to radians
    lat2rad=math.radians(float(lat2)) # convert to radians
    lon2rad=math.radians(float(lon2)) # convert to radians

    y = math.sin(lon2rad-lon1rad) * math.cos(lat2rad)
    x = math.cos(lat1rad)*math.sin(lat2rad) - math.sin(lat1rad)*math.cos(lat2rad)*math.cos(lon2rad-lon1rad)
    cors = math.atan2(y, x)

    course=math.degrees(cors)
    course=(course+360) % 360
    course=str(course)
    course=course[0:4]
    return(course)

def dec2dmm(angle):
    # convert degree decimal positions to DegreesMinutes.Minutes
    posangle=float(angle) #converts to number format
    posangle=abs(posangle) #converts to positive (if negative) format
    degint=math.floor(posangle)
    degree=str(degint).zfill(2)
    minutesdec=posangle-float(degree)
    minutes=minutesdec*600000000000
    return(str(degree) + str(minutes)[:2] + "." + str(minutes)[2:6])

def newpos():
    global latprev,lat,lonprev,lon
    # Find the new position on basis of last position, course and speed
    COGrad=math.radians(float(COG))
    lat=latprev+math.cos(COGrad)*distance/60
    lon=lonprev+math.sin(COGrad)*distance/60
    print("New position " + str(lat) + "," +str(lon))

while True:
    datenow=str(datetime.now())[2:4] + str(datetime.now())[5:7]  + str(datetime.now())[8:10]      
    timenow=str(datetime.now())[11:13] + str(datetime.now())[14:16] + str(datetime.now())[17:19]
    #lat=lat+variation
    latdms=dec2dmm(lat)
    #lon=lon+variation
    londms=dec2dmm(lon)
    #dist=WPTdistance(lat,lon,latprev,lonprev)
    distance=SOG*looptime/3600
#    print("Distance since last update: " + str(distance))
    print("Input values: ")
    print("SOG: " + str(SOG))
    print("COG: " + str(COG))
    print("TWS: " + str(TWS))
    print("TWD: " + str(TWD))
    print("Calculated values: ")
    print("TWA: " + str(TWA))
    print("AWAcos: " + str(AWAcos))
    print("AWA: " + str(AWA))
    print("AWD: " + str(AWD))
    print("AWS: " + str(AWS))
    
    newpos()
    looptime=0

    # Send GPS position
    latprev=lat
    lonprev=lon
#    GPS="$GPRMC,"+timenow+".00"+",A," + str(latdms) +",N," + str(londms)+ ",E,"+str('{:3.1f}'.format(SOG))+","+str('{:3.1f}'.format(COG))+","+datenow+",,,A*73"
    GPScheckString="GPRMC,"+timenow+".00"+",A," + str(latdms)+"0" +",N,"+"0" + str(londms)+"0"+ ",E,"+str('{:3.1f}'.format(SOG))+","+str('{:3.1f}'.format(COG))+","+"100722"+",,,A"

    i=0
    checksum = 0
    while i < len(GPScheckString):
        checksum = checksum ^ ord(GPScheckString[i])
        i+=1
    print(hex(checksum))
    GPS="$" + GPScheckString + "*" + str(hex(checksum))[2:4]
    print(GPS)
    #GPS="$GPRMC,"+timenow+".00"+",A," + str(latdms)+"0" +",N,"+"0" + str(londms)+"0"+ ",E,"+str('{:3.1f}'.format(SOG))+","+str('{:3.1f}'.format(COG))+","+"100722"+",,,A*6C"
#    GPS="$GPRMC,131023.00,A,5645.28810,N,01131.51710,E,6.0,41.4,100722,,,A*6C" # test string with correct checksum
    UDPsubmit(GPS)

    # Speed adjustments before next GPS update
    if SOGup==True:
        SOG+=0.01
    else:
        SOG-=0.01
    if SOG>=SOGmax:
        SOGup=False
    if SOG<=SOGmin:
        SOGup=True

    # Course adjustments before next GPS update (set for a 360 deg turn)
    COG=COG+0.1
    if COG>360:
        COG=0
    time.sleep(submitperiod)
    looptime=looptime+submitperiod

    # Send Wind data
    true2apparent()
    VWR="$IIVWR," + str(abs(AWA)) + "," + str(AWD) + "," + str(AWS) + ",N," + str(TWD) + "," + str(TWS) + ",," #Relative Wind Speed and Angle
    UDPsubmit(VWR)
    
    # Wind speed adjustments before next update
    if AWSup==True:
        AWS=AWS+0.1
    else:
        AWS=AWS-0.1
    if AWS>=AWSmax:
        AWSup=False
    if AWS<=AWSmin:
        AWSup=True
    
    # Wind angle adjustments before next update
    AWA=AWA+1
    if AWA>=180:
        AWA=-AWA # Apparent wind angle
    if AWA>=0:    
        AWD="R" #Apparent wind direction (left/right)
    else:
        AWD="L"
    AWS=AWS+0.1 #Apparent wind speed

    time.sleep(submitperiod)
    looptime=looptime+submitperiod
    
    # Send depth info
    DBT="$IIDBT,0076.5,f,"+str(depth)+",M,," # Depth Below Transducer
    UDPsubmit(DBT)
    depth=depth+0.1
    time.sleep(submitperiod)
    looptime=looptime+submitperiod

    # send other instrument data
    STW=abs(SOG-random.randint(0, 3))
    VHW="$IIVHW,,,"+str(MAG)+",M,"+str(STW)+",N,," #  Heading and Water speed
    UDPsubmit(VHW)    
    time.sleep(submitperiod)
    looptime=looptime+submitperiod
    
    UDPsubmit(MTW)
    time.sleep(submitperiod)
    looptime=looptime+submitperiod

    WEA = "$GPWEA," + str(tempin) + "," + str(tempout) + "," + str(pressure) + "," + str(humidity) + "," + "44" + ","
    UDPsubmit(WEA)
    time.sleep(submitperiod)
    looptime=looptime+submitperiod

    UDPsubmit(HDM) # Auto pilot switched on
    UDPsubmit(HSC) # Auto pilot in auto mode
    
    print("looptime= " +str(looptime))

    VOL="$IIVOL," + str(VOL1) + ","  + str(VOL2) + ","  + str(VOL3) + "," + ",V" + "," #Analogue readings of voltage

    UDPsubmit(VOL)
    time.sleep(submitperiod)
    VOL1+=0.1
    VOL2+=0.2
    VOL3+=0.3
    if MAG > 360:
        MAG=0
    MAG+=1
   

'''
Test sensors:
dmesg | grep tty

Sample string: $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68


           225446       Time of fix 22:54:46 UTC
           A            Navigation receiver warning A = OK, V = warning
           4916.45,N    Latitude 49 deg. 16.45 min North
           12311.12,W   Longitude 123 deg. 11.12 min West
           000.5        Speed over ground, Knots
           054.7        Course Made Good, True
           191194       Date of fix  19 November 1994
           020.3,E      Magnetic variation 20.3 deg East
           *68          mandatory checksum
'''
