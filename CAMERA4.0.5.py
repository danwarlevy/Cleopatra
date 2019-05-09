import RPi.GPIO as GPIO
import picamera
import time
import os
import subprocess
import threading
import atexit
import datetime
import numpy as np
import shutil
import sys
import socket
import subprocess
import nmap
import shlex

buttonPin2 = 14
buttonPin1 = 15
programRunningLED = 24
recordingLED = 23
laserPin= 16

recordTime = 0
recordMaxTime = 1200 #20 min

recording = False
toggleRecording = False
restart = False
laserState = False
isMoving = False
isStreaming = False
toggleStreaming = False
isMoving = False
toggleConvert = False

enablePWR = False
enableIMU = False

destDir = "/media/usb/"

#runs if you manually stop the program
def exit_handler():
    GPIO.cleanup()
    if camera.recording:
        camera.close()
    subprocess.call(["sudo", "umount",destDir])


#infinite loop to check the state variables
def main():

    #loop forever
    while True:
        time.sleep(0.01)
        stateCheck()

# method to make the led blink to indicate something
# blinking green means usb not inserted
# double blinking green means camera not connected
# blink red means files are transfering
def blink(x,pin, blinkCondition = True):
    keepGoing = True
    while keepGoing:
        for i in range(0,x):
            pin.start(1)
            time.sleep(.10)
            pin.start(0)
            time.sleep(.40)
        time.sleep(1)
        if callable(blinkCondition):
            keepGoing = blinkCondition()

#handles when the top button is pressed
def buttonPress1(channel):
    print("buttonPress1")
    startPressTime = time.time()
    pressTime=0

    while((GPIO.input(buttonPin1)==GPIO.LOW)):
        pressTime = time.time()-startPressTime

    if pressTime<0.01:
        print ("not long enough\t"+str(pressTime))
    elif pressTime<4:
        print("pressed for \t"+str(pressTime))
	#print("buttonPress")
        setToggle(True)
        setRecording(not getRecording())
    else:
        print("convert pressed for \t"+str(pressTime))
        setToggleStreaming(True)
        setStreaming(not isStreaming)

#handles when the bottom buttonis pressed
def buttonPress2(channel):
    global toggleConvert
    print("buttonPress2")
    startPressTime = time.time()

    while((GPIO.input(buttonPin2)==GPIO.LOW)):
    	pass

    pressTime = time.time()-startPressTime

    if pressTime<0.01:
        print ("not long enough\t"+str(pressTime))
    elif pressTime<4:
        print("pressed for \t"+str(pressTime))
	#print("buttonPress")
        setLaserState(not laserState)
    else:
        toggleConvert = True


#scans for a streaming reciever
# connects to one of the connected devices
def ipScan():
    np = nmap.PortScanner()
    iplist = np.scan(hosts='192.168.4.*',arguments='-sP')['scan'].keys()
    for item in iplist:
        if item != '192.168.4.1':
            return item
    return ''

#indicates streaming status
def streamingEnabled(en):
    if en:
        programRunningPWM.ChangeDutyCycle(0)
        time.sleep(.25)
        programRunningPWM.ChangeDutyCycle(1)
        time.sleep(.25)
        programRunningPWM.ChangeDutyCycle(0)
        time.sleep(.25)
        programRunningPWM.ChangeDutyCycle(1)
        time.sleep(.25)
    else:
        programRunningPWM.ChangeDutyCycle(0)
        time.sleep(.25)
        programRunningPWM.ChangeDutyCycle(1)
        time.sleep(.25)


#handles the video stream
def streamVideo():

    #look for ip address
    ip = ipScan()

    if ip == '':
        streamingEnabled(False)
        return
    else:
        streamingEnabled(True)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.connect((ip,8000))
    #client_socket.listen((0))

    # Make a file-like object out of the connection
    connection = client_socket.makefile('wb')


    time.sleep(2)
    # Start recording,
    try:
    	camera.start_recording(connection, format='h264',splitter_port=2,bitrate = 1000000)
    	print("started streaming")


    	while(isStreaming):
        	#camera.wait_recording(splitter_port=2)
                pass
    	camera.stop_recording(splitter_port = 2)
    finally:
        # connection.close()
        # client_socket.close()
        pass
        streamingEnabled(False)

#handles recording
def recordingThread(startTimeStamp):


    recordingPWM.ChangeDutyCycle(80)
    restartCount = 1
    finishingTime=startTimeStamp+toSec(restartCount*recordMaxTime)

    timeStamp = startTimeStamp.strftime("%m_%d_%y_%H_%M_%S")
    camera.start_recording('Cleopatra'+timeStamp+'.h264', splitter_port=1)
    print("am recording")

    while(getRecording()):
        camera.wait_recording(0.01)
        if(finishedWaiting(finishingTime)):
            timeStamp = finishingTime.strftime("%m_%d_%y_%H_%M_%S")

            camera.split_recording('Cleopatra'+timeStamp+'.h264',splitter_port = 1)
            restartCount+=1
            finishingTime=startTimeStamp+toSec(restartCount*recordMaxTime)


    #stop recording
    camera.stop_recording(splitter_port=1)
    #turn off led
    recordingPWM.ChangeDutyCycle(0)




def stateCheck():
    global toggleConvert
    if getToggle(): #if button was just pressed
        setToggle(False)

        #start recording
        if getRecording() and not isMoving: #if supposed to be recording
            print("toggle and rec")
            mytime = datetime.datetime.now()
            timeStamp = mytime.strftime("%m_%d_%y_%H_%M_%S")

            myRecord = threading.Thread(name='recordingThread', target=recordingThread, args = (mytime,))
            myRecord.start()
            if enableIMU:
                myotherRecord = threading.Thread(name='imuDataThread', target=imuDataThread, args = (mytime,))
                myotherRecord.start()
            if enablePWR:
                powerData = threading.Thread(name='inaDataThread', target=inaDataThread, args = (mytime,))
                powerData.start()


            # get time stamp and start recording

        else: #if not supposed to be recording
            pass

    if toggleStreaming:
        setToggleStreaming(False)
        if isStreaming: # start streaming

            streamThread = threading.Thread(name='streamThread', target=streamVideo)
            streamThread.start()
        else:
            pass

    elif getRecording():#if button not just pressed but supposed to be recording

        #check for exceptions
        #camera.wait_recording()
        pass



    else:
        pass

    if laserState:
        GPIO.output(laserPin,GPIO.HIGH)
    else:
        GPIO.output(laserPin,GPIO.LOW)

    if toggleConvert:
        print("calling method")
        convertsFiles()
        toggleConvert = False


def setToggle(set):
    global toggleRecording
    toggleRecording = set

def getToggle():
    global toggleRecording
    return toggleRecording

def setRecording(set):
    global recording
    recording = set

def getRecording():
    global recording
    return recording

def setRecordTime(set):
    global recordTime
    recordTime = set

def getRecordTime():
    global recordTime
    return recordTime

def setLaserState(set):
    global laserState
    laserState = set

def getRecordTime():
    return laserState


def setMoving(set):
    global isMoving
    isMoving = set

def setConvert(set):
    global toggleConvert
    toggleConvert = set

def getMoving():
    return isMoving

def setStreaming(set):
    global isStreaming
    isStreaming = set

def setToggleStreaming(set):
    global toggleStreaming
    toggleStreaming = set

#check if recorded longer than the set recorging length time
#returns true if exceeded max record time
def checkRecordTime():
    global recordTime
    return (time.time()-recordTime)>recordMaxTime

#returns true if past input time
def finishedWaiting(time):
    #print(startTime + toSec(time) + toSec(restartRecordingCount * acquisitionLoopTime))
    return datetime.datetime.now() > time

def toSec(secs):
    return datetime.timedelta(seconds = secs)

#prints time since started recording ad time until new video
def timeElapsed():
	print("Time Elapsed :"+str(time.time()-(getRecordTime()))+" seconds")
	print("Time to go :"+str(recordMaxTime-time.time()+getRecordTime())+" seconds")

#converts the files for mp4
def convertsFiles():

    print("moving")

    if(recording):
        print("return")
        return


    setMoving(True)
    filesToMove = os.listdir(destDir)
    fileindex = 1
    fileCount=len(filesToMove)
    print(filesToMove)

    blinkThread = threading.Thread(name = "blinkThread", target = blink, args=(1,recordingPWM,), kwargs = {'blinkCondition':getMoving})
    blinkThread.start()

    for file in filesToMove: #there will only be 3 files to get
        if file.endswith(".h264"):
            print("converting file "+str(fileindex)+ " of "+str(fileCount))
            #shutil.move(destDir+file,destDir+file)
            command = shlex.split("sudo ffmpeg -framerate 30 -i {} -c copy {}.mp4".format(destDir+file,destDir+file[:-5]))
            print(command)
            from subprocess import CalledProcessError
            try:
                output = subprocess.call(command)
            except CalledProcessError as e:
                print(e.output)
                #fileindex +=1

    print("done")
    filesToMove = os.listdir(destDir)
    fileindex = 1
    for file in filesToMove:
        if file.endswith(".h264"):
            os.remove(destDir+file)
            print("removing file "+str(fileindex)+ " of "+str(fileCount))
            fileindex +=1

    GPIO.output(programRunningLED,GPIO.HIGH)
    setMoving(False)

#sets up the program
def init():


    bno = 0
    ina = 0


    #Set GPIO mode
    GPIO.setmode(GPIO.BCM)

    #Setup I/O pins
    GPIO.setup(buttonPin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(buttonPin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(programRunningLED, GPIO.OUT)
    GPIO.setup(recordingLED, GPIO.OUT)
    GPIO.setup(laserPin, GPIO.OUT, initial=GPIO.LOW)

    #setup PWM
    frequency = 5000 #5000 Hz
    recordingPWM = GPIO.PWM(recordingLED,frequency)
    programRunningPWM = GPIO.PWM(programRunningLED,frequency)
    recordingPWM.start(0)#80% duty cycle
    programRunningPWM.start(0.01)

    #Prepare for interrupts on button pin
    GPIO.add_event_detect(buttonPin1,GPIO.FALLING,callback=buttonPress1,bouncetime=500) #TODO: Test bounce time
    GPIO.add_event_detect(buttonPin2,GPIO.FALLING,callback=buttonPress2,bouncetime=500)
    print('button set')

    try:
        camera = picamera.PiCamera()
        camera.resolution = (1280, 720)
        camera.framerate = 30
    except picamera.PiCameraError:
        blink(2,programRunningPWM)

    x = subprocess.call(["sudo", "mount","/dev/sda1",destDir])
    if(x!=0):
        blink(1,programRunningPWM)

    timeStamp = datetime.datetime.now()
    print(timeStamp)

    print("starting")
    os.chdir(destDir)



    return camera, recordingPWM, programRunningPWM

camera, recordingPWM, programRunningPWM = init()
atexit.register(exit_handler)

main()
