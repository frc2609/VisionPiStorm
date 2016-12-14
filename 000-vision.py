# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import os,sys,inspect,time#thread
from subprocess import call
#import numpy as np
import numpy
import Image
#import imutils
from collections import deque
import cv2
#from networktables import NetworkTable
import math
#currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
#parentdir = os.path.dirname(currentdir)
#sys.path.insert(0,parentdir) 
from PiStorms import PiStorms
psm = PiStorms()
#NetworkTable.setIPAddress("roborio-2609-frc.local")#Change the address to your own
#NetworkTable.setClientMode()
#NetworkTable.initialize()
#sd = NetworkTable.getTable("RaspberryPi")
memoryPts = 64
pts = deque(maxlen=memoryPts) #Number of points of memory
#pts = []
# initialize the camera and grab a reference to the raw camera capture
try:
    camera = PiCamera()
except:
     m = ["VisionCode", "Camera not enabled.", "Run raspi-config and enable camera"]
     psm.screen.askQuestion(m,["OK"])
     exit()
#Set the camera options
camera.vflip = False
camera.hflip = False
camera.resolution = (640, 480)
camera.framerate = 60
camera.awb_mode = 'off'
camera.awb_gains = (0.2, 0.2)
camera.brightness = 50
camera.exposure_mode = 'sports'
camera.exposure_mode = 'off'
#camera.color_effects = 'None'
camera.contrast = 0
camera.drc_strength = 'off'
camera.exposure_compensation = 0
camera.flash_mode = 'off'
camera.image_effect = 'none'
camera.iso = 400
camera.saturation = 0
camera.sharpness = 0
camera.video_denoise = False
camera.meter_mode = 'spot' #Retrieves or sets the metering mode of the camera.
camera.video_stabilization = False
#NetworkTable.setIPAddress("roborio-2609-frc.local")#Change the address to your own
#NetworkTable.setClientMode()
#NetworkTable.initialize()
#sd = NetworkTable.getTable("RaspberryPi")
camera.shutter_speed = 500 #Random value chosen

rawCapture = PiRGBArray(camera, size=(640, 480))
kernel = numpy.ones((5,5), numpy.uint8)
psm.screen.termPrintAt(8, " camera.exposure_speed".format(camera.exposure_speed))
# allow the camera to warmup
time.sleep(0.5)


loops = 0
#sd.putNumber('H_L',30)
#sd.putNumber('H_U',90)
#sd.putNumber('S_L',120)
#sd.putNumber('S_U',255)
#sd.putNumber('V_L',120)
#sd.putNumber('V_U',255)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    #timestart = time.time()
    image = frame.array
    #Read the sliders from the NetworkTable
    #try:
    #    H_L = sd.getNumber('H_L')
    #    H_U = sd.getNumber('H_U')
    #    S_L = sd.getNumber('S_L')
    #    S_U = sd.getNumber('S_U')
    #    V_L = sd.getNumber('V_L')
    #    V_U = sd.getNumber('V_U')
    #except KeyError:
    H_L = 30
    H_U = 90
    S_L = 120
    S_U = 255
    V_L = 120
    V_U = 255
    #print('Smartdashboard Connect: N/A')
    #Convert to HSV and filter for colour desired
    lower_hsv = numpy.array([H_L,S_L,V_L])
    upper_hsv = numpy.array([H_U,S_U,V_U])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image_hsv = cv2.inRange(hsv, lower_hsv, upper_hsv)
    # CV erode iterations 2 Border_constant
    image_erosion = cv2.erode(image_hsv, None, iterations=2)
    # CV dilate iterations 2 Border_constant
    image_dilation = cv2.dilate(image_erosion, None, iterations=2)
    # Find contours
    #ret,thresh = cv2.threshold(image_dilation,0,255,cv2.THRESH_BINARY)
    cnts = cv2.findContours(image_dilation.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # Find bounding box's
        #x,y,w,h = cv2.boundingRect(cnts)
        c = max(cnts, key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only if radius meets a min size
        if radius > 10:
            cv2.circle(image,(int(x),int(y)),int(radius),(0,255,255),2)
            cv2.circle(image,center,5,(0,255,255),-1)
    #update points in queue
        pts.appendleft(center)
    for i in xrange(1, len(pts)):
        if pts[i-1] is None or pts [i] is None:
            continue
        #otherwise compute thickness of line and connect it
        thickness = int((numpy.sqrt(memoryPts) / float(i+1)*2.5))
        cv2.line(image,pts[i-1],pts[i],(0,0,255),thickness)
        centerX = int(M["m10"] / M["m00"])
        centerY = int(M["m01"] / M["m00"])
        angleToTarget = math.atan((centerX-(640/2))/320.9103533214) #  angleToTarget returns angle to target in rads.
        #                                                    320.9103533214 is our focal length in pixels1
        #                                                    found out by width/(2*tan(FOV/2)) where FOV is in degrees
        #cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),3)
        #sd.putNumber('centerX', centerX)
        #sd.putNumber('angleToTarget', angleToTarget)
        #sd.putNumber('centerY', centerY)
        #sd.putString('center', str(center))   
    # show the frame and other images
    
    #cv2.drawContours(image, contours, -1, (0,0,255), 3)
    #cv2.imshow("Frame", image)
    try:
        psm.screen.fillImgArray(0, 0, 320, 240, image)
    except:
        psm.screen.termPrintAt(6, " Image not printable")
        time.sleep(.5)
#    try:
#        psm.screen.termPrintAt(7, " angleToTarget".format(angleToTarget))
#    except:
#        psm.screen.termPrintAt(7, " angleToTarget not found")
#        time.sleep(.5)
    #print(camera.exposure_speed)
    #cv2.imshow("image_erosion", image_erosion)
    #cv2.imshow("thresh", thresh)
    #cv2.imshow("image_hsv", image_hsv)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break
    # if the PiStorm screen is pressed, break from the loop
    if (psm.screen.checkButton(0,0,320,320)):
                    psm.screen.clearScreen()
                    psm.screen.termPrintAt(9,"Exiting to menu")
                    break 


    #sd.putNumber('piLoops', loops)
    loops+=1



cv2.destroyAllWindows()
