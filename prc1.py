import cv2
import numpy as np
import math
import sys
import time
from imutils import contours
import imutils
import RPi.GPIO as GPIO

GPIO.setwarnings(False)

#throttle
throttlePin = 25 # Physical pin 22
in3 = 23 # physical Pin 16
in4 = 24 # physical Pin 18

#Steering
steeringPin = 22 # Physical Pin 15
in1 = 17 # Physical Pin 11
in2 = 27 # Physical Pin 13


GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

GPIO.setup(throttlePin,GPIO.OUT)
GPIO.setup(steeringPin,GPIO.OUT)

# Steering
# in1 = 1 and in2 = 0 -> Left
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
steering = GPIO.PWM(steeringPin,1000)
steering.stop()

# Throttle
# in3 = 1 and in4 = 0 -> Forward
GPIO.output(in3,GPIO.HIGH)
GPIO.output(in4,GPIO.LOW)
throttle = GPIO.PWM(throttlePin,1000)
throttle.stop()
def get_center(center1,center2):
                  
        x=int((center1[0]+center2[0])/2)
        y=int((center1[1]+center2[1])/2)
        return (x,y)

def make_center1(img_half,img):
        cnts = cv2.findContours(img_half.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        for c in cnts:
            M = cv2.moments(c)
            
            if M["m00"] !=0 :
                
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                                
                
            
                        
                # draw the contour and center of the shape on the image
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(img, "center", (cX - 20, cY - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                #cv2.imshow('img_c',img)
                
                return (cX,cY)
  
            
    
def make_co(image,para):
        slope,intercept=para
        y1=image.shape[0]
        y2=int(y1*0.5)
        x1=int((y1-intercept)/slope)
        x2=int((y2-intercept)/slope)
        return np.array([x1,y1,x2,y2])
        
    
def filter_color(img):
        ltw=np.array([200,200,200])
        utw=np.array([255,255,255])
        maskw=cv2.inRange(img,ltw,utw)
        wi=cv2.bitwise_and(img,img,mask=maskw)
        

        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lty=np.array([90,100,100])
        uty=np.array([110,255,255])
        masky=cv2.inRange(hsv,lty,uty)
        yi=cv2.bitwise_and(img,img,mask=masky)
       

        fi=cv2.addWeighted(wi,1.0,yi,1.0,0.0)
        
        return fi

   
def image_processing(img):

        img = filter_color(img)

        gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        blur=cv2.GaussianBlur(gray , (3,3) , 0)

        canny = cv2.Canny( blur , 50 , 150 )
        return canny


def region_of_interest(img,vertices):

        background = np.zeros_like(img)
        cv2.fillPoly(background,vertices,255)
       
        path=cv2.bitwise_and(img,background)
        
        return path

    

video = cv2.VideoCapture(0)
video_next = cv2.VideoCapture(0)

###change frame width and height acc to camera
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
video_next.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video_next.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
###

time.sleep(1)

speed = 8
lastTime = 0
lastError = 0

kp = 0.4
kd = kp * 0.65
i=0

while True:
    ret,frame = video.read()
    
    video.set(cv2.CAP_PROP_POS_FRAMES, i)
    _,img=cap.read()
    img2=img
    
    a=image_processing(img)
    
    width=img.shape[1]
    height=img.shape[0]
    
    v1 = np.array([[(width/8,height/8),(width/8,height/2),(width/2,height/2),(width/2,height/8)]],dtype=np.int32)
    v2 = np.array([[(width/2,height/8),(width/2,height/2),(width*(7/8),height/2),(width*(7/8),height/8)]],dtype=np.int32)    
    cropped_image1=region_of_interest(a,v1)
    cropped_image2=region_of_interest(a,v2)
    
    
    cv2.imshow('cr1',cropped_image1)
    cv2.imshow('cr2',cropped_image2)
    
    coords1=make_center1(cropped_image1,img)
    coords2=make_center1(cropped_image2,img)
    slope1 = (coords2[1]-coords1[1])/(coords2[0]-coords1[0])


    j=i+2
    video_next.set(cv2.CAP_PROP_POS_FRAMES, j)
    _,img3 = cap_next.read()


    b=image_processing(img3)

    c1=region_of_interest(b,v1)
    c2=region_of_interest(b,v2)
    
    coords3=make_center1(c1,b)
    coords4=make_center1(c2,b)
    slope2 = (coords4[1]-coords3[1])/(coords4[0]-coords3[0])

    angle = math.atan((slope2-slope1)/(1+(slope1*slope2)))
    angle_deg=180/3.14159*angle
    cv2.line(img, coords1, coords2, (0, 0, 255), 5)
    cv2.line(img, coords3, coords4, (0, 0, 255), 5)
    cv2.imshow('img_final',img)
    cv2.imshow('img3',img3)
    
    i=i+2

    
    """
    print("slope1")
    print(slope1)
    print("slope2")
    print(slope2)
    """
    
    print(angle_deg)
   
    

    now = time.time()
    dt = now - lastTime

    deviation = angle_deg 
    error = abs(deviation)
    
    if deviation < 5 and deviation > -5:
        deviation = 0
        error = 0
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        steering.stop()

    elif deviation > 5:
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        steering.start(100)
        

    elif deviation < -5:
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        steering.start(100)

    derivative = kd * (error - lastError) / dt
    proportional = kp * error
    PD = int(speed + derivative + proportional)
    spd = abs(PD)

    if spd > 25:
        spd = 25
        
    throttle.start(spd)

    lastError = error
    lastTime = time.time()
        

    key = cv2.waitKey(1)
    if key == 27:
        break
    
video.release()
video_next.release()


cv2.destroyAllWindows()
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
throttle.stop()
steering.stop()
