import RPi.GPIO as GPIO
import time
import cv2
import imutils
from imutils.video import VideoStream
from time import sleep


#GPIO Setup

GPIO.setwarnings(False)
    #Ultrasonic sensor
GPIO_TRIGGER = 18
GPIO_ECHO = 24
    #Wheels
L_IN1 = 25 #IN1 pin 22
L_IN2 = 22 #IN2 pin 15
R_IN1 = 5 #IN3 pin 29
R_IN2 = 8 #IN4 pin 24
    #catcher
    #shooter
    #servoMotor
GPIO_SERVO_catcher = 21
GPIO_SERVO_shooter = 26
    #GateUltrasonic
GPIO_TRIG2 = 2
GPIO_ECHO2 = 3

def gpio_setup():
    #set pin access mode
    GPIO.setmode(GPIO.BCM)
    #set up ultrasonic
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)
    #gateUltrasonic
    GPIO.setup(GPIO_TRIG2, GPIO.OUT)
    GPIO.setup(GPIO_ECHO2, GPIO.IN)
    #set up driver motor
    GPIO.setup(L_IN1, GPIO.OUT)
    GPIO.setup(L_IN2, GPIO.OUT)
    GPIO.setup(R_IN1, GPIO.OUT)
    GPIO.setup(R_IN2, GPIO.OUT)
    #servo motor
    GPIO.setup(GPIO_SERVO_shooter ,GPIO.OUT)
    GPIO.setup(GPIO_SERVO_catcher ,GPIO.OUT)
#sample
# input ==> GPIO.input(GPIO_ECHO)
# output ==> GPIO.output(GPIO_TRIGGER, True/False)

def SetAngle(angle, pwm, GPIO_SERVO):
    duty = angle / 18 + 2
    GPIO.output(GPIO_SERVO, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(GPIO_SERVO, False)
    pwm.ChangeDutyCycle(0)

def shooter(degree):
    gpio_setup()
    pwm = GPIO.PWM(GPIO_SERVO_shooter, 50)
    pwm.start(0)
    SetAngle(degree, pwm, GPIO_SERVO_shooter)
    pwm.stop()
    GPIO.cleanup()
    
def catcher(degree):
    gpio_setup()
    pwm = GPIO.PWM(GPIO_SERVO_catcher, 50)
    pwm.start(0)
    SetAngle(degree, pwm, GPIO_SERVO_catcher)
    pwm.stop()
    GPIO.cleanup()

def detect_ball(frame):
    upper_hsv = (18, 235, 255)
    lower_hsv = (12, 123, 45)
    erode = 1
    dilate = 1
    output = None #out put is center of ball that detected by camera
    frame = imutils.resize(frame, width= 600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    mask = cv2.erode(mask, None, iterations = erode)
    mask = cv2.dilate(mask, None, iterations = dilate)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if len(cnts)> 0:
        c = max(cnts, key = cv2.contourArea) #select bigest contour
        ((x, y), radius) = cv2.minEnclosingCircle(c) #calculate minimum circle based on contour
        
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 10 and radius < 170:
            output = center
    return output

def detect_goal(frame):
    upper_hsv=  (62, 196, 157)
    lower_hsv=  (45, 113, 95)
    output=None
    frame=imutils.resize(frame,width=600)
    blurred=cv2.GaussianBlur(frame,(11,11),0)
    hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv, lower_hsv,upper_hsv)
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)
    contours=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=imutils.grab_contours(contours)
    if len(contours)>0:
        c=max(contours,key=cv2.contourArea)
        rect=cv2.minAreaRect(c)
        output=rect[0]
    return output

#distance function
def distance():
    gpio_setup()
    #send signal
    GPIO.output(GPIO_TRIGGER, True)
    #set Trigger to low after 0.00001s to LOW
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER, False)
    start_time = time.time()
    stop_time = time.time()
 

    #start_time
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    #stop_time
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    #check elapsed time
    elapsed_time = stop_time - start_time
    #check distance , (sonic speed (34300 cm/s)
    distance = (elapsed_time * 34300)/2
    
    print(distance)
    GPIO.cleanup()
    return distance

def distanceGate():
    gpio_setup()
    #send signal
    GPIO.output(GPIO_TRIG2, True)
    #set Trigger to low after 0.00001s to LOW
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIG2, False)
    start_time = time.time()
    stop_time = time.time()
 

    #start_time
    while GPIO.input(GPIO_ECHO2) == 0:
        start_time = time.time()

    #stop_time
    while GPIO.input(GPIO_ECHO2) == 1:
        stop_time = time.time()

    #check elapsed time
    elapsed_time = stop_time - start_time
    #check distance , (sonic speed (34300 cm/s)
    distance = (elapsed_time * 34300)/2
    
    print(distance)
    GPIO.cleanup()
    return distance

class Moving:
    
    @staticmethod
    def move_forward(power: int = 0.3):
        gpio_setup()
        GPIO.output(R_IN2, True)
        GPIO.output(L_IN2, True)
        time.sleep(1 * power)
        GPIO.output(R_IN2, False)
        GPIO.output(L_IN2, False)
        print("move forward")
        GPIO.cleanup()

    @staticmethod
    def happy(power: int = 0.3):
        Moving.turn_left()
        Moving.turn_right()
        Moving.turn_left(power=3)
        catcher(20)
        catcher(130)
        catcher(60)
        Moving.turn_left()
        Moving.turn_right()
        Moving.turn_right(power=3)
        catcher(20)
        catcher(130)
        catcher(60)
        print("done dancing")
        

    @staticmethod
    def move_backward(power: int = 0.3):
        gpio_setup()
        GPIO.output(R_IN1, True)
        GPIO.output(L_IN1, True)
        time.sleep(1 * power)
        GPIO.output(R_IN1, False)
        GPIO.output(L_IN1, False)
        print("done backward")
        GPIO.cleanup()

    

    @staticmethod
    def turn_right(power: int = 0.3):
        gpio_setup()
        GPIO.output(L_IN2, True)
        GPIO.output(R_IN1, True)
        time.sleep(1 * power)
        GPIO.output(L_IN2, False)
        GPIO.output(R_IN1, False)
        print("turn right")
        GPIO.cleanup()

    @staticmethod
    def turn_left(power: int = 0.3):
        gpio_setup()
        GPIO.output(R_IN2, True)
        GPIO.output(L_IN1, True)
        time.sleep(1 * power)
        GPIO.output(R_IN2, False)
        GPIO.output(L_IN1, False)
        print("turn left")
        GPIO.cleanup()
        
 


if __name__=="__main__":

    gpio_setup()
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
    isBallCatched = False
    isBallCenter = False
    #isBallCatched = False
    #GPIO.cleanup()
    while True:
        time.sleep(0.7)
        frame = vs.read()
        ballOutput = detect_ball(frame=frame)
        gateOutput = detect_goal(frame=frame)
        time.sleep(0.7)
    
        if gateOutput is not None and isBallCatched:
            if 200 <= gateOutput[0] <= 450:
                print("moving forward")
                Moving.move_forward()
                if distanceGate() < 50:
                    print("gate catched")
                    time.sleep(0.5)
                    catcher(130)
                    time.sleep(0.7)
                    shooter(50)
                    Moving.happy()
                    GPIO.cleanup()
                    exit()
            elif 200>= gateOutput[0]:
                Moving.turn_right()
            elif gateOutput[0] >= 450:
                Moving.turn_left()
        if isBallCenter:
             if distance() < 10:
                    print("ball catched")
                    isBallCatched = True
                    Moving.move_forward()
                    catcher(20)
                    isBallCenter = False
             else:
                 Moving.move_forward()
        
        elif ballOutput is not None and not isBallCatched:
            if 200 <= ballOutput[0] <= 450:
                print("moving forward")
                Moving.move_forward()
                isBallCenter = True
            elif 200 <= ballOutput[0]:
                print("turn_left")
                Moving.turn_left()
            else:
                print("turn_right")
                Moving.turn_right()

        elif ballOutput is None and not isBallCatched:
            print("search for ball")
            Moving.turn_right()

        elif isBallCatched and gateOutput is None:
            print("search for gate")
            Moving.turn_right()
        