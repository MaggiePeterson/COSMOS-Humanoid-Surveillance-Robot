import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO


# Initialise the PCA9685 PWM driver using the default address (0x40)
# and set the pwm parameters
pwm = Adafruit_PCA9685.PCA9685()
pwm_frequency = 50
pwm.set_pwm_freq(pwm_frequency)
servo_min = 0
servo_max = 600

# Function to calculate the servo pulse width (number between 0 and 4095)
def servoSetting(angle):
    return ((servo_max - servo_min) * angle//180 + servo_min)

LedPin1 = 11 # pin11
LedPin2 = 13 # pin13

def ledSetup():
        ''' One time set up configurations'''
        GPIO.setmode(GPIO.BOARD)                # Numbers GPIOs by physical location
                                                # An alternative is GPIO.BCM
        GPIO.setup(LedPin2, GPIO.OUT)            # Set LedPin's mode is output
        GPIO.output(LedPin2, GPIO.LOW)          # Set LedPin high (+3.3V) to turn off led
                                                        # An alternative is GPIO.BCM
        GPIO.setup(LedPin1, GPIO.OUT)            # Set LedPin's mode is output
        GPIO.output(LedPin1, GPIO.LOW)          # Set LedPin high (+3.3V) to turn off led

# CAMERA
# This is a basic program that illustrates a few ways
# to take pictures and store them directly to file

import picamera
import time
import picamera.array
import cv2
import numpy as np
import pygame
# This needs to be imported explicitly

pygame.init()
pygame.mixer.init()

# Initialize the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))
# Create a window for later use by the track bar
cv2.namedWindow('Mask')
cv2.namedWindow('newImage')
cv2.namedWindow('OtherMask')
cv2.namedWindow('3PartMask')

# This line is optional; if you need to rotate the image
camera.rotation = 180              

state = 0 

i = 0
def destroy():
        GPIO.output(LedPin1, GPIO.LOW)
        GPIO.output(LedPin2, GPIO.LOW) # led off
        GPIO.cleanup()                          # Release resource

if __name__ == '__main__':                      # Program starts here
        ledSetup()
        try:
            for frame in camera.capture_continuous(rawframe, format = "bgr", use_video_port = True):
            # Clear the stream in preparation for the next frame
                rawframe.truncate(0)
            ##    firstImageGray = firstImage.copy()
            ##    a = nextImage.copy()
                if  i == 0:
                    firstImage = frame.array
                    nextImage = frame.array
                    firstGray = cv2.cvtColor(nextImage, cv2.COLOR_BGR2GRAY)
                    firstGrayBlur = cv2.GaussianBlur(firstGray, (21, 21), 0)
                    i += 1
                else:
                    nextImage = frame.array
                    
                    
                nextImage = frame.array
                nextGray = cv2.cvtColor(nextImage, cv2.COLOR_BGR2GRAY)
                nextGrayBlur = cv2.GaussianBlur(nextGray, (21, 21), 0)
                #newImage = nextGrayBlur - firstGrayBlur
                newImage = cv2.absdiff(nextGrayBlur, firstGrayBlur)
                lowerThreshold = np.array(21)
                upperThreshold = np.array(255)
                bestImage = cv2.inRange(newImage, lowerThreshold, upperThreshold)
                
                w,h = bestImage.shape
                leftThird = bestImage[:,0:h//3]
                middleThird = bestImage[:, h//3:2*h//3]
                rightThird = bestImage[:, 2*h//3:h]
                
                numWhiteLeft = cv2.countNonZero(leftThird)
                numWhiteMiddle = cv2.countNonZero(middleThird)
                numWhiteRight = cv2.countNonZero(rightThird)
                
                #0 is nothing
                
                if numWhiteLeft > numWhiteRight and numWhiteLeft > numWhiteMiddle: #coming from left
                    state = 1
                    pwm.set_pwm(0, 0, servoSetting(180))

                    #moveLeft
                elif numWhiteMiddle > numWhiteLeft and numWhiteMiddle > numWhiteRight:
                    state = 0
                    pwm.set_pwm(0, 0, servoSetting(90))
                    GPIO.output(LedPin1, GPIO.HIGH)   # led on
                    GPIO.output(LedPin2, GPIO.HIGH)   # led on
                    #middle       
                elif numWhiteRight > numWhiteLeft and numWhiteRight > numWhiteMiddle:
                    state = 2
                    pwm.set_pwm(0, 0, servoSetting(1))
                    GPIO.output(LedPin1, GPIO.HIGH)   # led on
                    GPIO.output(LedPin2, GPIO.HIGH)   # led on
                else:
                    #lol
                    GPIO.output(LedPin2, GPIO.LOW) #led is off
                    GPIO.output(LedPin1, GPIO.LOW)   # led off

                    #turnright
                if numWhiteRight + numWhiteLeft + numWhiteMiddle > 100:
                    pygame.mixer.music.load('bigbrother.mp3')
                    pygame.mixer.music.play(loops=0, start=0.0)
                    
                cv2.imshow("Mask", nextGray)
                cv2.imshow("OtherMask", nextGrayBlur)
                cv2.imshow("newImage", bestImage)
                cv2.waitKey(1)
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                print("destroy")
                destroy()


# Stop the camera preview
camera.stop_preview()

# Clean up the camera resources
camera.close()

