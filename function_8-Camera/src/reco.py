import jetson.inference
import jetson.utils
import RPi.GPIO as GPIO
import time
import threading
import os



beep_on = False



#Send PWM to the buzzer
def beep(pwm):
    global beep_on
    etat_beep = False
    while (True):
        if ((beep_on != etat_beep) and beep_on):
            pwm.start(50)
            etat_beep = True
        elif (beep_on != etat_beep and (not beep_on)):
            pwm.stop()
            etat_beep = False
    beep_on = False


def main():
    global beep_on
    # load the object detection model
    net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.7)

    camera = jetson.utils.videoSource("v4l2:///dev/video0")      # '/dev/video0' for V4L2

    display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file

    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(33, GPIO.OUT)
    pwm = GPIO.PWM(33, 2300)

    x = threading.Thread(target=beep, args=(pwm,), daemon=True)
    x.start()

    counter_person = 0

    while display.IsStreaming():
        person_detected = False
        img = camera.Capture()
        detections = net.Detect(img)
        for detection in detections:
            if (detection.ClassID == 1):
                counter_person += 1
                person_detected = True
        print(counter_person)        
        if (not person_detected):
            counter_person = 0
        if (counter_person > 2):
            beep_on = True
            counter_person = 3
        else:
            beep_on = False

        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

if __name__ == '__main__':
    main()


