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
    beep_on = True
    pwm.start(50)
    time.sleep(0.5)
    pwm.stop()
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



    while display.IsStreaming():
        img = camera.Capture()
        detections = net.Detect(img)
        for detection in detections:
            if (detection.ClassID == 1):
                if (beep_on == False) :
                    x = threading.Thread(target=beep, args=(pwm,), daemon=True)
                    x.start()
        display.Render(img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

if __name__ == '__main__':
    main()


