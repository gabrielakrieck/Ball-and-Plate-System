# import the necessary packages
import argparse
import time
from collections import deque
import cv2
import imutils
import numpy as np
import serial
from imutils.video import VideoStream

# Condicoes Iniciais de Controle
desired_X = 150 # Centre of the video
desired_Y = 150 # Centre of the video
kp = 0.05
ki = 0.0005
kd = 0.05
previous_error_x = 0
previous_error_y = 0
timenow = 0
pid_i_x = 0
pid_i_y = 0

#Calibracao dos motores
dif=20
Xi=105 #Posicao inicial do servo X
Yi=85 #Posicao inicial do servo Y

Ymax=Yi+dif #Valor maximo de Y
Ymin=Yi-dif #Valor minimo de Y
Xmax=Xi+dif #Valor maximo de X
Xmin=Xi-dif #Valor minimo de X

# Funcao que envia o valor da posicao Y para a porta serial
def y_axis(valor):
    if valor <= Ymin:
        valor = Ymin
        ser.write(bytes('y', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))
    elif valor >= Ymax:
        valor = Ymax
        ser.write(bytes('y', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))
    else:
        ser.write(bytes('y', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))

# Funcao que envia o valor da posicao X para a porta serial
def x_axis(valor):
    if valor <= Xmin:
        valor = Xmin
        ser.write(bytes('y', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))
    elif valor >= Xmax:
        valor = Xmax
        ser.write(bytes('y', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))
    else:
        ser.write(bytes('x', 'UTF-8'))
        ser.write(bytes(chr(int(valor)), 'UTF-8'))



# Configuracao da porta serial
ser = serial.Serial("COM3", 9600);  # Define porta e velocidade de comunicacao
print
ser.portstr;  # Imprime a porta em uso
print('Aguarde, incializando a porta...');
time.sleep(3);  # Aguarda 3 segundos




# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (20, 50, 50)
greenUpper = (150, 255, 255)

# define camera
camera = 1  # it could be 0, 1 in this case

# other form
gL = np.array([100,50,50])
gU = np.array([150,255,255])

pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=camera).start()

# otherwise, grab a reference to the video file
else:
    vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(1.0)

# keep looping
while True:
    # grab the current frame
    frame = vs.read()
    frame = cv2.flip(frame, -2)
    # create a cropped frame
    crop_frame = frame[0:465, 80:560]  # Crop from {x, y, w, h } => {0, 0, 300, 400}
    cv2.imshow("cropped", crop_frame)
    # handle the frame from VideoCapture or VideoStream

    frame = crop_frame[1] if args.get("video", False) else crop_frame

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=300)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        ballX = int(M["m10"] / M["m00"])
        ballY = int(M["m01"] / M["m00"])

        X_string = str(int(M["m10"] / M["m00"]))
        Y_string = str(int(M["m01"] / M["m00"]))
        previous_errorY_str = str(int(previous_error_y))
        previous_errorX_str = str(int(previous_error_x))

        print("posicao x:" + X_string + " " + "posicao y:" + Y_string + " " + "erroY: " + previous_errorY_str + " " + "erroX: " + previous_errorX_str)

        mask2 = cv2.inRange(hsv, gL, gU)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # ########################################################################################################
    # PID calculation

    error_Y = (ballY - desired_Y)/1.0
    error_X = (ballX - desired_X)/1.0

    # Proportional_XY
    pid_p_y = kp * error_Y
    pid_p_x = kp * error_X

    # Integral_XY
    if -8 < error_Y < 8:
        pid_i_y = pid_i_y + (ki * error_Y)

    if -8 < error_X < 8:
        pid_i_x = pid_i_x + (ki * error_X)
    # Derivative_XY
    time_previous = timenow
    timenow = time.time()
    elapsedTime = timenow - time_previous

    pid_d_y = kd * ((error_Y - previous_error_y)/elapsedTime)
    pid_d_x = kd * ((error_X - previous_error_x)/elapsedTime)

    previous_error_y = error_Y
    previous_error_x = error_X

    PID_Y = pid_p_y + pid_i_y + pid_d_y
    PID_X = -(pid_p_x + pid_i_x + pid_d_x)

    PID_X_str = str(PID_X)
    print("PID X: " + PID_X_str)

    servo_y = Yi + PID_Y
    servo_x = Xi + PID_X



    print ("servoy:" + str(servo_y) + " " + "servox: " + str(servo_x) + "PX: " + str(pid_p_x))  # servo_signal is a float so converting it to integer values

    #Aplica os valores calculados aos motores
    y_axis(servo_y)
    x_axis(servo_x)
    time.sleep(0.05);

    # press q to end loop

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()

# otherwise, release the camera
else:
    vs.release()

# close all windows
cv2.destroyAllWindows()