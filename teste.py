import argparse
import time
from collections import deque
import cv2
import imutils
import numpy as np
import serial
from imutils.video import VideoStream


class ArduinoSerial:
    # Construtor:
    def __init__(self, porta_serial, bound_value):
        self.arduino = serial.Serial(porta_serial, bound_value,)  # Define porta e velocidade de comunicacao
        print('Aguarde, inicializando a porta...')
        time.sleep(3);  # Aguarda 3 segundos

    def sair(self):  # Funcao que destroi a Janela principal, antes fecha a porta serial
        self.arduino.close()  # Fecha a porta serial
        print('Fechando a porta serial...')


class Servo:

    # Construtor:
    def __init__(self, ard, initial_value, eixo):
        self.dif = 25
        self.ard = ard #  atributo do tipo ArduinoSerial
        self.initial_value = initial_value # angulo inicial
        self.eixo = eixo # char indicando qual servo deve se movimentar
        self.max = initial_value+self.dif # valor maximo que o servo pode assumir
        self.min = initial_value-self.dif # valor minimo que o servo pode assumir
        self.move(initial_value) # posiciona o servo no angulo inicial



    # Metodos:
    # Metodo que determina o valor do angulo baseado nos valores de maximo e minimo
    def defAngle(self, value):
        if value <= self.min:
            value = self.min
        elif value >= self.max:
            value = self.max
        return value

    # Metodo que movimenta o servo para o angulo especificado dado o sinal de controle
    def control_move(self, PID_value):  # Funcao que envia o valor do slide X para a porta serial
        val=self.defAngle(PID_value+self.initial_value) # calcula o valor do angulo
        self.ard.arduino.write(bytes(self.eixo, 'UTF-8'))
        self.ard.arduino.write(bytes(chr(int(val)), 'UTF-8'))  # Converte a variavel val de str para int e de int para char (byte)

    # Metodo que movimenta o servo para o angulo especificado
    def move(self, angle):  # Funcao que envia o valor do slide X para a porta serial
        val=self.defAngle(angle)
        self.ard.arduino.write(bytes(self.eixo, 'UTF-8'))
        self.ard.arduino.write(bytes(chr(int(val)), 'UTF-8'))  # Converte a variavel val de str para int e de int para char (byte)

class PID:
    # Construtor:
    def __init__(self, kp, ki, kd, servo, factor):
        self.servo = servo  # atributo do tipo Servo
        self.kp = kp  # valor do parametro kp
        self.ki = ki  # valor do parametro ki
        self.kd = kd  # kd
        self.factor = factor
        self.pid_i = 0
        self.time_previous = 0
        self.timenow = 0
        self.previous_error = 0

    # Metodos
    def control_code(self, current_pos, desired_pos):
        # Estimation error
        error = (current_pos-desired_pos)/self.factor

        # PID parameters
        # Proportional
        pid_p=self.kp*error

        # Integral
        if -8 < error < 8:
            self.pid_i = self.pid_i + (self.ki*error)
        else:
            self.pid_i=0

        # Derivative
        self.time_previous = self.timenow
        self.timenow = time.time()
        elapsedtime = self.timenow - self.time_previous

        pid_d= self.kd*((error - self.previous_error)/elapsedtime)

        # Calculo sinal de controle
        contr_signal = pid_p + self.pid_i + pid_d

        # Atualizacao do valor do erro
        self.previous_error = error

        # Retorna o valor so sinal de controle
        return contr_signal

class Vision:
    # Construtor:
    def __init__(self, camera):
        self.posx = 0
        self.posy = 0

        # construct the argument parse and parse the arguments
        self.ap = argparse.ArgumentParser()
        self.ap.add_argument("-v", "--video",
                        help="path to the (optional) video file")
        self.ap.add_argument("-b", "--buffer", type=int, default=64,
                        help="max buffer size")
        self.args = vars(self.ap.parse_args())

        # define the lower and upper boundaries of the "green"
        # ball in the HSV color space, then initialize the
        # list of tracked points
        self.greenLower = (20, 50, 50)
        self.greenUpper = (150, 255, 255)

        # define camera
        self.camera = camera  # it could be 0, 1 in this case

        # other form
        self.gL = np.array([100, 50, 50])
        self.gU = np.array([150, 255, 255])

        self.pts = deque(maxlen=self.args["buffer"])

        self.key = cv2.waitKey(1) & 0xFF

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not self.args.get("video", False):
            self.vs = VideoStream(src=camera).start()

        # otherwise, grab a reference to the video file
        else:
            self.vs = cv2.VideoCapture(self.args["video"])

        self.frame = self.vs.read()


    # Metodo que captura e corta o frame
    def Frame_Capture(self):
        # grab the current frame
        self.frame = self.vs.read()
        self.frame = cv2.flip(self.frame, -2)
        # create a cropped frame
        self.crop_frame = self.frame[0:465, 80:560]  # Crop from {x, y, w, h } => {0, 0, 300, 400}
        # cv2.imshow("cropped", crop_frame)
        # handle the frame from VideoCapture or VideoStream
        self.frame = self.crop_frame[1] if self.args.get("video", False) else self.crop_frame

    def getBallPosition(self):
        # resize the frame, blur it, and convert it to the HSV
        # color space
        self.frame = imutils.resize(self.frame, width=300)
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
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

           # X_string = str(int(M["m10"] / M["m00"]))
           # Y_string = str(int(M["m01"] / M["m00"]))
           # previous_errorY_str = str(int(previous_error_y))
           # previous_errorX_str = str(int(previous_error_x))

           # print(
           #    "posicao x:" + X_string + " " + "posicao y:" + Y_string + " " + "erroY: " + previous_errorY_str + " " + "erroX: " + previous_errorX_str)

           # mask2 = cv2.inRange(hsv, gL, gU)

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)

        # update the points queue
        self.pts.appendleft(center)

        # loop over the set of tracked points
        for i in range(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            # them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(self.args["buffer"] / float(i + 1)) * 2.5)
            cv2.line(self.frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        # show the frame to our screen
        cv2.imshow("Frame", self.frame)
        #self.key = cv2.waitKey(1) & 0xFF
        self.posx=ballX
        self.posy=ballY

    def getPosx(self):
        return self.posx

    def getPosy(self):
        return self.posy

# Inicio do programa

# Inicializacoes
setpoint = [150, 150]

arduino = ArduinoSerial("COM4", 115200) # Inicializacao da interface serial do arduino
servo_x = Servo(arduino, 100, 'x') # Inicializacao do servo x
servo_y = Servo(arduino, 85, 'y') # Inicializacao do servo y
pid_x = PID(0.05, 0, 0.05, servo_x, 1.8) # Inicializacao do controlador do eixo X
pid_y = PID(0.05, 0, 0.05, servo_y, 1.8) # Inicializacao do controlador do eixo Y
camera = Vision(1) # Inicializacao das configuracoes de visao


# keep looping
while True:
    camera.Frame_Capture # captura o valor do frame

    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if camera.frame is None:
        break


    camera.getBallPosition # atualiza os valores de posicao da bola
    posx = camera.getPosx()
    posy = camera.getPosy()
    setpointx = setpoint[0]
    setpointy = setpoint[1]

    contrX = pid_x.control_code(posx, setpointx) # recebe o sinal de controle do eixo X
    contrY = pid_y.control_code(posy, setpointy) # recebe o sinal de controle do eixo Y
    servo_x.control_move(contrX) # servo x atua no sistema
    servo_y.control_move(contrY) # servo y atua no sistema
    time.sleep(0.02);

    # press q to end loop
    # if the 'q' key is pressed, stop the loop
    #if np.key == ord("q"):
    #    break

# if we are not using a video file, stop the camera video stream
if not camera.args.get("video", False):
    camera.vs.stop()

# otherwise, release the camera
else:
    camera.vs.release()

# close all windows
cv2.destroyAllWindows()




