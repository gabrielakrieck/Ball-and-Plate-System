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
        self.arduino = serial.Serial(porta_serial, bound_value, )  # Define porta e velocidade de comunicacao
        print('Aguarde, inicializando a porta...')
        time.sleep(2)  # Aguarda 3 segundos

    # Metodos
    # Metodo que fecha a porta serial
    def sair(self):
        self.arduino.close()  # Fecha a porta serial
        print('Fechando a porta serial...')

class Servo:
    # Construtor:
    def __init__(self, ard, initial_value, eixo):
        self.dif = 15
        self.ard = ard  # atributo do tipo ArduinoSerial
        self.initial_value = initial_value  # angulo inicial
        self.eixo = eixo  # char indicando qual servo deve se movimentar
        self.max = initial_value + self.dif  # valor maximo que o servo pode assumir
        self.min = initial_value - self.dif  # valor minimo que o servo pode assumir
        self.move(initial_value)  # posiciona o servo no angulo inicial

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
        val = self.defAngle(PID_value + self.initial_value)  # calcula o valor do angulo
        self.ard.arduino.write(bytes(self.eixo, 'UTF-8'))
        self.ard.arduino.write(bytes(chr(int(val)), 'UTF-8'))  # Converte a variavel val de str para int e de int para char (byte)

    # Metodo que movimenta o servo para o angulo especificado
    def move(self, angle):
        val = self.defAngle(angle)
        self.ard.arduino.write(bytes(self.eixo, 'UTF-8'))
        self.ard.arduino.write(
            bytes(chr(int(val)), 'UTF-8'))  # Converte a variavel val de str para int e de int para char (byte)


class PID:
    # Construtor:
    def __init__(self, kp, ki, kd, servo, factor, ki_max):
        self.servo = servo  # atributo do tipo Servo
        self.kp = kp  # valor do parametro kp
        self.ki = ki  # valor do parametro ki
        self.kd = kd  # valor do parametro kd
        self.factor = factor
        self.pid_i = 0
        self.previous_error = 0
        self.Integrator_max = ki_max
        self.Integrator_min = -1*ki_max
        self.error = 0
        self.contr_signal = 0

    # Metodos
    def control_code(self, current_pos, desired_pos, Ts):
        # Estimation error
        self.error = int(current_pos) - int(desired_pos)/self.factor

        # PID parameters
        # Proportional
        pid_p = self.kp * self.error

        # Integral
        self.pid_i = self.pid_i + self.error

        if self.pid_i > self.Integrator_max:
            self.pid_i = self.Integrator_max
        elif self.pid_i < self.Integrator_min:
            self.pid_i = self.Integrator_min

        # Derivative
        pid_d = self.kd * ((self.error - self.previous_error) / Ts)

        # Calculo sinal de controle
        self.contr_signal = pid_p + self.pid_i + pid_d

        # Atualizacao do valor do erro
        self.previous_error = self.error

        # Retorna o valor so sinal de controle
        return self.contr_signal

    def printInfo(self, eixo):
        print('[Controlador {} -- Erro {}; Sinal de controle {}]').format(eixo, self.error, self.contr_signal)



class Vision:
    # Construtor:
    def __init__(self, cam):
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
        self.greenLower = (0, 0, 0)
        self.greenUpper = (200, 105, 105)

        # define camera
        self.cam = cam  # it could be 0, 1 in this case

        self.pts = deque(maxlen=self.args["buffer"])

        self.key = cv2.waitKey(1) & 0xFF

        # if a video path was not supplied, grab the reference
        # to the webcam
        if not self.args.get("video", False):
            self.vs = VideoStream(src=cam).start()

        # otherwise, grab a reference to the video file
        else:
            self.vs = cv2.VideoCapture(self.args["video"])
            self.vs.set(3, 320)
            self.vs.set(4, 240)

        self.frame = self.vs.read()

    # Metodo que captura e corta o frame
    def Frame_Capture(self):
        # grab the current frame
        self.frame = self.vs.read()
        self.frame = cv2.flip(self.frame, -1)
        # create a cropped frame
        self.crop_frame = self.frame[10:480, 30:535]  # Crop from {x, y, w, h } => {0, 0, 300, 400}

        # handle the frame from VideoCapture or VideoStream
        self.frame = self.crop_frame[1] if self.args.get("video", False) else self.crop_frame

        # redimensiona a imagem, borra, transforma em HSV
        self.frame = imutils.resize(self.frame, width=300)
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # mascara de cor para dilatar e remover ruidos do objetos
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, None, iterations=4)

        cv2.imshow("masked_main", mask)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None

        # desenhar set points
        cv2.circle(self.frame, (33, 260), 20, (255, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (169, 260), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (225, 185), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (280, 255), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (280, 129), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (105, 85), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (25, 15), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (285, 90), 10, (0, 0, 0), thickness=5, lineType=8, shift=0)
        cv2.circle(self.frame, (275, 15), 20, (0, 0, 255), thickness=5, lineType=8, shift=0)

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
        self.key = cv2.waitKey(1) & 0xFF
        self.posx = self.pixel2cm(ballX)
        self.posy = self.pixel2cm(ballY)

    def getPosx(self):
        return self.posx

    def getPosy(self):
        return self.posy

    # Conversao de pixel para cm
    def pixel2cm(self, pos):
        return pos/100

class Array:
    # Construtor:
    def __init__(self):
        self.tamanho_max=1500

    def add(self, value):
        if(len(self)<self.tamanho_max):
            self.append(value) # armazena o valor na ultima posicao do array
        else:
            self.pop(0) # exclui elemento da posicao 0
            for i in range(1, 1, self.tamanho_max):
               self[i-1] = self[i] # transpoe os valores do array
        self.append(value) # armazena o valor na ultima posicao do array

    def toString(self):
        strg = ""
        for i in range(0, 1, len(self)-1):
            strg += str(self[i]) + ";"

# Metodos auxiliares
def aguardaTs(Ti, Tf, Ts):
    time.sleep(Ts-(Tf-Ti))
    return

def imprimeInfoEixo(eixo, pos, control):
    print('[Eixo {} -- Pos {}; Sinal de controle {}]').format(eixo, pos, control)
    return

def salvaArquivos(vetor, nome_arquivo):
    strg=nome_arquivo + ".txt"
    file = open("./arquivos/" + str(strg), 'w')
    file.write(vetor.toString())
    file.close
    return

# def armazenaDados(erroX, erroY, controleX, controleY, posX, posY):


# laço do percurso

centro = [150, 150]
setpoint_0 = [33, 272]
setpoint_1 = [169, 264]
setpoint_2 = [231, 188]
setpoint_3 = [280, 255]
setpoint_4 = [280, 129]
setpoint_5 = [105, 85]
setpoint_6 = [25, 15]
setpoint_7 = [285, 90]
setpoint_fim = [285, 15]

# INICIALIZACOES
Ts = 0.033 # Tempo de espera
# erroX = Array()
contrlX = Array()
posicaoX = Array()

# erroY = Array()
contrlY = Array()
posicaoY = Array()



arduino = ArduinoSerial("COM3", 115200)  # Inicializacao da interface serial do arduino
servo_x = Servo(arduino, 100, 'x')  # Inicializacao do servo x
servo_y = Servo(arduino, 85, 'y')  # Inicializacao do servo y

pid_x = PID(0.055, 0, 0.065, servo_x, 2, 5)  # Inicializacao do controlador do eixo X
pid_y = PID(0.055, 0, 0.065, servo_y, 2, 5)  # Inicializacao do controlador do eixo Y
cam = Vision(1)

# keep looping
while True:
    Ti = time.time()  # Determina o tempo inicial
    cam.Frame_Capture()  # Captura o frame

    # Se o frame nao for detectado, interrompe o loop
    if cam.frame is None:
        print("Frame nao detectado")
        break

    cam.getBallPosition # Atualiza os valores de posicao da bola
    posx = cam.getPosx() # Armazena o valor da posicao x da bola na variavel posx
    posy = cam.getPosy() # Armazena o valor da posicao x da bola na variavel posy

    contrX = pid_x.control_code(posx, centro[0]/100)  # recebe o sinal de controle do eixo X
    contrY = pid_y.control_code(posy, centro[1]/100)  # recebe o sinal de controle do eixo Y

    servo_x.control_move(contrX * (-1))  # servo x atua no sistema
    servo_y.control_move(contrY)  # servo y atua no sistema

    # Imprime informacoes do sistema na tela:
    imprimeInfoEixo('x', str(posx), str(contrX*(-1))) # eixo X
    imprimeInfoEixo('y', str(posy), str(contrY)) # eixo Y
    contrX.printInfo('x') # controlador X
    contrY.printInfo('y') # controlador Y

    # Armazena os valores das variaveis nos arrays:
    contrlX.add(contrX)
    contrlY.add(contrY)
    posicaoX.add(posx)
    posicaoY.add(posy)


    Tf = time.time() # Determina o tempo final
    aguardaTs(Ti,Tf,Ts) # Delay

    # press q to end loop
    #  if the 'q' key is pressed, stop the loop
    # if np.key == ord("q"):
    #    break

# if we are not using a video file, stop the camera video stream
if not cam.args.get("video", False):
    cam.vs.stop()

# otherwise, release the camera
else:
    cam.vs.release()

# close all windows
cv2.destroyAllWindows()