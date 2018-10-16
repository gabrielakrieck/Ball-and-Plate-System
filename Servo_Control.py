from tkinter import *
import serial
import time

#Calibracao dos motores
dif=20
Xi=100 #Posicao inicial do servo X
Yi=85 #Posicao inicial do servo Y

Ymax=Yi+dif #Valor maximo de Y
Ymin=Yi-dif #Valor minimo de Y
Xmax=Xi+dif #Valor maximo de X
Xmin=Xi-dif #Valor minimo de X

# Configuracao da porta serial

ser = serial.Serial("COM3", 9600);  # Define porta e velocidade de comunicacao
print
ser.portstr;  # Imprime a porta em uso
print('aguarde, incializando a porta...');
time.sleep(3);  # Aguarda 3 segundos


def y_axis(valor):  # Funcao que envia o valor do slide Y para a porta serial
    ser.write(bytes('y', 'UTF-8'))
    ser.write(bytes(chr(int(valor)), 'UTF-8'))  # Converte a variavel valor de str para int e de int para char ( byte )


def x_axis(valor):  # Funcao que envia o valor do slide X para a porta serial
    ser.write(bytes('x', 'UTF-8'))
    ser.write(bytes(chr(int(valor)), 'UTF-8'))  # Converte a variavel valor de str para int e de int para char ( byte )


def sair():  # Funcao que destroi a Janela principal, antes fecha a porta serial
    ser.close()  # Fecha a porta serial
    print('Fechando a porta serial...')
    time.sleep(1)
    print('Fechando a janela..')
    time.sleep(1)
    root.destroy()  # Destroi a janela



# processo de criacao da UI com TKinter

root = Tk()  # Cria a janela
root.title('Servo Slider Control')  # Define o titulo da janela
root.geometry('240x280')  # Define o tamanho

# Cria um objeto Scale que pode variar de Ymin a Ymax, e associa-o a funcao y_axis
scale1 = Scale( root, from_=Ymin, to=Ymax, command=y_axis, width=15, length=179)
scale1.set(Yi) #Define condicao inicial do slider
scale1.place(x=50,y=30) # Coloca o Scale na janela

# Cria um objeto Scale que pode varia de Xmin a Xmax, e associa-o a funcao x_axis
scale2 = Scale( root, from_=Xmin, to=Xmax, command=x_axis, width=15, length=179)
scale2.set(Xi) #Define condicao inicial do slider
scale2.place(x=150,y=30) # Coloca o Scale na janela


# Cria os Labels
label = Label(root, text='Ball and Plate System - Slide Servo Control')
label.place(x=10, y=5) # Posiciona o label
label = Label(root, text='Y_Spin')
label.place(x=65, y=210) # Posiciona o label
label = Label(root, text='X_Spin')
label.place(x=165, y=210) # Posiciona o label

# Cria Botao
b = Button(root, text ='Exit', command = sair) # Anexa a funcao Sair a evento de clique do botao
b.place(x=120, y=240) # Posiciona o botao

#Executa o programa
root.mainloop()