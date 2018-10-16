from tkinter import *
import time



#Inicializacao variaveis
kpX=0.0
kiX=0.0
kdX=0.0
kpY=0.0
kiY=0.0
kdY=0.0

# Funcao que termina o processo, destruindo a janela principal
def sair():
    print('Fechando a janela..')
    time.sleep(1) #Delay 1 segundo
    root.destroy()  # Destroi a janela

# Funcao que determina o valor de Kp, Ki e Kd do PID do eixo X
def set_PIDx():
    kpX=e1.get()
    kiX=e2.get()
    kdX=e3.get()
    print('Eixo X - {Kp: ' + str(kpX) +' Ki: ' + str(kiX)+ ' Kd: ' + str(kdX) +'} \n')

# Funcao que determina o valor de Kp, Ki e Kd do PID do eixo Y
def set_PIDy():
    kpY=e4.get()
    kiY=e5.get()
    kdY=e6.get()
    print('Eixo Y - {Kp: ' + str(kpY) + ' Ki: ' + str(kiY) + ' Kd: ' + str(kdY) + '} \n')


root = Tk()  # Cria a janela
root.title('Ball and Plate System')  # Define o titulo da janela
root.geometry('380x180')  # Define o tamanho da janela


# Cria os Labels
label = Label(root, text='PID - X axis')
label.place(x=70, y=20) # Posiciona o label
label = Label(root, text='Kp:')
label.place(x=10, y=50) # Posiciona o label
label = Label(root, text='Ki:')
label.place(x=10, y=75) # Posiciona o label
label = Label(root, text='Kd:')
label.place(x=10, y=100) # Posiciona o label
label = Label(root, text='PID - Y axis')
label.place(x=250, y=20) # Posiciona o label
label = Label(root, text='Kp:')
label.place(x=200, y=50) # Posiciona o label
label = Label(root, text='Ki:')
label.place(x=200, y=75) # Posiciona o label
label = Label(root, text='Kd:')
label.place(x=200, y=100) # Posiciona o label

#Parametros de entrada
e1=Entry(root)
e2=Entry(root)
e3=Entry(root)
e4=Entry(root)
e5=Entry(root)
e6=Entry(root)
e1.place(x=40, y=50)
e2.place(x=40, y=75)
e3.place(x=40, y=100)
e4.place(x=230, y=50)
e5.place(x=230, y=75)
e6.place(x=230, y=100)

#Cria botoes
b1 = Button(root, text ='Exit', command = sair) # Anexa a funcao Sair a evento de clique do botao
b1.place(x=120, y=130) # Posiciona o botao
b2 = Button(root, text ='Save', command = set_PIDx) # Anexa a funcao set_PIDx a evento de clique do botao
b2.place(x=80, y=130) # Posiciona o botao
b3 = Button(root, text ='Save', command = set_PIDy) # Anexa a funcao Sair a evento de clique do botao
b3.place(x=270, y=130) # Posiciona o botao

#Executa o programa
root.mainloop()