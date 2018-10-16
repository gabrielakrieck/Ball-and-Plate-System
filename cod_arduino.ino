// Controle de dois servo motores usando Python e TK
 
#include <Servo.h>
 
Servo x_servo, y_servo; // Cria dois objetos do tipo Servo.
int eixo =0 , bytelido = 0; // Cria duas variaveis do tipo int.
 
void setup (){
  Serial.begin(9600); // Configura a velocidade da comunicação serial.
  x_servo.attach(9); // Anexa o objeto x_servo ao pino 9 do arduino.
  y_servo.attach(10); // Anexa o objeto y_servo ao pino 10 do arduino.
  x_servo.write(100); // posiciona x_servo no angulo zero grau.
  y_servo.write(85); // posiciona y_servo no angulo zero grau.
}
 
void loop(){
    while (Serial.available() > 0){ // Enquanto houver algum byte na entrada serial.
      eixo = Serial.read(); //Lê um byte e armazena na variável eixo.
      if ( eixo == (byte)'x' ) { // Verifica se eixo é igual ao caractere x.
        //delay(10); // Espera 10ms
        bytelido = Serial.read(); // Como eixo igual a 'x', bytelido recebe o valor do 'slider x' da UI em python.
        x_servo.write(bytelido); // Passa para o x_servo o valor de bytelido.
      }
      if ( eixo == (byte)'y' ) { // Verifica se eixo é igual ao caractere y.
        //delay(10); // Espera 10ms
        bytelido = Serial.read(); // Como eixo igual a 'y', bytelido recebe o valor do 'slider y' da UI em python.
        y_servo.write(bytelido); // Passa para o x_servo o valor de bytelido.
      }
    }
}
