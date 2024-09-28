char cmd;                     //Define a variável dos comandos seriais

void setup() {
  Serial.begin(9600);         //Inicia o Monitor Serial
  pinMode(15, OUTPUT);         //Define o pino como saída
  pinMode(14, OUTPUT);         //Define o pino como saída
  digitalWrite(14, LOW);
  digitalWrite(15, LOW);
}

void loop() {
  cmd = Serial.read();        //Realiza a leitura do serial
  if (cmd == 'w') {           //Se o comando for "l", liga o led
    analogWrite(14, 0);
    analogWrite(15, 0);
  }

  else if (cmd == 'd') {      //Se o comando for "d", desliga o led
    analogWrite(14, 0);
    analogWrite(15, 128);
  }

  else if (cmd == 'a') {      //Se o comando for "d", desliga o led
    analogWrite(14, 128);
    analogWrite(15, LOW);
  }
}
