char cmd;                     //Define a variável dos comandos seriais

void setup() {
  Serial.begin(9600);         //Inicia o Monitor Serial
  pinMode(14, OUTPUT);         //Define o pino como saída
  pinMode(15, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(18, OUTPUT);         //Define o pino como saída
  pinMode(19, OUTPUT);
  digitalWrite(14, 0);
  digitalWrite(15, 0);
  digitalWrite(2, 0);
  digitalWrite(4, 0);
  digitalWrite(18, 0);
  digitalWrite(19, 0);
}

void loop() {
  cmd = Serial.read();        //Realiza a leitura do serial
  if (cmd == 'w') {           //Se o comando for "l", liga o led
    analogWrite(14, 0);
    analogWrite(15, 0);
    analogWrite(2, 0);
    analogWrite(4, 64);
    analogWrite(18, 64);
    analogWrite(19, 0);
  }

  else if (cmd == 'd') {      //Se o comando for "d", desliga o led
    analogWrite(14, 0);
    analogWrite(15, 32);
    analogWrite(2, 0);
    analogWrite(4, 64);
    analogWrite(18, 0);
    analogWrite(19, 64);
  }

  else if (cmd == 'a') {      //Se o comando for "d", desliga o led
    analogWrite(14, 32);
    analogWrite(15, 0);
    analogWrite(2, 0);
    analogWrite(4, 64);
    analogWrite(18, 0);
    analogWrite(19, 64);
  }

  else if (cmd == 'p') {      //Se o comando for "d", desliga o led
    analogWrite(14, 0);
    analogWrite(15, 0);
    analogWrite(2, 0);
    analogWrite(4, 0);
    analogWrite(18, 0);
    analogWrite(19, 0);
  }
}
