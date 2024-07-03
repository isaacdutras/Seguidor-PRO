#include <Arduino.h>
#include <QTRsensors.h>

#define GENERO "Binario"

#define VERSAO 1.0

/*
Planos da nova pcb:

botao: pino 23

PONTE H:

IN1: pino 4
IN2: pino 5    (Pinos pwm output)
IN3: pino 18  
IN4: pino 19
SLEEP: pino 22  (pino output)
FALL: pino 21 (pino input)

SENSOR PRINCIPAL:
D1: 13
D2: 14
D3: 27
D4: 26  (Dos pinos digitais pode substitutir se o substituto suportar Entrada de Dados e ser ADC)
D5: 25  
D6: 33
D7: 32
D8: 35
IR: 12 (pode ser qulquer outro pino output)(Esse pino nao pode ser usado como INPUT)

SENSORES LATERAIS:
Sensor direita: Pino 34
Sensor Esquerda: Pino 15
*/

// Definição Pinos da ponte H
const int IN1 = 2;    // Horario Motor Esquerdo //Trocar pelo 4
const int IN2 = 4;    // Antihorario Motor Esquerdo //Trocar pelo 5
const int IN3 = 5;    // Horario Motor Direito //Trocar pelo 18
const int IN4 = 18;   // Antihorario Motor Direito //Trocar pelo 19
const int SLEEP = 21; // Ativa ou desativa o output da ponte H //trocar pelo 22
const int FALL = 19;  // Detecta se tem algum erro //trocar pelo 21

#define OFFSET 0 // Diferenca de potencia entre os dois motores

void calibrar();
double calculoPid();
void controlePid();
void controleMotor(int vel_M1, int vel_M2);
void contagem_de_voltas();

const int BITS = 10;     // Define a resolucao do pwm para 10 Bits (2^10 ou 1024)
const int FREQ = 10000; // Define a frequencia do pwm
const int IN1_chanel = 0;
const int IN2_chanel = 1;
const int IN3_chanel = 2;
const int IN4_chanel = 3;

const int velMax_M1 = 1023 + OFFSET;
const int velMax_M2 = 1023;

const int velMin_M1 = 930 + OFFSET;
const int velMin_M2 = 930;

const int setpoint = 3500;

const uint8_t NUM_SENSORES = 8;
uint16_t valorSensores[NUM_SENSORES];

const int BOTAO_CONTROLE = 23;
const int SENSOR_CURVA = 15;
const int SENSOR_PARADA = 34;
unsigned long int tempo;

int ultimo_val_sensor = 0;

double kp_c = 2, ki_c = 0, kd_c = 0;
double kp = 0.6, ki = 0, kd = 0.4;

bool curva = 0;

int n_voltas = 1; // Variável para definir o número de voltas na pista
int cont_voltas = 0;

uint16_t val_sensor;

bool Linha_preta = true;

QTRSensors qtr; // Objeto qtr

void setup()
{
  Serial.begin(115200);

  pinMode(SLEEP, OUTPUT);
  pinMode(FALL, INPUT_PULLUP);
  pinMode(SENSOR_CURVA, INPUT);
  pinMode(SENSOR_PARADA, INPUT);
  pinMode(BOTAO_CONTROLE, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttachPin(IN1, IN1_chanel);
  ledcAttachPin(IN2, IN2_chanel);
  ledcAttachPin(IN3, IN3_chanel);
  ledcAttachPin(IN4, IN4_chanel);

  ledcSetup(IN1_chanel, FREQ, BITS);
  ledcSetup(IN2_chanel, FREQ, BITS);
  ledcSetup(IN3_chanel, FREQ, BITS);
  ledcSetup(IN4_chanel, FREQ, BITS);

  controleMotor(0,0);

}

void loop()
{
  
  unsigned int tempo_pressionado = 0;

  if (!digitalRead(BOTAO_CONTROLE))
  {
    tempo = millis();

    while (!digitalRead(BOTAO_CONTROLE))
    {
      tempo_pressionado = millis() - tempo;
    }
  }

  while(tempo_pressionado <= 2000 && tempo_pressionado != 0)
  {
    
    if (digitalRead(FALL) && cont_voltas < n_voltas + 1)
    {
      digitalWrite(SLEEP, 1);
      controlePid();
      Serial.println("FUNCIONA");
      contagem_de_voltas();
    }

    else
    {
      Serial.println("Erro na inicalizacao");
      digitalWrite(SLEEP, 0);
      break;
    }

  }

  if(tempo_pressionado > 2000 && tempo_pressionado != 0)
  {
    calibrar();
  }

}

void calibrar()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){13, 14, 27, 26, 25, 33, 32, 35}, NUM_SENSORES);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW);

  delay(1000);
}

double calculoPid(double input, double kp, double ki, double kd)
{
  double erro, valor_kp, valor_kd, Pid;
  static double valor_ki;

  erro = setpoint - input;

  valor_kp = erro * kp;

  valor_ki += erro * ki;

  valor_kd = (ultimo_val_sensor - input) * kd;

  ultimo_val_sensor = input;

  Pid = valor_kp + valor_ki + valor_kd;

  return Pid;
}

void controlePid()
{
  double pid;

  val_sensor = qtr.readLineBlack(valorSensores);

  if(digitalRead(SENSOR_CURVA) == Linha_preta)
  {
    curva = !curva;
    while(digitalRead(SENSOR_CURVA) == Linha_preta);
  }

  if (curva == 1 && valorSensores[3] < 900 && valorSensores[4] < 900)
  {
    pid = calculoPid(val_sensor, kp_c, ki_c, kd_c);
  }

  else if(curva == 0 && valorSensores[3] > 900 && valorSensores[4] > 900)
  {
    pid = calculoPid(val_sensor, kp, ki, kd);
  }
  else
  {
    pid = calculoPid(val_sensor, kp_c, ki_c, kd_c);
  }

  int vel_M1 = velMin_M1 + pid;
  int vel_M2 = velMin_M2 - pid;

  if (val_sensor == 0)
  {
    vel_M1 = velMax_M1;
    vel_M2 = 0;
  }
  else if (val_sensor == 7000)
  {
    vel_M1 = 0;
    vel_M2 = velMax_M2;
  }

  if (vel_M1 > velMax_M1)
  {
    vel_M1 = velMax_M1;
  }
  if (vel_M2 > velMax_M2)
  {
    vel_M2 = velMax_M2;
  }

  controleMotor(vel_M1, vel_M2);
}

void controleMotor(int vel_M1, int vel_M2)
{
  if (vel_M1 > 0)
  {
    ledcWrite(IN1_chanel, 0);
    ledcWrite(IN2_chanel, vel_M1);
  }
  else
  {
    ledcWrite(IN1_chanel, -vel_M1);
    ledcWrite(IN2_chanel, 0);
  }

  if (vel_M2 > 0)
  {
    ledcWrite(IN3_chanel, vel_M2);
    ledcWrite(IN4_chanel, 0);
  }
  else
  {
    ledcWrite(IN3_chanel, 0);
    ledcWrite(IN4_chanel, -vel_M2);
  }
}

void contagem_de_voltas()
{
  if (digitalRead(SENSOR_PARADA) == Linha_preta)
  {
    cont_voltas++;
    while (digitalRead(SENSOR_PARADA) == Linha_preta);
    
  }
}
