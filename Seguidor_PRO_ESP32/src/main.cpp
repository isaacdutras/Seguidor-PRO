#include <Arduino.h>
#include <QTRsensors.h>

#define GENERO "Binario"

#define VERSAO 1.0

// Definição Pinos da ponte H
#define IN1 2    // Horario Motor Esquerdo certo
#define IN2 4    // Antihorario Motor Esquerdo errado
#define IN3 5    // Horario Motor Direito errado
#define IN4 18   // Antihorario Motor Direito errado
#define SLEEP 19 // Ativa ou desativa o output da ponte H
#define FALL 21  // Detecta se tem algum erro

#define OFFSET 0 // Diferenca de potencia entre os dois motores

void calibrar();
double calculoPid();
void controlePid();
void controleMotor(int vel_M1, int vel_M2);

const int BITS = 10;     // Define a resolucao do pwm para 10 Bits (2^10 ou 1024)
const int FREQ = 100000; // Define a frequencia do pwm
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

unsigned long int tempo;

int ultimo_val_sensor = 0;

double kp_c = 0, ki_c = 0, kd_c = 0;
double kp = 0.160, ki = 0.01, kd = 0.45;

uint16_t val_sensor;

QTRSensors qtr; // Objeto qtr

void setup()
{
  Serial.begin(115200);

  ledcSetup(IN1_chanel, FREQ, BITS);
  ledcSetup(IN2_chanel, FREQ, BITS);
  ledcSetup(IN2_chanel, FREQ, BITS);
  ledcSetup(IN4_chanel, FREQ, BITS);

  ledcAttachPin(IN1, IN1_chanel);
  ledcAttachPin(IN2, IN2_chanel);
  ledcAttachPin(IN1, IN3_chanel);
  ledcAttachPin(IN4, IN4_chanel);

  pinMode(SLEEP, OUTPUT);
  pinMode(FALL, INPUT_PULLUP);

  calibrar();

  controleMotor(0, 0);
}

void loop()
{

  if (digitalRead(FALL))
  {
    digitalWrite(SLEEP, 1);
    controlePid();
  }

  else
  {
    Serial.println("Erro na inicalizacao");
  }
}

void calibrar()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32, 35, 34}, NUM_SENSORES);
  qtr.setEmitterPin(13);

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
  val_sensor = qtr.readLineBlack(valorSensores);

  double pid = calculoPid(val_sensor, kp, ki, kd);

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
    ledcWrite(IN3, 0);
    ledcWrite(IN4_chanel, -vel_M2);
  }
}
