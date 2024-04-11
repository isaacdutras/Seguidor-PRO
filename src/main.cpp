#include <Arduino.h>
#include <QTRSensors.h> //Biblioteca do sensor infravermelho

#define GENERO "Binario"

#define VERSAO 1.0
// Definição Pinos da ponte H--------------------------------------------------------------------------------------

#define IN1 7   // Horario Motor Esquerdo certo
#define IN2 6   // Antihorario Motor Esquerdo errado
#define IN3 8   // Horario Motor Direito errado
#define IN4 9   // Antihorario Motor Direito errado
#define SLEEP 3 // Ativa ou desativa o output da ponte H
#define FALL 2  // Detecta se tem algum erro
//-----------------------------------------------------------------------------------------------------------------

#define OFFSET 0 // Diferenca de potencia entre os dois motores
// Declaração das funções-------------------------------------------------------------------------------------------

void calibrar();
double calculoPid();
void controlePid();
void controleMotor(int vel_M1, int vel_M2);
// Variaveis----------------------------------------------------------------------------------------------------------------

const int velMax_M1 = 255 + OFFSET;
const int velMax_M2 = 255;

const int velMin_M1 = 232 + OFFSET;
const int velMin_M2 = 232;

const int setpoint = 3500;

const uint8_t NUM_SENSORES = 8;
uint16_t valorSensores[NUM_SENSORES];

unsigned long int tempo;

int ultimo_val_sensor = 0;

double kp_c = 0, ki_c = 0, kd_c = 0;
double kp = 0.164, ki = 0, kd = 0.4;

uint16_t val_sensor;

QTRSensors qtr; // Objeto qtr

void setup()
{
  //Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(FALL, INPUT_PULLUP);

  calibrar();

  controleMotor(0,0);
  
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
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, NUM_SENSORES);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

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
    analogWrite(IN1, 0);
    analogWrite(IN2, vel_M1);
  }
  else
  {
    analogWrite(IN1, -vel_M1);
    analogWrite(IN2, 0);
  }

  if (vel_M2 > 0)
  {
    analogWrite(IN3, vel_M2);
    analogWrite(IN4, 0);
  }
  else
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, -vel_M2);
  }

}
