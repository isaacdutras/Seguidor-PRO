#include <Arduino.h>
#include <QTRSensors.h> //Biblioteca do sensor infravermelho

#define VERSAO 1.0
// Definição Pinos da ponte H--------------------------------------------------------------------------------------

#define IN1 7   // Horario Motor Esquerdo
#define IN2 5   // Antihorario Motor Esquerdo
#define IN3 8   // Horario Motor Direito
#define IN4 6   // Antihorario Motor Direito
#define SLEEP 3 // Ativa ou desativa o output da ponte H
#define FALL 2  // Detecta se tem algum erro
//-----------------------------------------------------------------------------------------------------------------

#define OFFSET 15 // Diferenca de potencia entre os dois motores
// Declaração das funções-------------------------------------------------------------------------------------------

void calibrar();
double calculoPid();
void controlePid();
void controleMotor(int vel_M1, int vel_M2);
// Variaveis----------------------------------------------------------------------------------------------------------------

const int velMax_M1 = 200;
const int velMax_M2 = 200 + OFFSET;

const int velMin_M1 = 150;
const int velMin_M2 = 150 + OFFSET;

const int setpoint = 3500;

const uint8_t NUM_SENSORES = 5;
uint16_t valorSensores[NUM_SENSORES];

unsigned long int tempo;

int ultimo_val_sensor = 0;

double kp_c = 0, ki_c = 0, kd_c = 0;
double kp = 0.04, ki = 0, kd = 0;

double val_sensor;
// Objetos---------------------------------------------------------------------------------------------------------

QTRSensors qtr;
//----------------------------------------------------------------------------------------------------------------

void setup()
{

  Serial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(FALL, INPUT_PULLUP);

  calibrar();

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void loop()
{
  if (digitalRead(FALL))
  {
    digitalWrite(SLEEP, 1);

    if (millis() - tempo >= 200)
    {
      qtr.readLineBlack(valorSensores);

      for (int i = 0; i < NUM_SENSORES; i++)
      {
        Serial.print(valorSensores[i]);
        Serial.print('\t');
      }
      Serial.println("");

      controlePid();
    }
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

  for (uint8_t i = 0; i < NUM_SENSORES; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < NUM_SENSORES; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
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
  static int last_value = 0;
  double media = 0;
  double soma = 0;

  qtr.readLineBlack(valorSensores);

  while (valorSensores[0] > 900 && valorSensores[1] > 900 && valorSensores[2] > 900 && valorSensores[3] > 900 && valorSensores[4] > 900 && valorSensores[5] > 900 && valorSensores[6] > 900 && valorSensores[7] > 900)
  {
    if (ultimo_val_sensor > 0)
    {
      controleMotor(velMax_M1, -velMax_M2);
    }
    else
    {
      controleMotor(-velMax_M1, velMax_M2);
    }
    qtr.readLineBlack(valorSensores);
  }

  for (int i = 0; i < NUM_SENSORES; i++)
  {
    int aux = valorSensores[i] > 700;
    media += aux * i * 1000;
    soma += aux;
  }

  if (soma == 0)
    soma = 1;

  val_sensor = (media / soma);

  if (val_sensor == 0 && last_value == 7000)
    val_sensor = 7000;

  else
    last_value = val_sensor;

  double pid = calculoPid(val_sensor, kp, ki, kd);

  int vel_M1 = velMin_M1 - pid > 0 ? velMin_M1 - pid : 0;
  int vel_M2 = velMin_M2 + pid > 0 ? velMin_M2 + pid : 0;

  if (vel_M1 > velMax_M1)
  {
    vel_M1 = velMax_M1;
  }
  if (vel_M2 > velMax_M2)
  {
    vel_M2 = velMax_M2;
  }
  Serial.print("SENSOR: ");
  Serial.print(val_sensor);
  Serial.print('\t');
  Serial.print("M1: ");
  Serial.print(vel_M1);
  Serial.print("M2: ");
  Serial.println(vel_M2);

  controleMotor(vel_M1, vel_M2);
}

void controleMotor(int vel_M1, int vel_M2)
{

  if (vel_M1 > 0)
  {
    analogWrite(IN1, vel_M1);
  }
  else
  {
    analogWrite(IN2, -vel_M1);
  }

  if (vel_M2 > 0)
  {
    analogWrite(IN3, vel_M2);
  }
  else
  {
    analogWrite(IN4, -vel_M2);
  }
}
