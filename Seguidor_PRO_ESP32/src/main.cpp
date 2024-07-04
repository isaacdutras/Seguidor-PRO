#include <Arduino.h>
#include <QTRsensors.h>

#define GENERO "Binario"

#define VERSAO 1.0

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

const int velMax_M1 = 1080 + OFFSET;
const int velMax_M2 = 1080;

const int velMin_M1 = 750 + OFFSET;
const int velMin_M2 = 750;

const int setpoint = 3500;

const uint8_t NUM_SENSORES = 6;
uint16_t valorSensores[NUM_SENSORES];

const int BOTAO_CONTROLE = 23;
const int SENSOR_CURVA = 15;
const int SENSOR_PARADA = 34;
unsigned long int tempo;

int ultimo_val_sensor = 0;

double kp_c = 5, ki_c = 0, kd_c = 4.2;
double kp = 0.13, ki = 0, kd = 0.8;

bool curva = 0;

int n_voltas = 1; // Variável para definir o número de voltas na pista
int cont_voltas = 0;

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
  digitalWrite(SLEEP, 0);
  calibrar();
} 

void loop()
{

  int var = qtr.readLineBlack(valorSensores);

  for(int i = 0; i < NUM_SENSORES; i++)
  {
    Serial.print(valorSensores[i]);
    Serial.print('\t');
  }

  Serial.println(var);

  // unsigned int tempo_pressionado = 0;

  // if (!digitalRead(BOTAO_CONTROLE))
  // {
  //   tempo = millis();

  //   while (!digitalRead(BOTAO_CONTROLE))
  //   {
  //     tempo_pressionado = millis() - tempo;
  //   }
  // }

  // while(tempo_pressionado <= 500 && tempo_pressionado != 0)
  // {
    
  //   if (digitalRead(FALL) && cont_voltas < n_voltas + 1)
  //   {
  //     digitalWrite(SLEEP, 1);
  //     controlePid();
  //     contagem_de_voltas();
  //   }

  //   else
  //   {
  //     digitalWrite(SLEEP, 0);
  //     cont_voltas = 0;
  //     break;
  //   }
 
  // }

  // if(tempo_pressionado >= 1200 && tempo_pressionado != 0)
  // {
  //   calibrar();
  // }

}

void calibrar()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){14, 27, 26, 25, 33, 32}, NUM_SENSORES);
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

  int val_sensor = qtr.readLineBlack(valorSensores);

  // if (digitalRead(SENSOR_CURVA) == Linha_preta)
  // {
  //   curva = !curva;

  //   while (digitalRead(SENSOR_CURVA));
    
  // }
  // if (digitalRead(SENSOR_PARADA) == Linha_preta)
  // {
  //   cont_voltas++;
  //   while (digitalRead(SENSOR_PARADA) == Linha_preta);
    
  // }
  // else if(digitalRead(SENSOR_PARADA) == Linha_preta && digitalRead(SENSOR_CURVA) == Linha_preta);

  // if (valorSensores[0] > 900)
  // {
  //   val_sensor = 0;
  // }
  // if (valorSensores[7] > 900)
  // {
  //   val_sensor = 7000;
  // }

  // if (curva == 1 && valorSensores[3] < 900 && valorSensores[4] < 900)
  // {
  //   pid = calculoPid(val_sensor, kp_c, ki_c, kd_c);
  // }
  // else if (curva ==  0 && valorSensores[3] > 900 && valorSensores[4] > 900)
  // {
  //   pid = calculoPid(val_sensor, kp, ki, kd);
  // }
  // else
  // {
  //   pid = calculoPid(val_sensor, kp_c, ki_c, kd_c);
  // }

  pid = calculoPid(val_sensor, kp, ki, kd);
  
  int vel_M1 = velMin_M1 - pid;
  int vel_M2 = velMin_M2 + pid;

  if (vel_M1 > velMax_M1)
  {
    vel_M1 = velMax_M1;
  }
  if (vel_M2 > velMax_M2)
  {
    vel_M2 = velMax_M2;
  }

  if (vel_M1 < 0)
  {
    vel_M1 = 0;
  }
  
  if (vel_M2 < 0)
  {
    vel_M2 = 0;
  }

  controleMotor(vel_M1, vel_M2);

}

void controleMotor(int vel_M1, int vel_M2)
{
  if (vel_M1 > 0)
  {
    ledcWrite(IN1_chanel, 0);
    ledcWrite(IN2_chanel, vel_M1);
    Serial.print("M1: ");
    Serial.print(vel_M1);
    Serial.println('\t');

  }
  else
  {
    ledcWrite(IN1_chanel, -vel_M1);
    ledcWrite(IN2_chanel, 0);
    Serial.print("M1: ");
    Serial.print(-vel_M1);
    Serial.println('\t');
  }

  if (vel_M2 > 0)
  {
    ledcWrite(IN3_chanel, vel_M2);
    ledcWrite(IN4_chanel, 0);
    Serial.print("M2: ");
    Serial.println(vel_M2);
  }
  else
  {
    ledcWrite(IN3_chanel, 0);
    ledcWrite(IN4_chanel, -vel_M2);
    Serial.print("M2: ");
    Serial.println(-vel_M2);
  }
}

void contagem_de_voltas()
{
  if (digitalRead(SENSOR_PARADA) == Linha_preta)
  {

  }
}