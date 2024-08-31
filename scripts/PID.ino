#include <BluetoothSerial.h>;

BluetoothSerial SerialBT;

#define PWMA 4 
#define AIN2 17
#define AIN1 16
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

int n_samples=0, i=0, j=0;

const unsigned int sensor[] = {27, 26, 25, 33, 32, 35};
const int n_sensores = 6;
int pesos[] = {-3, -2, -1, 1, 2, 3};

int rspeed;
int lspeed;
const int base_speed = 120;

int pos = 0;
int sensor_read[n_sensores];
long sensor_average = 0;
int sensor_sum = 0;

float p;
float integral;
float integral_max = 1000;
float integral_min = -1000;
float d;
float lp;
float error;
float correction;

float Kp = 0.8;
float Ki = 0;
float Kd = 1;

//as the time is stored into unsigned long int variables, the program can run for nearly 70 minutes
unsigned long current_time=0;
unsigned long last_time=0;
unsigned long current_sample=0;
unsigned long last_sample=0;

int pid_calc();
void calc_turn();

void setup()
{
  SerialBT.begin("LineFollower");
  for(int i=0; i<n_sensores; i++){
    pinMode(sensor[i], INPUT);
  }

  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW); 
  
  Serial.begin(115200);

//this for-loop will divide the weights for the total number of sensors
//so the standard deviation is more quickly calculated
//sdeviation = sqrt(sum(xi-mean)^2)/n
//the division is done at the just once
  for(i=0;i<n_sensores, i++){
    pesos[i] = i/n_sensores;
  }
  
  analogWrite(PWMA, base_speed);
  analogWrite(PWMB, base_speed);
}


void loop()
{
  if (SerialBT.available()) {
    String input = SerialBT.readString();
    updatePIDConstants(input); 
  }
  current_time = micros();
  if(current_time-last_time >= 4000){
    calc_turn();
  }
}

int pid_calc()
{
  sensor_average = 0;
  sensor_sum = 0;

  //the samplings shall be taken in a constant frequency
  //to do so, each sample begin 500 microsseconds after the last
  for(i=0;i<n_samples;i++){
    current_sample = micros;
    if (current_sample-last_sample >= 500)
    {
      for(j = 0; i < n_sensores; j++)
      {
        //reads each sensor individualy "n_samples" times
        sensor_read[j] += analogRead(sensor[j]);
      }
    }
    last_sample = current_sample;
  }

  for(i=0;i<n_sensores;i++){
    //calculates the weighted standard deviation
    //
    //use the variable error
    //
  }

  p = error;
  integral += p;
  integral = constrain(integral, integral_min, integral_max);
  d = p - lp;
  lp = p;

  return  int(Kp*p + Ki*integral + Kd*d);
}

void calc_turn()
{
  correction = pid_calc();
  rspeed = base_speed - correction;
  lspeed = base_speed + correction;
  
  rspeed = constrain(rspeed, 0, 255);
  lspeed = constrain(lspeed, 0, 255);
  
  //motor control discretization
  //this condition keeps all the motor driver commands equaly spaced in time
  current_time = micros();
  if(current_time-last_sample >= 4000){
    analogWrite(PWMA, rspeed);
    analogWrite(PWMB, lspeed); 
  }
  last_time = current_time;
}

void updatePIDConstants(String input) {
  input.trim();
  int kpIndex = input.indexOf("Kp=");
  int kiIndex = input.indexOf("Ki=");
  int kdIndex = input.indexOf("Kd=");
  
  if (kpIndex != -1 && kiIndex != -1 && kdIndex != -1) {
    String kpValue = input.substring(kpIndex + 3, input.indexOf(",", kpIndex));
    String kiValue = input.substring(kiIndex + 3, input.indexOf(",", kiIndex));
    String kdValue = input.substring(kdIndex + 3);

    Kp = kpValue.toFloat();
    Ki = kiValue.toFloat();
    Kd = kdValue.toFloat();

    SerialBT.print("Updated Kp: ");
    SerialBT.print(Kp);
    SerialBT.print(", Ki: ");
    SerialBT.print(Ki);
    SerialBT.print(", Kd: ");
    SerialBT.println(Kd);
  } else {
    SerialBT.println("Formato invalido! Esperado: Kp=0.5,Ki=0.0003,Kd=0.6");
  }
}