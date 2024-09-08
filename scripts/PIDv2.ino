#include <BluetoothSerial.h>;

BluetoothSerial SerialBT;

#define PWMA 4 
#define AIN2 17
#define AIN1 16
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

//number of samples to be taken in one sampling cycle
//n_samples will be taken, then smoothed to be passed to the PID function
//to be optimized
int n_samples=4;

//iterators
int i=0, j=0;

//array, number and weights of the front IR sensorss
const unsigned int sensor[] = {27, 26, 25, 33, 32, 35};
const int n_sensors = 6;
float wheight[] = {-3, -2, -1, 1, 2, 3};

//right motor | left motor
//base speed is one variable to be optimized
int rspeed;
int lspeed;
const int base_speed = 100;

//pos represents the robot's deviation from the line
int pos = 0;

//array to store the readings from the front IR sensors
//the average is used to calculate the deviation
int sensor_read[n_sensors];
float sensor_wheighted_sum = 0;
float sensor_sum = 0;

//variables to be multiplied by kp, ki and kd
float p;
float integral;
float d;

//lp will store the last error
float lp;

//correction stores the return from the PID function
float correction;

//multipliers to be optimized
float Kp = 0.8;
float Ki = 0;
float Kd = 1;
//limitants to the integral variable
float integral_max = 1000;
float integral_min = -1000;

//as the time is stored into unsigned long int variables, the program can run for nearly 70 minutes
unsigned long current_time=0;
unsigned long last_time=0;
unsigned long current_sample_time=0;
unsigned long last_sample_time=0;

int pid_calc();
void calc_turn();

void setup()
{
  SerialBT.begin("LineFollower");
  for(int i=0; i<n_sensors; i++){
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
  for(i=0;i<n_sensors; i++){
    wheight[i] = i/n_samples;
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
  while(current_time - last_sample_time < 10000){}
  calc_turn();
}

int pid_calc()
{
  sensor_wheighted_sum = 0;
  sensor_sum = 0;

  //the samplings shall be taken in a constant frequency
  //to do so, each sample begin 500 microsseconds after the last
  for(i=0;i<n_samples;i++){
    current_sample_time = micros();
      while(current_time-last_sample_time < 500){
    }
      for(j = 0; j < n_sensors; j++)
      {
        //reads each sensor individualy "n_samples" times
        sensor_read[j] += analogRead(sensor[j]);
        Serial.print(sensor_read[j]);
        Serial.print("  ");
      }
      Serial.println();
    last_sample_time = current_sample_time;
  }

  for(i=0;i<n_sensors;i++){
    sensor_wheighted_sum+= sensor_read[i]*wheight[i];
    sensor_sum += sensor_read[i];
  }
  p = sensor_wheighted_sum/sensor_sum;
  Serial.print(p);
  Serial.println();
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
  Serial.print(lspeed);
  Serial.print("  ");
  Serial.print(rspeed);
  Serial.println();
  
  //keep the pwm in this range
  rspeed = constrain(rspeed, 0, 255);
  lspeed = constrain(lspeed, 0, 255);
  
  //motor control discretization
  //this condition keeps all the motor driver commands equaly spaced in time
  current_time = micros();
  while(current_time-last_sample_time < 8000){
  }
  analogWrite(PWMA, rspeed);
  analogWrite(PWMB, lspeed); 
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