#include <BluetoothSerial.h>;

BluetoothSerial SerialBT;

#define PWMA 4 
#define AIN2 17
#define AIN1 16
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

const unsigned int sensor[] = {27, 26, 25, 33, 32, 35};
const unsigned int sensorLat[] = {13, 14, 34, 39};
const int n_sensores = 6;
int pesos[] = {-3, -2, -1, 1, 2, 3};

//speeds
int rspeed;
int lspeed;
const int base_speed = 100;

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
float sp;

float Kp = 2.4;
float Ki = 0.002;
float Kd = 7;
int cont, estado = 0;
int pid_calc();
void calc_turn();
void motor_drive(int , int );

void setup()
{
  SerialBT.begin("LineFollower");
  //sensors
  for(int i=0; i<n_sensores; i++){
    pinMode(sensor[i], INPUT);
  }

  //motors
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
  
  sp = 0;
  analogWrite(PWMA, base_speed);
  analogWrite(PWMB, base_speed);
}


void loop()
{
  //atual = micros();
  //Serial.println(atual-anterior);
  //anterior = atual;
  if (SerialBT.available()) {  // Check if Bluetooth data is available
    String input = SerialBT.readString();  // Read the incoming data as a string
    updatePIDConstants(input);  // Function to parse and update Kp, Ki, Kd
  }
  Serial.println(cont);
  Serial.println(analogRead(39));
  
  switch (estado){
    case 0:
      calc_turn();
      delay(5);
      if (analogRead(34) < 3500|| analogRead(39) < 3500)
        estado++;
      break;
    case 1:
      calc_turn();
      delay(5);
      if (analogRead(34) > 3500|| analogRead(39) > 3500)
        estado ++;
      break;
    case 2:
      calc_turn();
      delay(5);
      if (analogRead(34) < 3500|| analogRead(39) < 3500)
        estado++;
      break; 
    case 3:
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0); 
  }
  /*
  if(cont < 2){
    calc_turn();
    delay(5);
    
    if(analogRead(34) < 3500|| analogRead(39) < 3500){
    //direito com linha e esquerdo não  
      cont++;
      
    }
  } else {
    
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0); 
  }
  */
}

int pid_calc()
{
  sensor_average = 0;
  sensor_sum = 0;

  for(int i = 0; i < 6; i++)
  {
    sensor_read[i]=analogRead(sensor[i]);
    sensor_average += sensor_read[i]*pesos[i]*1000;
    sensor_sum += sensor_read[i];
  }

  pos = int(sensor_average / sensor_sum);

  error = pos-sp;
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
  
  analogWrite(PWMA, rspeed);
  analogWrite(PWMB, lspeed); 
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
