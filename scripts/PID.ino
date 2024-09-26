#include <BluetoothSerial.h>;

BluetoothSerial SerialBT;

#define PWMA 4 
#define AIN2 17
#define AIN1 16
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

const unsigned int sensor[] = {14, 27, 26, 25, 33, 32, 35, 34};
const int n_sensores = 8;
int pesos[] = {-4, -3, -2, -1, 1, 2, 3, 4};

//speeds
int rspeed;
int lspeed;
const int base_speed = 80;

float pos = 0;
float sensor_read[n_sensores];
float sensor_average = 0;
float sensor_sum = 0;
int r_read=0;

float p;
float integral;
float integral_max = 5000;
float integral_min = -5000;
float d;
float lp;
float error;
float correction;

float Kp = 2.4;
float Ki = 0.002;
float Kd = 7;
int state = 0;
int pid_calc();
void calc_turn();

String input;
long anterior;

void setup()
{
  SerialBT.begin("LineFollower");
  //sensors
  for(int i=0; i<n_sensores; i++){
    pinMode(sensor[i], INPUT);
  }
  pinMode(0, INPUT);

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
  
  while(!SerialBT.available()) {delay(10);}
  input = SerialBT.readString();  // Read the incoming data as a string
  updatePIDConstants(input); 
  
  anterior = micros();
  analogWrite(PWMA, base_speed);
  analogWrite(PWMB, base_speed);
}


void loop()
{
  if (SerialBT.available()) {  // Check if Bluetooth data is available
    input = SerialBT.readString();  // Read the incoming data as a string
    updatePIDConstants(input);  // Function to parse and update Kp, Ki, Kd
  }

  r_read = analogRead(36) < 3000;
  switch (state){
    case 0:
      if (r_read)
        state++;
      break;
    case 1:
      if (!r_read)
        state++;
      break;
    case 2:
      if (r_read)
        state++;
      break; 
    case 3:
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
      while(!SerialBT.available()) {delay(10);}
      String input = SerialBT.readString();
      updatePIDConstants(input);
      state=0;
      break;
  }
  calc_turn();
}

int pid_calc()
{
  sensor_average = 0;
  sensor_sum = 0;
  while(micros() - anterior < 10000){}
    for(int i = 0; i < n_sensores; i++)
    {
      sensor_read[i]=analogRead(sensor[i]);
      sensor_average += sensor_read[i]*pesos[i];
      sensor_sum += sensor_read[i];
    }
  anterior = micros();
 
  pos = (sensor_average / sensor_sum)*100;
  if(pos>1){
    p=pos-1;
  }
  else{
    if(pos<-1){
      p = pos+1;
    }
  }

  integral += p;
  integral = constrain(integral, integral_min, integral_max);
  d = p - lp;
  lp = p;
  
  return  int(Kp*p + Ki*integral + Kd*d);
}

void calc_turn()
{
  correction = pid_calc();
  rspeed = (base_speed + correction);
  lspeed = (base_speed - correction);
  //Serial.print(lspeed);
  //Serial.print("  ");
  //Serial.println(rspeed);
  
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
