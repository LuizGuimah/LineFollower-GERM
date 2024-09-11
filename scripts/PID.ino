#define sens1 13
#define sens2 14

int estado = 0;

void setup() {
  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println(analogRead(sens1));

  switch (estado){
    case 0:
      Serial.println("Linha preta 0");
      if (analogRead(sens1) < 3500){
        estado++;
        Serial.println("Linha branca 0");
      }
      break;
    case 1:
      Serial.println("Linha branca 1");
      if (analogRead(sens1) > 3500){
        estado ++;
        Serial.println("Linha preta 1");
      }
      break;
    case 2:
      Serial.println("Linha preta 2");
      if (analogRead(sens1) < 3500){
        Serial.println("Linha branca 2, PARADO");
      }
      break; 
  }
}
