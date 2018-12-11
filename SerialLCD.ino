#include <LiquidCrystal.h> 
#include <Servo.h>
#include "leOS.h"
#define ANALOG_IN 0 //wejscie na pinie A0
#define SENSOR_VCC 5.0 //zasilanie 5.0V lub 3.3V

LiquidCrystal lcd(2, 3, 4, 5, 7, 8);
Servo serwo1, serwo2, serwo3;
String inputString = "";
bool stringComplete = false;
char buf[60];
int i,j,k;
unsigned int analogInput = 0;   //wartość z przetwornika ADC
float inputToVoltage = 0;       //zamiana wartości z przetwornika na napięcie
float dystans = 0;
float coeffs[2] = {11.4858, 0.0705}; //wspołczynniki a i b rownania aproksymacji liniowej y=ax+b
float fi1=0, fi3=30, d3=60;
leOS threads;


void setup() {
  Serial.begin(9600);
  threads.begin(); 
  inputString.reserve(60);
  serwo1.attach(6);
  serwo2.attach(9);
  serwo3.attach(10);
  lcd.begin(13, 2);
  lcd.setCursor(0, 0);
  lcd.print("Lacze z RPi...");
  lcd.setCursor(0, 1);
  lcd.print("Dane: ");
  delay(200);
  
  serwo1.write(map(fi1, 80, 35, 0, 45));
  serwo2.write(map(fi3, 124, 34, 0, 90));
  threads.addTask(distance, 200); 
  threads.addTask(d3reg, 200);
}

void loop() {    // print the string when a newline arrives
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  if (stringComplete) {
    Serial.println(inputString);
    inputString.toCharArray(buf,60);
    
    divideArray(buf, &fi1, &fi3, &d3);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(":");
    lcd.print(fi1);
    
    lcd.setCursor(0,1);
    lcd.print(":");
    lcd.print(fi3);
    
    lcd.setCursor(8,1);
    lcd.print(":");
    lcd.print(d3);

    
    serwo1.write(map(fi1, 80, 35, 0, 45));
    serwo2.write(map(fi3, 124, 34, 0, 90));
    delay(50);
    
    memset(buf, 0 , sizeof(buf));
    i=0;
    j=0;
    k=0;
    inputString = "";
    stringComplete = false;
  }
}
 
void divideArray(char* inputArray, float* fi1_org, float* fi3_org, float* d3_org){
  int i=0, j=0, k=0;
  char fi1_buf[20], fi3_buf[20], d3_buf[20];
  
  memset(fi1_buf, 0 , sizeof(fi1_buf));
  memset(fi3_buf, 0 , sizeof(fi3_buf));
  memset(d3_buf, 0 , sizeof(d3_buf));

  while(i>=0){
    if((inputArray[i] == 'x') or (inputArray[i] == '\n')){
      j=i+1;
      i=-1;
    }
    else {
      fi1_buf[i] = inputArray[i];
      i+=1;
    }
  }
  i=0;
  
  while(j>=0){
    if((buf[j] == 'x') or (buf[i] == '\n')){
      k=j+1;
      j=-1;
    }
    else {
      fi3_buf[i] = buf[j];
      i+=1;
      j+=1;
    }
  }
  i=0;

  while(k>=0){
    if((buf[k] == 'x') or (buf[i] == '\n')){
      k=-1;
    }
    else {
      d3_buf[i] = buf[k];
      i+=1;
      k+=1;
    }
  }  
  *fi1_org = atof(fi1_buf)/1000;
  *fi3_org = atof(fi3_buf)/1000;
  *d3_org = atof(d3_buf)/1000;
}

void distance(){ Serial.print("watek");
  analogInput = analogRead(A0);
  inputToVoltage = SENSOR_VCC/1024.0 * analogInput;
  dystans = coeffs[0]/(inputToVoltage - coeffs[1])-0.42;
  dystans *= 10; //cm to mm
  //Serial.print("\n Sygnał z przetwornika ADC:\t "); Serial.print(analogInput);
  //Serial.print("\n Napięcie wejścia: \t"); Serial.print(inputToVoltage); Serial.print("V \n");
  Serial.print(" Odleglosc: \t"); Serial.print(dystans); Serial.print("cm \n");
  }

void d3reg(){
  if(dystans >= d3 + 5) serwo3.write(0);
  else if (dystans <= d3 - 5) serwo3.write(180);
  else serwo3.write(90);
}
