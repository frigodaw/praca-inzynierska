#include <LiquidCrystal.h> 
#include <Servo.h>
#include "leOS.h"
#define trigPin A5
#define echoPin A4

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
Servo serwo1, serwo2, serwo3;
leOS threads;

String inputString = "";
bool stringComplete = false, isConnected = false, measureDistance = true;
char buf[60];
int i,j,k;
float distance, kalman;
float fi1 = 0, fi3 = 30, d3 = 100;    //parametry startowe
float A = 1, C = 1, S, K;   //macierze 1x1
float V, W;                 //wektory
float x0, P0, xpri, Ppri, xpost, Ppost, eps;
float dt = 200, std_dev = 5;


void setup() {
  Serial.begin(9600);
  threads.begin(); 
  serwo1.attach(9);
  serwo2.attach(10);
  serwo3.attach(11);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  inputString.reserve(60);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Lacze z RPi...");
  delay(50);
  
  x0 = d3;
  P0 = d3+2;
  xpri = x0;
  Ppri = P0;
  xpost = x0;
  Ppost = P0;
  kalman = x0;
  V = 2*std_dev*dt/1000;
  W = std_dev*std_dev;
  
  serwo1.write(map(fi1, 80, 35, 0, 45));
  serwo2.write(map(fi3, 54, 144, 90, 0));
  threads.addTask(kalmanFilter, dt); 
  threads.addTask(regulator, dt);
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
    inputString.toCharArray(buf,60);
    divideArray(buf, &fi1, &fi3, &d3);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(":");
    lcd.print(fi1);
    
    lcd.setCursor(8,0);
    lcd.print(":");
    lcd.print(kalman);
    
    lcd.setCursor(0,1);
    lcd.print(":");
    lcd.print(fi3);
    
    lcd.setCursor(8,1);
    lcd.print(":");
    lcd.print(d3);
    
    serwo1.write(map(fi1, 80, 35, 0, 45));
    serwo2.write(map(fi3, 54, 144, 90, 0));
    
    memset(buf, 0 , sizeof(buf));
    inputString = "";
    stringComplete = false;
    isConnected = true;
  }
  
  if (isConnected){
    lcd.setCursor(8,0);
    lcd.print(":");
    lcd.print(kalman);
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

void kalmanFilter(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH) / 5.8;
  
  xpri = A*xpost;
  Ppri = A*Ppost*A + V;
  eps = distance - C*xpri;
  S = C*Ppri*C + W;
  K = Ppri*C/S;
  xpost = xpri + K*eps;
  Ppost = Ppri - K*S*K;
  kalman = xpost;
    
  
  /*Serial.print(distance);
  Serial.print("\t");
  Serial.println(kalman);*/
}

void regulator(){
  if(kalman >= d3 + 5) serwo3.write(0);
  else if (kalman <= d3 - 5) serwo3.write(180);
  else  {
    serwo3.write(90);
  }
}
