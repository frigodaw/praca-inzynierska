#include <LiquidCrystal.h> 
#include <Servo.h>

LiquidCrystal lcd(2, 3, 4, 5, 7, 8);
Servo serwo1, serwo2, serwo3;
String inputString = "";
bool stringComplete = false;
int i,j,k;
float fi1, fi3, d3;
char buf[60];

void setup() {
  Serial.begin(9600);
  inputString.reserve(60);
  serwo1.attach(6);
  serwo2.attach(9);
  serwo3.attach(10);
  lcd.begin(13, 2);
  lcd.setCursor(0, 0);
  lcd.print("Lacze z RPi...");
  lcd.setCursor(0, 1);
  lcd.print("Dane: ");
  
}

void loop() {    // print the string when a newline arrives
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

    serwo1.writeMicroseconds(fiToUs(fi1,-90,90));
    serwo2.writeMicroseconds(fiToUs(fi3,-90,90));
    serwo3.writeMicroseconds(fiToUs(d3,-90,90));
    
    memset(buf, 0 , sizeof(buf));
    i=0;
    j=0;
    k=0;
    inputString = "";
    stringComplete = false;
  }
}


void serialEvent() {    //when new data comes, each time between loop()
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
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

int fiToUs(float angle, float minimum, float maximum){
  int us = (int)(round((1000*angle + 1000*maximum - 2000*minimum)/(maximum - minimum)));
  Serial.print(us);
  return us;
}
