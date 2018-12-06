#include <LiquidCrystal.h> 
#include <Servo.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
Servo serwo9, serwo10; 

char znak[7];
int numer_serwa;
int wypelnienie;

void setup() {
  serwo9.attach(9);
  
  lcd.begin(16, 2);
  Serial.begin(9600);
  lcd.setCursor(0, 0);
  lcd.print("Lacze z RPi...");
  lcd.setCursor(0, 1);
  lcd.print("Dane: ");
}
 
void loop() {
  if (Serial.available()){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Polaczono z RPi!");
    lcd.setCursor(0, 1);
    lcd.print("Nr:");
    znak[0] = Serial.read();  //numer serwa
    if (znak[0] == '9') numer_serwa = 9;
    else if (znak[0] == 'a') numer_serwa = 10;
    else if (znak[0] == 'b') numer_serwa = 11;
    lcd.print(numer_serwa);
    lcd.setCursor(5, 1);
    lcd.print(" Kat:");
    
    for(int i=1; i<7; i++){
      if (znak[i] > -1){
        znak[i] = Serial.read();
        lcd.setCursor(9+i, 1);
        lcd.print(znak[i]);
      }
    }

    wypelnienie = (int)(5/900 * atof(&znak[1]) + 1000);
    serwo9.writeMicroseconds(wypelnienie);
      
    Serial.write("papiesz 2138");
  }
}
