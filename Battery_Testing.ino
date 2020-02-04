/* Battery Tester Project 
 *  Created on 1/29/2020
 *  Intended for EE MUN Term 5 Project
 */
#include <rgb_lcd.h>
#include <SD.h>
#include <SPI.h>
#define float_voltage 13.8
#define default_temp 25
#define Current_Sens_Pin A0
#define Bat_VSensor_Pin A1
#define Temp_Sensor A2
#define Charged_BatVoltage 12.5
/*RELAYS***************************/
#define RELAY_1 1
#define RELAY_2 2
#define RELAY_3 3
#define RELAY_5 5
#define RELAY_6 6
#define MOSI 11
#define MISO 12
#define CLK 13
#define CS 10
/*OUTPUT CONTROL*******************/
#define T_base 3 // PWM Pin
/*OTHER CONSTS AND OBJECTS*********/
double cutoff_voltage = 0.0;
double open_circuit_voltage = 0.0;
double battery_terminal_voltage = 0.0;
double peukerts_const = 1.4;
int C_rating = 20;
int Ah_rating = 7;
double discharge_current = 0.0;
double user_current_input = 0.0;
double discharge_time = 0.0;
bool user_set = false;
rgb_lcd lcd;
/*ROTARY ENCODER*******************/
#define clk 7
#define dt 8
#define sw 4
volatile boolean button = false;
volatile boolean up = false;
volatile boolean TurnDetected = false;
char arrowpos = 0;
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/*INTERRUPT************************/
ISR(PC_INT2_VECT){
  if (digitalRead(sw) == LOW){
    button = true;
  }
}
void isr0(){
  TurnDetected = true;
  up = (digitalRead(clk) == digitalRead(dt));
}
/*LCD POINTER Character***********/
byte cursor_char[8] = {
  0b10000,
  0b11000,
  0b11100,
  0b11110,
  0b11110,
  0b11100,
  0b11000,
  0b10000
};
/**********************************************/
/*Functions*/
/*Function to check the open-circuit battery voltage and terminal voltage*/
bool setup_battery_charging(bool first_setup=false);  // get open circuit voltage and check if charging required 
void record_battery_data();     // record the current and voltage readings
void manage_charging();         // 
void check_temp();              // check the temperature of the battery
void setup_discharge();         // setup the discharge rate and cutoff voltage
void update_battery_status();   // get the battery status
double calculate_Ah_rating();   // calculate the Ah rating using Peukert's Law
/*Helper Functions****************/
double get_current();
double get_voltage();
void record_data();
/**Screens********************************/    
void welcome_screen();
void screen0();
void screen1();
void screen2();


void setup(){
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_5, OUTPUT);
  pinMode(RELAY_6, OUTPUT);
  pinMode(T_base, OUTPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  ADCSRA &= ~PS_128;
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
  PCICR |= 0b00000100;
  PCMSK2 |= 0b00010000;
  attachInterrupt(0, isr0, RISING);
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 0);
//Serial.begin(9600);
  lcd.clear();
  lcd.createChar(0, cursor_char);
  welcome_screen();
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
}


void loop(){

  

  
}



void welcome_screen(){
  lcd.clear();
  lcd.setCursor(1, 0); // lcd.setCursor(col, row)
  lcd.print("Battery Tester");
  lcd.setCursor(1,1);
  lcd.print("Welcome");
}

void screen0(){
  
}

bool setup_battery_charging(bool first_setup=false){
  double charge_current = Ah_rating/3.0;
  if(first_setup){
  open_circuit_voltage = (map(analogRead(Bat_VSensor_Pin), 0, 1023, 0 ,5)*((100.0+330.0)/100.0));
  if (open_circuit_voltage > Charged_BatVoltage){
    return false;
  }
  }
  if (user_set){
  charge_current = user_current_input;
  }
  if(charge_current > 4.0){
    charge_current = 4.0;
  }
  int base_voltage = 0;
  analogWrite(T_base, base_voltage);
  digitalWrite(RELAY_1, HIGH);
  while(get_current() < charge_current){
    base_voltage += 1;
    if (base_voltage > 255){
      base_voltage = 255;
    }
    analogWrite(T_base, base_voltage);
    delayMicroseconds(100);
  }
  while(charge_current > get_current()){
    base_voltage -= 1;
    if(base_voltage <= 0){
      base_voltage = 0;
    }
    analogWrite(T_base, base_voltage);
    delayMicroseconds(100);
  }
  return true;
  
}

double get_current(){
  return ((analogRead(Current_Sens_Pin)/1024.0)*(5000/100));
}

double get_voltage(){
  return (map(analogRead(Bat_VSensor_Pin), 0, 1023, 0 ,5)*((100.0+330.0)/100.0));
}
