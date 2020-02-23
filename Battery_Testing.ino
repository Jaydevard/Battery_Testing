

/* Battery Tester Project 
 *  Created on 1/29/2020
 *  Intended for EE MUN Term 5 Project
 *  Author: Jaydev Madub && Girish Harendra Ramful
 */
#include <rgb_lcd.h>
#include <SD.h>
#include <SPI.h>
#include <Chrono.h>
#include <LightChrono.h>
#include <curveFitting.h>
#define float_voltage 13.8
#define default_temp 25
#define Current_Sens_Pin A0
#define Bat_VSensor_Pin A1
#define Temp_Sensor A2
#define Current_SensD_Pin A3
#define Charged_BatVoltage 12.5
/*RELAYS***************************/
#define RELAY_1 1
#define RELAY_2 8
#define MRELAY 7
/*MOSFETS**************************/
#define MOSF1 9
#define MOSF2 10
/*SD Card**************************/
#define MOSI 11
#define MISO 12
#define CLK 13
#define CS 6
/*OUTPUT CONTROL*******************/
#define T_base 5 // PWM Pin
/*OTHER CONSTS AND OBJECTS*********/
const double cutoff_voltage = 0.0;
double open_circuit_voltage = 0.0;
double battery_terminal_voltage = 0.0;
const double peukerts_const = 1.4;
int C_rating = 20;
int Ah_rating = 7;
double discharge_current = 0.0;
double user_current_input = 0.0;
double discharge_time = 0.0;
bool user_set = false;
bool charge_complete = false;
bool discharge_complete = false;
bool charge_mode = false;
bool discharge_mode = false;
rgb_lcd lcd;
File datalog;
Chrono Timer;
/*ROTARY ENCODER*******************/
#define clk 2
#define dt 3
#define sw 4
double x[100]; 
double y[100];
volatile boolean button = false;
volatile boolean up = false;
volatile boolean TurnDetected = false;
char arrowpos = 0;
char screen = 0;
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/*INTERRUPT************************/
ISR(PCINT2_vect){
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

byte custom_char[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b00000,
  0b00000,
  0b11111,
  0b01110,
  0b00100,
};
/**********************************************/
/*Functions*/
/*Function to check the open-circuit battery voltage and terminal voltage*/
void record_battery_data();     // record the current and voltage readings
void manage_charging();         // function to manage the charging process
void manage_discharing();       // function to manage discharginh process
void check_temp();              // check the temperature of the battery
void setup_discharge();         // setup the discharge rate and cutoff voltage
void update_battery_status();   // get the battery status
double calculate_Ah_rating();   // calculate the Ah rating using Peukert's Law
/*Helper Functions****************/
double get_current();           
double get_voltage();
double get_discharge_current();
int record_data();
bool close_file();
/**Screens********************************/    
void welcome_screen();
void mode_screen();
void set_testing_param_screen();
void charge_mode_screen();
void discharge_mode_screen();
void charging_mode_ongoing_screen();
void discharging_mode_ongoing_screen();


void setup(){
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(MOSF1, OUTPUT) ;
  pinMode(MOSF2, OUTPUT);
  pinMode(T_base, OUTPUT);
  pinMode(sw, INPUT_PULLUP);
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  ADCSRA &= ~PS_128;
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
  PCICR |= 0b00000100;
  PCMSK2 |= 0b00010000;
  attachInterrupt(0, isr0, RISING);
  TCCR1A = 0;
  TCCR1A = (1 << COM1A1)|(1 << COM1B1)|(1 << WGM11);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13)|(1 << WGM12)|(1 << CS10);
  ICR1 = 2047;
  OCR1A = 0;
  OCR1B = 0;
  lcd.begin(16, 2);
  lcd.setRGB(255, 0, 0);
  //Serial.begin(9600);
  lcd.clear();
  lcd.createChar(0, cursor_char);
  lcd.createChar(1, custom_char);
  lcd.clear();
  welcome_screen();
  delay(4000);
  mode_screen();
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0);
  //test_writing();
}


void loop(){

  /*Turn Detected Switch case*****/
  
  if(TurnDetected){
    delay(200);
    switch(screen){
      case 0:       // General screen
        switch(arrowpos){
          case 0:
            if(!up){
              mode_screen();
              lcd.setCursor(0, 1);
              lcd.write((uint8_t)0);
              arrowpos = 1;
            }
            break;
          case 1:
            if (!up){
              mode_screen();
              lcd.setCursor(8, 1);
              lcd.write((uint8_t)0);
              arrowpos = 2;
            }
            if(up){
              mode_screen();
              lcd.setCursor(0,0);
              lcd.write((uint8_t)0);
              arrowpos = 0;
            }
          break;
          case 2:
            if (up){
              mode_screen();
              lcd.setCursor(0,1);
              lcd.write((uint8_t)0);
              arrowpos = 1;
            }
          break;
        }
      break;
    case 1: // set testing param screen
     switch(arrowpos){
      case 0:
       if(!up){
          set_testing_param_screen();
          lcd.setCursor(0,1);
          lcd.write((uint8_t)0);
          arrowpos = 1;
       }
       break;
     case 1:
       if(up){
          set_testing_param_screen();
          lcd.setCursor(0,0);
          lcd.write((uint8_t)0);
          arrowpos=0;
       }
       else{
         set_testing_param_screen();
         lcd.setCursor(11,1);
         lcd.write((uint8_t)0);
         arrowpos=2;
       }
       break;
     case 2:
      if(up){
        set_testing_param_screen();
        lcd.setCursor(0,1);
        lcd.write((uint8_t)0);
        arrowpos=1;
      }
      break;
     case 3: 
      if(up){
        Ah_rating += 1;
        set_testing_param_screen();
        lcd.setCursor(5,0);
        lcd.print(Ah_rating);
        lcd.print("Ah");
        lcd.write((uint8_t)1);
        lcd.print("   ");
      }
     else{
        Ah_rating -= 1;
        if(Ah_rating  <= 0){
          Ah_rating = 0.0;
        }
        set_testing_param_screen();
        lcd.setCursor(5,0);
        lcd.print(Ah_rating);
        lcd.print("Ah");
        lcd.write((uint8_t)1);
        
      }
     break;
    
     }
   break;
    case 2: // C Mode
      switch(arrowpos){
        case 0:
          if (!up){
            charge_mode_screen();
            lcd.setCursor(0, 1);
            lcd.write((uint8_t)0);
            arrowpos = 1;
          }
          break;
        case 1:
          if(up){
            charge_mode_screen();
            lcd.setCursor(0, 0);
            lcd.write((uint8_t)0);
            arrowpos = 0;
          }
          if (!up){
            charge_mode_screen();
            lcd.setCursor(11, 1);
            lcd.write((uint8_t)0);
            arrowpos = 2;
          }
          break;
        case 2:
          if (up){
            charge_mode_screen();
            lcd.setCursor(0,1);
            lcd.write((uint8_t)0);
            arrowpos = 1;
          }
          break;
        case 3:
          if(up){
            user_current_input = user_current_input + 0.1;
            lcd.setCursor(4,0);
            lcd.print(user_current_input);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          else{
            user_current_input = user_current_input - 0.1;
            if(user_current_input < 0){
              user_current_input = 0;
            }
            lcd.setCursor(4,0);
            lcd.print(user_current_input);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          break;
      }
      break;
   case 3:    // D Mode
    switch(arrowpos){
      case 0:
          if (!up){
            discharge_mode_screen();
            lcd.setCursor(0, 1);
            lcd.write((uint8_t)0);
            arrowpos = 1;
          }
          break;
      case 1:
          if(up){
            discharge_mode_screen();
            lcd.setCursor(0, 0);
            lcd.write((uint8_t)0);
            arrowpos = 0;
          }
          if (!up){
            discharge_mode_screen();
            lcd.setCursor(11, 1);
            lcd.write((uint8_t)0);
            arrowpos = 2;
          }
          break;
      case 2:
          if (up){
            discharge_mode_screen();
            lcd.setCursor(0,1);
            lcd.write((uint8_t)0);
            arrowpos = 1;
          }
          break;
      case 3:
          if(up){
            discharge_current = discharge_current + 0.1;
            lcd.setCursor(4,0);
            lcd.print(discharge_current);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          else{
            discharge_current = discharge_current - 0.1;
            if(discharge_current < 0){
              discharge_current = 0;
            }
            lcd.setCursor(4,0);
            lcd.print(discharge_current);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          break;
      }
   break;

  case 5: // Display charge parameters screen
   break;
  
  
   }
 TurnDetected = false;
 }

 if(button){
  delay(200);
  switch(screen){
    case 0:
     switch(arrowpos){
        case 0:
        screen = 1;
        lcd.clear();
        set_testing_param_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos = 0;
        break;
        case 1:
        screen = 2;
        charge_mode_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
        break;
        case 2:
        screen = 3;
        discharge_mode_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
       break;
     }
    break;
   case 1:
     switch(arrowpos){
       case 0:
        set_testing_param_screen();
        lcd.setCursor(5,0);
        lcd.print(Ah_rating);
        lcd.print("Ah");
        lcd.write((uint8_t)1);
        arrowpos = 3;
        break;
       case 1: // case for testing_param_screen(Start Button)
        charging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        while(true){};   // This is where the charge routine will go
        
        break;
       case 2:
        screen = 0;
        mode_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
        break;
       case 3: // case for the AHR Rating
        set_testing_param_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
        break;
     }
   break;
   case 2:
    switch(arrowpos){
      case 0:
        charge_mode_screen();
        screen =2;
        lcd.setCursor(4,0);
        lcd.print(user_current_input);
        lcd.print("A");
        lcd.write((uint8_t)1);
        arrowpos=3;
        break;
      case 1:
        charging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        while(true){}; // THis is where the charge routine will go
        break;
      case 2:
        mode_screen();
        screen = 0;
        lcd.setCursor(0,1);
        lcd.write((uint8_t)0);
        arrowpos=1;
        break;
      case 3:
        charge_mode_screen();
        screen =2;
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
        break;
    }
    break;
   case 3: // discharge_mode_set_param_start_back
    switch(arrowpos){
      case 0:
        arrowpos=3;
        discharge_mode_screen();
        lcd.setCursor(4,0);
        lcd.print(discharge_current);
        lcd.print("A");
        lcd.write((uint8_t)1);
        break;
      case 1:
        discharging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        while(true){};
        break;
      case 2:
        screen=0;
        mode_screen();
        lcd.setCursor(8,1);
        lcd.write((uint8_t)0);
        arrowpos=2;
        break;
      case 3:
        discharge_mode_screen();
        lcd.setCursor(0,0);
        lcd.write((uint8_t)0);
        arrowpos=0;
        break;
    }
   
   break;
   
   }
   button = false;
  
 }
  
}

void welcome_screen(){
  lcd.clear();
  lcd.setCursor(1, 0); // lcd.setCursor(col, row)
  lcd.print("Battery Tester");
  lcd.setCursor(5,1);
  lcd.print("Welcome");
}

//****************************/
// _Testing Mode
// _C Mode  _D Mode
void mode_screen(){
  lcd.clear();
  lcd.setCursor(1, 0);  // col, row
  lcd.print("Testing Mode");
  lcd.setCursor(1, 1);
  lcd.print("C Mode");
  lcd.setCursor(9, 1);
  lcd.print("D Mode");
}

void set_testing_param_screen(){
  lcd.clear();
  lcd.setCursor(1,0); // col, row
  lcd.print("AHR:");
  lcd.setCursor(1,1);
  lcd.print("Start");
  lcd.setCursor(12,1);
  lcd.print("Back");
}

void charge_mode_screen(){
  // To do : Change configuration to involve a set Current command
  // Set a cursor for CC and set a different arrowpos for changing the value
  // Set a maximum and minimum current charge
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("CC:");
  lcd.setCursor(1,1);
  lcd.print("Start");
  lcd.setCursor(12,1);
  lcd.print("Back");
}

void discharge_mode_screen(){
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("DC:");
  lcd.setCursor(1,1);
  lcd.print("Start");
  lcd.setCursor(12,1);
  lcd.print("Back");
}

void charging_mode_ongoing_screen(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("t:");
  lcd.print("   ");
  lcd.print("min");
  lcd.setCursor(9,0);
  lcd.print("V:");
  lcd.print("    ");
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print("    ");
  lcd.print("A");
  lcd.print(" ");
  lcd.print("CM");
  lcd.setCursor(12, 1);
  lcd.print("Back");
}

void discharging_mode_ongoing_screen(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("t:");
  lcd.print("   ");
  lcd.print("min");
  lcd.setCursor(9,0);
  lcd.print("V:");
  lcd.print("    ");
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print("    ");
  lcd.print("A");
  lcd.print(" ");
  lcd.print("DM");
  lcd.setCursor(12, 1);
  lcd.print("Back");
}

double get_current(){
  delayMicroseconds(200);
  return ((analogRead(Current_Sens_Pin)/1024.0)*(5000/100));
}

double get_discharge_current(){
  delayMicroseconds(200);
  return ((analogRead(Current_SensD_Pin)/1024.0)*(5000/100));
 
}

double get_voltage(){
  delayMicroseconds(200);
  return (map(analogRead(Bat_VSensor_Pin), 0, 1023, 0 ,5)*((100.0+330.0)/100.0));
}

void manage_charging(){
  int counter = 0;
  int seconds = 0;
  int mins = 0;
  if (charge_complete){
    return;
  }
  static int first_call = true;
  static double charge_current = Ah_rating/3.0;
  delayMicroseconds(100);
  if(first_call){
  open_circuit_voltage = (map(analogRead(Bat_VSensor_Pin), 0, 1023, 0 ,5)*((100.0+330.0)/100.0));
  if (open_circuit_voltage > Charged_BatVoltage){
    charge_complete = true;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Battery Charged");
    lcd.setCursor(0,1);
    lcd.print("Voltage at:");
    lcd.print(open_circuit_voltage);
    screen = 0;
 }
  first_call = false;
  return;
  }
 int base_voltage = 0;
 analogWrite(T_base, base_voltage);
 digitalWrite(RELAY_1, HIGH);
 while(!button){
      /*while(get_current() < charge_current){
        base_voltage += 1;
        if (base_voltage > 255){
          base_voltage = 255;
        }
      */  
      analogWrite(T_base, base_voltage);
      delayMicroseconds(100);
      counter++;
      /*
      if(get_voltage() >= float_voltage){
        charge_complete = true;
        break;
      }
      */
      if(counter == 10000){
        seconds +=1;
        counter =0;
        if(seconds == 60){
          mins += 1;
          seconds=0;
          lcd.setCursor(4,0);
          lcd.write(mins);
          lcd.setCursor(12,0);
          lcd.write(get_voltage());
          lcd.setCursor(2,0);
          lcd.write(get_current());     
        }
        
      }
      /*}
      while(charge_current > get_current()){
        base_voltage -= 1;
        if(base_voltage <= 0){
          base_voltage = 0;
       }
     analogWrite(T_base, base_voltage);
     delayMicroseconds(100);
     if(get_voltage() >= float_voltage){
       charge_complete = true;
       break;
      }
     }
     if(charge_complete){
     */
     //digitalWrite(RELAY_1, LOW);
     analogWrite(T_base, 0);
     /*
     charge_complete = true;
     return;
     */
    }
  
   delay(1000);
   digitalWrite(RELAY_1, LOW);
   analogWrite(T_base, 0);
}

void manage_discharging(){
  //Manage the MOSFETS using OCR1A and OCR1B
  double current_current_reading = get_discharge_current();
  int counter = 0;
  bool dual = true;
  delayMicroseconds(100);
  if(discharge_current <= 4.0){
    OCR1B = 0;
    dual = false;
  }
  if (dual && OCR1A > OCR1B){
    OCR1A = OCR1A/2;
    OCR1B = OCR1A;
  }
    counter = 0;
    while (get_discharge_current() < discharge_current){
      delayMicroseconds(100);
      counter++;
      OCR1A += 1;
      if(dual){
      OCR1B += 1;
      }
      if (counter > 5000){
        break;
      }
    }
    counter = 0;
    while (discharge_current < get_discharge_current()){
      delayMicroseconds(100);
      counter++;
      OCR1A = OCR1A -1;
      if (OCR1A = 0){
        OCR1A =0;
      }
      if(dual){
      OCR1B = OCR1B -1;
      if (OCR1B <= 0){
        OCR1B =0;
      }
      }
      if(counter > 5000){
        break;
      }
    }
}

int record_data(String _dataline){
  static bool open_file = true;
  if(open_file){
  if(!SD.begin(CS)){
    return -1;
  }
  datalog = SD.open("data.csv", FILE_WRITE);
  if (datalog){
    open_file = false;
 }
  else {return -1;}
  }
 datalog.println(_dataline);
 return 0;
   
}

bool close_file(){
  if(datalog){
  datalog.close();
  return true;
  }
  return false;
}

void calculate_Ah_rating(bool _usePeukerts){
  if (_usePeukerts){
    
  }
}

void update_battery_status(){
 
  
}


/*
void test_writing(){
  if(!SD.begin(CS)){
    Serial.println("Card Failed");
    return;
  }
  Serial.println("Card initialized");
  datalog = SD.open("data.csv", FILE_WRITE);
  if (datalog){
    for(int i=0; i < 50; i++){
      datalog.println(i);
 }
 datalog.close();
 }
 Serial.println("Finished writing");
 datalog = SD.open("data.csv");
 if (datalog){
  Serial.println("Init successful");
  while(datalog.available()){
    Serial.write(datalog.read());
  }
  datalog.close();
 }
 else{
  Serial.println("error opening the file");
 }
 
}
*/
