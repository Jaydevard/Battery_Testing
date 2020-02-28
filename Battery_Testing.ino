

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
#define MAX_CC 4.5
#define MAX_DC 8
/*RELAYS***************************/
#define C_RELAY 1
#define D_RELAY 8
#define M_RELAY 7
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
double charge_current = 0.0;
bool charge_complete = false;
bool discharge_complete = false;
bool test_mode = false;
long int counter = 0;

rgb_lcd lcd;
File datalog;
Chrono Timer1(Chrono::MICROS);
Chrono Timer2(Chrono::SECONDS);
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
  pinMode(C_RELAY, OUTPUT);
  pinMode(D_RELAY, OUTPUT);
  pinMode(M_RELAY, OUTPUT);
  digitalWrite(C_RELAY, LOW);
  digitalWrite(D_RELAY, LOW);
  digitalWrite(M_RELAY, LOW);
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
            charge_current = charge_current + 0.1;
            lcd.setCursor(4,0);
            lcd.print(charge_current);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          else{
            charge_current = charge_current - 0.1;
            if(charge_current < 0){
              charge_current = 0;
            }
            lcd.setCursor(4,0);
            lcd.print(charge_current);
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
        lcd.print(charge_current);
        lcd.print("A");
        lcd.write((uint8_t)1);
        arrowpos=3;
        break;
      case 1:
      { 
        double current = 0;
        int counter = 0;
        charging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        if(test_mode){
          current = Ah_rating/8;
        }
        if(current > MAX_CC){
          current = MAX_CC;
        }
        Timer1.start();
        Timer2.start();
        digitalWrite(C_RELAY, HIGH);
        adjust_charge_current(true, current);
        button = false;
        while(true){
          if(Timer1.hasPassed(100)){
            Timer1.restart();
            adjust_charge_current(false, current);
          }
          if(Timer2.hasPassed(1)){
            Timer2.restart();
            counter++;
            update_battery_status('c', counter);
          }
          if(button){
            digitalWrite(C_RELAY, LOW);
            Timer1.stop();
            Timer2.stop();
            screen =2;
            delay(1000);
            break;
          }
          
        };   // This is where the charge routine will go
       }
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
      {
        discharging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        double current = 0;
        bool dual = false;
        lcd.write((uint8_t)0);
        counter = 0;
        if(test_mode){
          current = Ah_rating/3;
          
        }
        else{
          current = discharge_current;
        }
        if(current > 4){
          dual = true;
        }
        if(current > 8){
            current = 8;
          }
        Timer1.start();
        Timer2.start();
        digitalWrite(D_RELAY, HIGH);
        button = false;
        while(true){
          if(Timer1.hasPassed(100)){
            Timer1.restart();
            //adjust_discharge_current(dual, current);
          }
          if(Timer2.hasPassed(1)){
            Timer2.restart();
            counter ++;
            update_battery_status('d', counter);
            
          }
          if(button){
            Timer1.stop();
            Timer2.stop();
            digitalWrite(D_RELAY, LOW);
            delay(1000);
            screen =3;
            break;
          }
          };
      }
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

void update_battery_status(char _mode, int _counter){
  double current = 0;
  if (_mode == 'd'){
  current = get_discharge_current();
  }
  else{
  current = get_current();
  
  }
  double voltage = get_voltage();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("t:");
  lcd.print(_counter);
  lcd.print("sec");
  lcd.setCursor(9,0);
  lcd.print("V:");
  lcd.print(voltage);
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print(current);
  lcd.print("A");
  lcd.print(" ");
  if(_mode == 'd'){
    lcd.print("DM"); 
  }
  else{
    lcd.print("CM");
  }
  lcd.setCursor(11,1);
  lcd.write((uint8_t)0);
  lcd.print("Back");
}

void adjust_discharge_current(bool _dual, double _target_current){
  if(_dual){
    if(get_discharge_current() < _target_current){
       OCR1A +=1;
       OCR1B +=1;
      }
    else{
       OCR1A -= 1;
       OCR1B -= 1;
     }
      }
    else{
      if(get_discharge_current() < _target_current){
      OCR1A +=1;
      }
      else{
      OCR1A -=1;
      }
   }
     
}


void adjust_charge_current(bool _reset, double target_current){
  static int PWM = 0;
  if(_reset){
    PWM = 0;
    analogWrite(T_base, PWM);
    return;
  }
  double current = get_current();
  if(current < target_current){
    PWM++;
    if(PWM >= 255){
      PWM = 255;
    }
  analogWrite(T_base, PWM);
  }
  else{
    PWM--;
    if(PWM <= 0){
      PWM = 0;
    }
   analogWrite(T_base, PWM);
  }
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
