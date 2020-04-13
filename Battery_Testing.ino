

/* Battery Tester Project 
 *  Created on 1/29/2020
 *  Intended for EE MUN Term 5 Project
 *  Author: Jaydev Madub 
 */
#include <rgb_lcd.h>
#include <SD.h>
#include <SPI.h>
#include <Chrono.h>
#include <curveFitting.h>
#define Current_Sens_Pin A0
#define Bat_VSensor_Pin A1
#define Current_SensD_Pin A3
#define Charged_BatVoltage 12.5
#define MAX_CC 4.0
#define MAX_DC 8.0
#define CC_REG_OFFSET 2.48*1000
#define DC_REG_OFFSET 2.48*1000
/*RELAYS***************************/
#define C_RELAY 7
#define D_RELAY 8
/*MOSFETS**************************/
#define MOSF1 9
#define MOSF2 5
/*SD Card**************************/
#define MOSI 11
#define MISO 12
#define CLK 13
#define CS 6
/*OUTPUT CONTROL*******************/
#define T_base 10 // PWM Pin
/*OTHER CONSTS AND OBJECTS*********/
double battery_voltage = 0.0;
const double peukerts_const = 1.4;
int C_rating = 20;
int Ah_rating = 7;
double open_circuit_voltage = 0;
double discharge_current = 0.0;
double charge_current = 0.0;
int charge_ah_rating = 0;
bool charge_complete = false;
bool discharge_complete = false;
bool test_mode = false;
long int counter = 0;
bool reset_data = false;
double internal_res = 0;

rgb_lcd lcd;
Chrono Timer1;
Chrono Timer2;
Chrono Timer3(Chrono::SECONDS);
/*ROTARY ENCODER*******************/
#define clk 2
#define dt 3
#define sw 4
/*HANDLING*************************/
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
void update_battery_status();   // get the battery status
double calculate_ah_rating(double d_current);   // calculate the Amp Hr rating
/*Helper Functions****************/
double get_current();           
double get_voltage();
double get_discharge_current();
void log_data(bool close_file=false);
void adjust_discharge_current(bool _dual, double _target_current, bool _reset=false);
void adjust_charge_current(bool _reset, double target_current);
void update_battery_status(char _mode, int _counter);
/**Screens********************************/    
void welcome_screen();
void mode_screen();
void set_testing_param_screen();
void charge_mode_screen();
void discharge_mode_screen();
void charging_mode_ongoing_screen();
void discharging_mode_ongoing_screen();

File check;
File reader;

void setup(){
  pinMode(C_RELAY, OUTPUT);
  pinMode(D_RELAY, OUTPUT);
  digitalWrite(C_RELAY, LOW);
  digitalWrite(D_RELAY, LOW);
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
  TCCR0B = TCCR0B & B11111000 | B00000011;
  attachInterrupt(0, isr0, RISING);
  lcd.begin(16, 2);
  lcd.setRGB(255, 0, 0);
  lcd.clear();
  lcd.createChar(0, cursor_char);
  lcd.createChar(1, custom_char);
  lcd.clear();
  Serial.begin(9600);
  welcome_screen();
  delay(4000);
  if(!SD.begin(CS)){
    lcd.setCursor(1,0);
    lcd.print("SD Fail");     
  }
  //SD.remove("log.txt");
  //check = SD.open("log.txt", FILE_WRITE);
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
        Ah_rating += 10;
        set_testing_param_screen();
        lcd.setCursor(5,0);
        lcd.print(Ah_rating);
        lcd.print("Ah");
        lcd.write((uint8_t)1);
        lcd.print("   ");
      }
     else{
        Ah_rating -= 5;
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
            charge_ah_rating = charge_ah_rating + 10;
            lcd.setCursor(5,0);
            lcd.print(charge_ah_rating);
            lcd.print("AH");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          else{
            charge_ah_rating = charge_ah_rating - 5;
            if(charge_ah_rating < 0){
              charge_ah_rating = 0;
            }
            lcd.setCursor(5,0);
            lcd.print(charge_ah_rating);
            lcd.print("AH");
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
            discharge_current = discharge_current + 0.5;
            if(discharge_current >= 8.0){
              discharge_current = 8.0;
            }
            lcd.setCursor(4,0);
            lcd.print(discharge_current);
            lcd.print("A");
            lcd.write((uint8_t)1);
            lcd.print("   ");
          }
          else{
            discharge_current = discharge_current - 0.5;
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
       {
        test_mode = true;
        
        counter =0;
        double current = 0;
        charging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        if(test_mode){
          current = Ah_rating/6.0;
        }
        if(current > MAX_CC){
          current = MAX_CC;
        }
        TCCR1B = TCCR1B & B11111000 | B00000001;
        Timer1.start();
        Timer2.start();
        Timer3.start();
        double previous_voltage_val = 0;
        bool charge_complete = false;
        digitalWrite(C_RELAY, HIGH);
        adjust_charge_current(true, current);
        open_circuit_voltage = get_voltage();
        if(open_circuit_voltage > Charged_BatVoltage){
          charge_complete = true;
        }
        bool set_voltage = false;
        bool break_out = false;
        button = false;
        charge_current = current;
        while(true){
          if(Timer1.hasPassed(100)){
            Timer1.restart();
            adjust_charge_current(false, current);
            
          }
         if(Timer2.hasPassed(1001)){
            Timer2.restart();
            counter++;
            update_battery_status('c', counter);
         }
          if(Timer3.hasPassed(32.5) && set_voltage){
            previous_voltage_val = get_voltage();
            set_voltage = false;
          }
          if(Timer3.hasPassed(120.5)){
            if(get_voltage() - previous_voltage_val < 0.05){
              current -= 0.2;
              charge_current = current;
            }
            if(current <=0){
              current = 0;
              charge_current = 0;
              charge_complete = true;
            }
            Timer3.restart();
            set_voltage = true; 
          }
          if(button){
            button = false;
            TurnDetected = false;
            Timer1.stop();
            Timer2.stop();
            digitalWrite(C_RELAY, LOW);
            while(button == false){
              delay(10);
              if(TurnDetected){
                screen =2;
                delay(1000);
                break_out = true;
                break;
              }
              if(button){
                Timer1.resume();
                Timer2.resume();
                digitalWrite(C_RELAY, HIGH);
                button = false;
                break;
              }
            }
           }
          if(break_out){
            break;
          }
         if(charge_complete){
          Timer1.stop();
          Timer2.stop();
          Timer3.stop();
          analogWrite(T_base,0);
          delay(2000);
          digitalWrite(C_RELAY, LOW);
          lcd.clear();
          lcd.setCursor(4,0);
          lcd.print("Bat Charged");
          lcd.setCursor(0,1);
          lcd.print("BatV:");
          lcd.print(get_voltage());
          delay(2000);
          break;
         }
        }   // This is where the charge routine will go
        if(break_out){
          break;
        }
        
        double d_current = 0;
        TCCR1B = TCCR1B & B11111000 | B00000011;
        break_out = false;
        bool dual = false;
        bool first_read = true;
        double first_voltage = 0;
        double last_voltage = 0;
        lcd.write((uint8_t)0);
        counter = 0;
        if(test_mode){
          d_current = Ah_rating/3.0;
        }
        else{
          d_current = discharge_current;
        }
        if(d_current > 4){
          dual = true;
        }
        if(d_current > 8){
            d_current = 8;
          }
        discharge_complete = false;
        open_circuit_voltage = get_voltage();
        digitalWrite(C_RELAY, LOW);
        delayMicroseconds(100);
        digitalWrite(D_RELAY, HIGH);
        lcd.clear();
        lcd.setCursor(2,0);
        lcd.print("Setting");
        lcd.setCursor(2,1);
        lcd.print("Discharge");
        internal_res = calculate_internal_res(d_current);
        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print("Int Res:");
        lcd.setCursor(2,1);
        lcd.print(internal_res);
        reset_data = true;
        double target_discharge_voltage = 11.0;
        if(d_current > 3){
          target_discharge_voltage = 11.0;
        }
        else if(d_current > 1.5){
          target_discharge_voltage = 11.50;
        }
        else{
          target_discharge_voltage = 11.7;
        }
        delay(2000);
        discharging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        Timer1.start();
        Timer2.start();
        Timer3.start();
        button = false;
        String data = "";
        adjust_discharge_current(dual, d_current, true);
        int smcount = 1;
        TurnDetected = false;
        while(true){ 
          if(Timer1.hasPassed(10)){
            Timer1.restart();
            adjust_discharge_current(dual, d_current);
          }
         if(Timer2.hasPassed(1000)){
            Timer2.restart();
            counter++;
            smcount = 1;
            update_battery_status('d', counter);
          }
          if(Timer3.hasPassed(30)){
            if(first_read){
              first_voltage = get_voltage();
              first_read = false;
            }
            log_data();
            last_voltage = get_voltage();
            if(get_voltage() - 0.1 < target_discharge_voltage){
              discharge_complete = true;
            }
            Timer3.restart();
          }
          if(button){
            button = false;
            TurnDetected = false;
            Timer1.stop();
            Timer2.stop();
            Timer3.stop();
            digitalWrite(D_RELAY, LOW);
            while(button == false){
              delay(10);
              if(TurnDetected){
                screen =1;
                delay(200);
                break_out = true;
                break;
              }
              if(button){
                Timer1.resume();
                Timer2.resume();
                Timer3.resume();
                digitalWrite(D_RELAY, HIGH);
                button = false;
                break;
              }
            }
           }
          
          if(break_out){
            break;
          }
          if(discharge_complete){
            Timer1.stop();
            Timer2.stop();
            Timer3.stop();
            analogWrite(MOSF1, 0);
            delayMicroseconds(100);
            analogWrite(MOSF2, 0);
            delay(2000);
            
            log_data(true);
            digitalWrite(D_RELAY, LOW);
            delay(1000);
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("Discharge");
            lcd.setCursor(3,1);
            lcd.print("Complete");
            delay(3000);
            double ahrating = calculate_ah_rating(d_current, counter,first_voltage, last_voltage);
            lcd.clear();
            lcd.setCursor(3,0);
            lcd.print("Test");
            lcd.setCursor(2,1);
            lcd.print("Complete");
            delay(3000);
            lcd.clear();
            lcd.setCursor(2,0);
            lcd.print("AH Rating:");
            lcd.setCursor(3,1);
            lcd.print(ahrating);
            while(!button);
            button = false;
            screen = 1;
            TurnDetected = true;
            break;
          }
          };
       }
    
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
        lcd.setCursor(5,0);
        lcd.print(charge_ah_rating);
        lcd.print("AH");
        lcd.write((uint8_t)1);
        arrowpos=3;
        break;
      case 1:
      { 
        test_mode = false;
        double current = 0;
        int counter = 0;
        double previous_voltage_val = 0;
        charging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        TCCR1B = TCCR1B & B11111000 | B00000001;
        current = charge_ah_rating/6.0;
        if(current > MAX_CC){
          current = MAX_CC;
        }
        Timer1.start();
        Timer2.start();
        Timer3.start();
        digitalWrite(C_RELAY, HIGH);
        adjust_charge_current(true, current);
        charge_current = current;
        bool break_out = false;
        bool set_voltage = true;
        button = false;
        while(true){
          if(Timer1.hasPassed(100)){
            Timer1.restart();
            adjust_charge_current(false, current);
          }
          if(Timer2.hasPassed(1001)){
            Timer2.restart();
            counter++;
            update_battery_status('c', counter);
          }
           if(Timer3.hasPassed(30.5) && set_voltage){
            previous_voltage_val = get_voltage();
            set_voltage = false;
          }
          if(Timer3.hasPassed(120.5)){
            if(get_voltage() - previous_voltage_val < 0.05){
              current -= 0.2;
              charge_current = current;
            }
            if(current <=0){
              current = 0;
              charge_current = 0;
              charge_complete = true;
            }
            Timer3.restart();
            set_voltage = true; 
          }
         
         if(button){
            button = false;
            TurnDetected = false;
            Timer1.stop();
            Timer2.stop();
            Timer3.stop();
            digitalWrite(C_RELAY, LOW);
            while(button == false){
              if(TurnDetected){
                screen =2;
                delay(200);
                break_out = true;
                break;
              }
              if(button){
                Timer1.resume();
                Timer2.resume();
                Timer3.resume();
                digitalWrite(C_RELAY, HIGH);
                button = false;
                break;
              }
            }
           }
          if(break_out){
            break;
          }
          if(charge_complete){
            Timer1.stop();
            Timer2.stop();
            Timer3.stop();
            analogWrite(T_base,0);
            delay(2000);
            digitalWrite(C_RELAY, LOW);
            lcd.clear();
            lcd.setCursor(4,0);
            lcd.print("Bat Charged");
            lcd.setCursor(0,1);
            lcd.print("BatV:");
            lcd.print(get_voltage());
            delay(2000);
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
        test_mode = false;
        double current = 0;
        bool break_out = false;
        bool dual = false;
        TCCR1B = TCCR1B & B11111000 | B00000011;
        counter = 0;
        current = discharge_current;
        if(current > 4){
          dual = true;
        }
        if(current > 8){
            current = 8.0;
          }
        double internal_res_current = 0.5;
        open_circuit_voltage = get_voltage();
        double internal_res = 0;  
        digitalWrite(C_RELAY, LOW);
        delayMicroseconds(100);
        digitalWrite(D_RELAY, HIGH);
        lcd.clear();
        lcd.setCursor(2,0);
        lcd.print("Setting..");
        lcd.setCursor(2,1);
        lcd.print("Discharge");
        double target_discharge_voltage = 11.0;
        if(current > 3){
          target_discharge_voltage = 11.0;
        }
        else if(current > 1.5){
          target_discharge_voltage = 11.5;
        }
        else{
          target_discharge_voltage = 11.7;
        }
        delay(2000);
        discharging_mode_ongoing_screen();
        lcd.setCursor(11,1);
        lcd.write((uint8_t)0);
        Timer1.start();
        Timer2.start();
        Timer3.start();
        button = false;
        String data = "";
        adjust_discharge_current(dual,current, true);
        int smcount = 1;
        TurnDetected = false;
        while(true){ 
          if(Timer1.hasPassed(10)){
            Timer1.restart();
            adjust_discharge_current(dual, current);
          }
          if(Timer2.hasPassed(1000)){
            Timer2.restart();
            update_battery_status('d', counter);
            counter++;
            
           }
          if(button){
            button = false;
            TurnDetected = false;
            Timer1.stop();
            Timer2.stop();
            digitalWrite(D_RELAY, LOW);
            while(button == false){
              delay(10);
              if(TurnDetected){
                screen =3;
                delay(200);
                break_out = true;
                break;
              }
              if(button){
                Timer1.resume();
                Timer2.resume();
                digitalWrite(D_RELAY, HIGH);
                button = false;
                break;
              }
            }
           }
          if(break_out){
            break;
          }
          if(Timer3.hasPassed(32.5)){
            Timer3.restart();
            if(get_voltage() < target_discharge_voltage){
              Timer1.stop();
              Timer2.stop();
              Timer3.stop();
              digitalWrite(D_RELAY, LOW);
              lcd.clear();
              lcd.setCursor(2,0);
              lcd.print("Discharge");
              lcd.setCursor(3,1);
              lcd.print("Complete");
              delay(500);
              screen = 3;
              TurnDetected = true;
              break;
            }
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
  lcd.print("AHR:");
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
  lcd.setCursor(9,0);
  lcd.print("V:");
  lcd.print("    ");
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print("    ");
  lcd.print("A");
  lcd.print("  ");
  lcd.print("C");
  lcd.setCursor(12, 1);
  lcd.print("Back");
}

void discharging_mode_ongoing_screen(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("t:");
  lcd.print("   ");
  lcd.setCursor(9,0);
  lcd.print("V:");
  lcd.print("    ");
  lcd.print("V");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print("    ");
  lcd.print("A");
  lcd.print("  ");
  lcd.print("D");
  lcd.setCursor(12, 1);
  lcd.print("Back");
}


// Function to get the current
double get_current(){
  double current = ((analogRead(Current_Sens_Pin)*5000.0/1023.00)-CC_REG_OFFSET)/100;
  return current;
}

// Function to get the discharge current
double get_discharge_current(){
  double current = ((analogRead(Current_SensD_Pin)*5000.0/1023.00)-DC_REG_OFFSET)/100;
  current = current - 0.35; // calibration offset
  return current;
 
}

// Function to get the battery voltage
double get_voltage(){
  double voltage = ((5.0*analogRead(Bat_VSensor_Pin))/1023.0)*3.04;
  return (voltage);
}

// Function to log the data to SD card every 30 second
void log_data(bool close_file = false){
  if(close_file){
    check.close();
    return;
  }
  String data = "";
  static bool starting = true;
  if(starting or reset_data){
    check.seek(0);
    starting = false;
    reset_data = false;
  }
  data += String(counter);
  data += ",";
  data += String(get_voltage());
  data += ";";
  if(check){
    check.println(data);
    Serial.println(data);
    data = "";
    
  }
  else{
    return -1;
  }
  
}

// parses the csv fields obtained from each dataline
String parsecsv(String& input, int& start_pos, char delim ){
  
  if(start_pos == -1){
    return "";
  }
  int _start = start_pos;
  int delim_pos = input.indexOf(delim, _start);
  if(delim_pos == -1){
    start_pos = delim_pos;
    return input.substring(_start);
  }
  else{
    start_pos = delim_pos+1;
    return input.substring(_start, delim_pos);
  }
}

// calculates the internal resistance of the battery
double calculate_internal_res(double d_current){
  delay(3000);
  while(true){
    adjust_discharge_current(true, d_current);
    delay(10);
    if(get_discharge_current() - d_current > 0.1){
      break;
    }
  }
  delay(4000);
  double term_voltage = get_voltage();
  double internal_res = (12.0 - term_voltage)/d_current;
  return internal_res;
}



// helper function to update the LCD screen every predetermined time 
void update_battery_status(char _mode, int _counter){
  double current = 0.0;
  if (_mode == 'd'){
  current = get_discharge_current();
  delayMicroseconds(200);
  }
  else{
  current = charge_current;
  delayMicroseconds(200);
  
  }
  double voltage = get_voltage();
  if(_mode == 'c'){
    if(charge_current > 1){
      voltage = voltage - 0.95; // voltage for charging
    }
    else{
      voltage = voltage - 0.65;  // voltage for charging
    }
  }
  delayMicroseconds(200);
  lcd.setCursor(2,0);
  lcd.print(_counter);
  lcd.print("sec");
  lcd.setCursor(11,0);
  lcd.print(voltage);
  lcd.setCursor(2,1);
  if(_mode == 'd'){
  lcd.print("       D");
  }
  else{
  lcd.print("       C");
  }
  lcd.setCursor(2,1);
  lcd.print(current);
  lcd.print("A");
  
 
}

// Function to help keep the discharge current constant
void adjust_discharge_current(bool _dual, double _target_current, bool _reset=false){
  static int PWM_9 = 0;
  static int PWM_5 = 0;
  if(_reset){
    PWM_9 = false;
    PWM_5 = false;
    analogWrite(MOSF1,PWM_9);
    delayMicroseconds(20);
    analogWrite(MOSF2, PWM_5);
    delayMicroseconds(20);
    return;
  }
  if(_dual){
    if((get_discharge_current() - _target_current) < -0.1){
       PWM_9++;
       PWM_5++;
       if(PWM_9 >= 255){
        PWM_9 = 255;
       }
       if(PWM_5 >=255){
        PWM_5 = 255;
       }
       analogWrite(MOSF1, PWM_9);
       delayMicroseconds(20);
       analogWrite(MOSF2, PWM_5);
       delayMicroseconds(20);
      }
    else if((get_discharge_current() - _target_current) > 0.1){
       PWM_9--;
       PWM_5--;
       if(PWM_5 == 0){
        PWM_5 = 0;
       }
       if(PWM_5 == 0){
        PWM_5 = 0;
       }
       analogWrite(MOSF1, PWM_9);
       delayMicroseconds(20);
       analogWrite(MOSF2, PWM_5);
       delayMicroseconds(20);
     }
      }
    else{
      if((get_discharge_current()-_target_current) < -0.1){
      PWM_9++;
      if(PWM_9 >=255){
        PWM_9 = 255;
      }
      analogWrite(MOSF1, PWM_9);
      delayMicroseconds(20);
      }
      else if((get_discharge_current() - _target_current) > 0.1){
      PWM_9--;
      if(PWM_9 == 0){
        PWM_9 = 0;
      }
      analogWrite(MOSF1, PWM_9);
      delayMicroseconds(20);
      }
   }
     
}


// Function to help keep the charging current constant
void adjust_charge_current(bool _reset, double target_current){
  static int PWM_10 = 0;
  if(_reset || target_current == 0.0){
    PWM_10 = 0;
    analogWrite(T_base, PWM_10);
    return;
  }
 PWM_10 = 1.19*(pow(target_current,2))+41.35*(target_current)-4.81; // calibration equation for charge current
 analogWrite(T_base, PWM_10);
}


// Function to calculate the Amp Hour Rating of the battery
double calculate_ah_rating( double d_current, int last_counter, double first_voltage, double last_voltage){
  double t[2] = {29, last_counter};
  double v[2] = {first_voltage, last_voltage};
  double multiplier = (12 - first_voltage)+1.25;
  double bat_voltage = 11.0 - (d_current*internal_res);
  int orderOfEqn = 1;
  double coeffs[orderOfEqn+1];
  int ret = fitCurve(orderOfEqn, sizeof(t)/sizeof(double), v, t, sizeof(coeffs)/sizeof(double), coeffs);
  if(ret == 0){
    double voltage_cutoff_time = coeffs[0]*bat_voltage + coeffs[1];
    double ah_rating = pow(((voltage_cutoff_time/3600.0)/(C_rating*1.0)), (1/peukerts_const))*(d_current*(C_rating*1.0));
    return ah_rating*multiplier;
  }
};
 



  


  
    
