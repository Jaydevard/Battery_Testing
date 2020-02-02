#include <Chrono.h>
#include <LightChrono.h>
#include <rgb_lcd.h>

/* Battery Tester Project 
 *  Created on 1/29/2020
 *  Intended for EE MUN Term 5 Project
 */

Chrono current_read_timer;



#define Resistor 500
#define Voltage 5
#define Current_Sens_Pin A0
#define Bat_VSensor_Pin A1
#define RELAY_1 2


void isr0(){

  
}

/**********************************************/
/*Functions*/
void check_battery_voltage(){
  int reading = map(Bat_VSensor_Pin, 0, 1023, 0 ,5);
  Serial.print("voltage= ");
  Serial.print(reading);
  
}


void setup(){

  pinMode(RELAY_1, OUTPUT);
  Serial.begin(9600);
  
}

void loop(){

  
}
