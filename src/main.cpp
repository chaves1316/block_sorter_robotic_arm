#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include "commands.h"
#include <Servo.h>

////////////// DISTANCE SENSOR Variables   ////////
VL53L0X tof;
float distance, prev_distance;

////////////// COLOR SENSOR Variables  ////////
double RED,GREEN,BLUE;
commands_t serial_commands;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

/////////////// State Machines ////////////////

typedef struct {
  bool dir;  //direction 
  unsigned long pos;
} motors;

typedef struct {
  int state, new_state;
  unsigned long tes, tis;
} fsm_t;

motors MB, MR, MH, MC; // MB - Motor Base; MR - Motor Reach; MH - Motor Height; MC - Motor Catch
fsm_t fsm;

///////////////// Update States ///////////////

void update_state(fsm_t & fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis resets
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

////////////// Set Servos ////////
Servo M_base,M_reach,M_height,M_catch;


///////////// Global Use Variables ////////////////////

unsigned long interval;
unsigned long currentMillis, previousMillis;
int loop_count;
double mean_dist, dist_to_reach;    /// min dist = 0.09m    max dist = 0.25m
bool start_activity, red, green, yellow, blue;

void calc_next_state(){
  if (start_activity == false){
    fsm.new_state = 0;
  } else if (fsm.state == 0 && start_activity == true){
    fsm.new_state = 1;
  } else if (fsm.state == 1 && distance < 0.180){ //o outro código que estava a usar antes: (mean_dist < 0.040 && mean_dist > 0.010)
    dist_to_reach = distance;
    MB.pos = MB.pos - 7; // o -10 está aqui pq o detetor mal vê que tem um objeto perto faz com que o motor da base pare imediatamente fazendo com que o braço fique ligeiramente desalinhado com a peça, então o -10 compensa esse desalinhamento
    MH.pos = 40;
    fsm.new_state = 2;
  } else if (fsm.state == 2 && (MR.pos >= dist_to_reach * 650)){ //825 obtido parando o braço quando ele estava na posição de apanhar uma peça (é a constante de proporcionalidade que relaciona a posição do motor que controla o reach e a distância a que está a peça)
    fsm.new_state = 3;
    dist_to_reach = 1;
  } else if (fsm.state == 3 && MC.pos == 100){
    if (RED > GREEN && RED > BLUE && GREEN < 1.5*BLUE) {
      red = true;
      green = false;
      yellow = false;
      blue = false;
    } else if (GREEN > RED && GREEN > BLUE) {
      red = false;
      green = true;
      yellow = false;
      blue = false;
    } else if (BLUE > RED && BLUE > GREEN) {
      red = false;
      green = false;
      yellow = false;
      blue = true;
    } else if (RED > GREEN && GREEN > BLUE) {
      red = false;
      green = false;
      yellow = true;
      blue = false;
    }
    fsm.new_state = 4;
  } else if (fsm.state == 4 && (MH.pos >= 120 && MR.pos <= 60)){
    fsm.new_state = 5;
  } else if (fsm.state == 5 && red == true && MB.pos <= 10){ //para o bloco vermelho
    fsm.new_state = 6;
  } else if (fsm.state == 6 && MH.pos <= 40 && MR.pos >= 80){
    fsm.new_state = 7;
  } else if (fsm.state == 7 && MH.pos == 120 && MR.pos == 100 && MC.pos == 0){
    fsm.new_state = 1;
  }
}

void calc_outputs(){

if (fsm.state == 0){ //estado em que o braço inicia
  MR.pos = 100;
  MB.pos = 140;
  MC.pos = 0;
  MH.pos = 120;

} else if (fsm.state == 1){ //estado em que o braço está a fazer scan para encontrar peças
  if (MB.pos <= 40){
    MB.dir = true;
  } else if(MB.pos >= 140){
    MB.dir = false;
  }
  
  if (MB.dir == true){
    MB.pos ++;
  } else if (MB.dir == false){
    MB.pos --;
  }

} else if (fsm.state == 2){ //estado em que o braço busca a peça (base fixa, altura e reach aumentam aos poucos para não se fazer movimentos bruscos e alterar a posição da peça)
  MR.pos ++;
  MH.pos ++;

} else if (fsm.state == 3){ //depois do reach atingir a posição definida, temos que fechar a garra 
  MC.pos ++;

} else if (fsm.state == 4){ //depois da garra fechar temos que levantar o braço
  MH.pos ++;
  MR.pos --;
  if (MH.pos > 120){
    MH.pos --;
  } else if (MR.pos < 60){
    MR.pos ++;
  }

} else if (fsm.state == 5){ //levar a peça para a posição que queremos rodando a base
  MB.pos --;
  
} else if (fsm.state == 6){ //ao chegar à posição predefinida para aquela cor, vamos baixar o braço
  MH.pos --;
  MR.pos ++;
  if (MH.pos < 40){
    MH.pos ++;
  } else if (MR.pos > 80){
    MR.pos --;
  }

} else if (fsm.state == 7){ //largamos a peça e volta tudo ao inicio
  MC.pos = 0;
  MH.pos ++;
  MR.pos ++;  
  if (MH.pos > 120){
    MH.pos --;
  } else if (MR.pos > 100){
    MR.pos --;
  }
}
}

void update_outputs(){
  M_base.write(MB.pos);
  M_reach.write(MR.pos);
  M_catch.write(MC.pos);
  M_height.write(MH.pos);
}



void setup() 
{ 
  M_base.attach(0, 544, 2344); // range 0(looking left) - 180(looking right)
  M_reach.attach(2, 544, 2344); // range 35(retracted) - 170(MH.dired)
  M_catch.attach(4, 544, 2344); // range 0(open) - 100(close)
  M_height.attach(6, 544, 2344); // range 40 (low) - 160 (high)
  
  /// defining max range 
  // MR.pos = 170;
  // MH.pos = 120;
  // MB.pos = 90;
  // MC.pos = 0;

  /// defining min range 
  // MR.pos = 100;
  // MH.pos = 20;
  // MB.pos = 90;
  // MC.pos = 0;
  
  // setting start state
  update_state(fsm, 0);

  // set activity to false (dont do things until told to)
  start_activity = false;

  interval = 40;
  loop_count = 0;

  Serial.begin(115200);

  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.begin();

  tof.setAddress(0x53);
  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  
  // Start new distance measure
  tof.startReadRangeMillimeters();  

  while (!tcs.begin()) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }

}

void loop() 
{
  
  // use serial to make the robot start measuring

  uint8_t b;
  if (Serial.available()) { 
    b = Serial.read();       
    if (b == '+') start_activity = true;  
    if (b == '-') start_activity = false;  
  }  

  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
    }

    uint16_t r, g, b, c;
    getRawData_noDelay(&r, &g, &b, &c);

    if (loop_count < 5){
      mean_dist += distance/5;
      RED += r/5;
      GREEN += g/5;
      BLUE += b/5;
      loop_count ++;
    } else {
      loop_count = 0;
      mean_dist = 0;
      RED = 0;
      GREEN = 0;
      BLUE = 0;
    }

    //Serial.print(" Dist: ");Serial.print(distance, 3);Serial.print(" ");

    if (loop_count == 4){
      //Serial.print(" start activity: ");Serial.print(start_activity);Serial.print(" ");

      Serial.print(" Dist: ");Serial.print(distance, 3);Serial.print(" ");
      //Serial.print(" mean_dist: ");Serial.print(mean_dist, 3);Serial.print(" ");
      //Serial.print(" dist_to_reach: ");Serial.print(dist_to_reach, 3);Serial.print(" ");

      Serial.print(" state: ");Serial.print(fsm.state);Serial.print(" ");

      Serial.print(" reach: ");Serial.print(MR.pos);Serial.print(" ");
      Serial.print(" height: ");Serial.print(MH.pos);Serial.print(" ");
      Serial.print(" catch: ");Serial.print(MC.pos);Serial.print(" ");
      Serial.print(" base: ");Serial.print(MB.pos);Serial.print(" ");

      if (red == true){
        Serial.print(" Color detected: Red " );
      } else if (green == true){
        Serial.print(" Color detected: Green ");
      } else if (yellow == true){
        Serial.print(" Color detected: Yellow ");
      } else if (blue == true){
        Serial.print(" Color detected: Blue ");
      }

      // Serial.print(" red: ");Serial.print(red);Serial.print(" ");
      // Serial.print(" green: ");Serial.print(green);Serial.print(" ");
      // Serial.print(" yellow: ");Serial.print(yellow);Serial.print(" ");
      // Serial.print(" blue: ");Serial.print(blue);Serial.print(" ");

      // if (RED > GREEN && RED > BLUE && GREEN < 1.5*BLUE) {
      //   Serial.print(" Color detected: Red " );
      // } else if (GREEN > RED && GREEN > BLUE) {
      //   Serial.print(" Color detected: Green ");
      // } else if (BLUE > RED && BLUE > GREEN) {
      //   Serial.print(" Color detected: Blue ");
      // } else if (RED > GREEN && GREEN > BLUE) {
      //   Serial.print(" Color detected: Yellow ");
      // } else {
      //   Serial.print(" Unable to identify color ");
      // }
      // Serial.print(" R: "); Serial.print(RED, 0); Serial.print(" ");
      // Serial.print(" G: "); Serial.print(GREEN, 0); Serial.print(" ");
      // Serial.print(" B: "); Serial.print(BLUE, 0); Serial.print(" ");

      Serial.println();
    }

    calc_next_state();

    update_state(fsm, fsm.new_state);

    calc_outputs();

    update_outputs();

    // Start new distance measure
    tof.startReadRangeMillimeters(); 
  }
}