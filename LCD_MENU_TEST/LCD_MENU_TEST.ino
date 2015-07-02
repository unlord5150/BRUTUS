//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

//#define PIN_INPUT 0
//#define RELAY_PIN 6

//Define Variables we'll be connecting to
double MTsetpoint, MTInput, MTOutput, HLTsetpoint, HLTInput, HLTOutput;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID MTPID(&MTInput, &MTOutput, &MTsetpoint, Kp, Ki, Kd, DIRECT);
PID HLTPID(&HLTInput, &HLTOutput, &HLTsetpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

int ISRpin = 2;
int latchPin = 5; //set the pinouts
int clockPin = 6;
int dataPin = 4;
int dataPinIN = 7;
int loadPin = 11;
int clockPinIN = 12;
int menuselectCol[] = {0,10,0,10,15};
int menuselectRow[] = {1,1,2,2,3};
int valveconCol[] = {0,8,0,8,0,8,15};
int valveconRow[] = {1,1,2,2,3,3,3};
int manconCol[] = {0,10,0,10,0,15};
int manconRow[] = {1,1,2,2,3,3};
int settempCol[] = {0,0,15,13,14};
int settempRow[] = {1,2,3,1,2};

int encoder0PinA = 2;
int encoder0PinB = 9;
int encoder0PinC = 10;
int encoder0Pos = 0;
int encoder0PinALast = HIGH;
int n = HIGH;
int k;
int l;
int m;
int encoderDir = 0;
int MTPIDOUT = LOW;
int HLTPIDOUT = LOW;
int latch1 = HIGH;
int latch2 = LOW;
int latch3 = LOW;
int latch4 = LOW;
int latch5 = LOW;
int latch6 = LOW;
int latch7 = LOW;
int latch8 = LOW;
int latch9 = LOW;
int latch10 = LOW;
int latch11 = LOW;
int latch12 = LOW;
int latch13 = LOW;
int latch14 = LOW;
int latch15 = LOW;
int latch16 = LOW;
int latch17 = LOW;
int p1 = 0;
int v1o = 0;
int v2o = 0;
int v3o = 0;
int v4o = 0;
int v5o = 0;
int v6o = 0;
int v1c = 0;
int v2c = 0;
int v3c = 0;
int v4c = 0;
int v5c = 0;
int v6c = 0;
int v1osw = 1;
int v2osw = 1;
int v3osw = 1;
int v4osw = 1;
int v5osw = 1;
int v6osw = 1;
int v1csw = 1;
int v2csw = 1;
int v3csw = 1;
int v4csw = 1;
int v5csw = 1;
int v6csw = 1;
int y1 =0;
int y2=0;
int y3=0;
int y4=0;
int y5=0;
int y6=0;
int y7=0;
int y8=0;
int y9=0;
int y10=0;
int y11=0;
int y12=0;
int y13=0;
int y14=0;
int y15=0;
int y16=0;
int inputState1[12]={v1osw, v2osw, v3osw, v4osw, v5osw, v6osw, v1csw, v2csw, v3csw, v4csw, v5csw, v6csw};
//int inputState2[8]={v2csw, v3csw, v4csw, v5csw, v6csw};
int outputState1[13]={p1, v1o, v2o, v3o, v4o, v5o, v6o, v1c, v2c, v3c, v4c, v5c, v6c};
int outputState2[7]={v2c, v3c, v4c, v5c, v6c, MTPIDOUT, HLTPIDOUT};
byte shiftIN1;
byte shiftIN2;
int shiftOUT1;
int shiftOUT2;
int currentStep;
unsigned long starttime = 0; //zero out start time
int timer1min = 1; //setpoint in minutes
int timer1set = (timer1min * 1000 *60); //setpoint in millis
unsigned long endtime = 0; //zero out end time
unsigned long timeRemaining = 0; //zero out the timer countdown
//unsigned long now = 0;
int timer1done = LOW;
int timersetup = LOW;
int timerlatch = LOW;


int MTTEMP;
int HLTTEMP;

void setup()
{
  pinMode (encoder0PinA, INPUT);
  pinMode (encoder0PinB, INPUT);
  pinMode (encoder0PinC, INPUT);
  pinMode (latchPin, OUTPUT);
  pinMode (dataPin, OUTPUT);
  pinMode (clockPin, OUTPUT);
  pinMode (dataPinIN, INPUT);
  pinMode (loadPin, OUTPUT);
  pinMode (clockPinIN, OUTPUT);
  // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.begin (9600);
  //windowStartTime = millis();

  //initialize the variables we're linked to
  HLTsetpoint = 168;
  MTsetpoint = 154;
  //Setpoint = 100;

  //tell the PID to range between 0 and the full window size
  MTPID.SetOutputLimits(0, WindowSize);
  HLTPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  MTPID.SetMode(AUTOMATIC);
  HLTPID.SetMode(AUTOMATIC);
}


void loop(){
updateinputs(); 
  if (latch1 == HIGH){
    homescreen();
  }
  if(latch2 == HIGH){
     menuscreen();
   }
  if (latch6 == HIGH){
    valveControl();
  }
 if (latch8 == HIGH){
   manualControl();
 }
if (latch10 == HIGH){
    settemps();
  }
if (latch14 == HIGH){
  PIDcontrolMT();
}
if (latch16 == HIGH){
  PIDcontrolHLT();
}
updateoutputs();
}

void homescreen(){

  if (digitalRead(encoder0PinC) == LOW){  
    latch2 = HIGH;
    encoder0Pos = 0;
    delay(100);
    //encoder0PinALast = HIGH;
  }
  if (latch5 == LOW){
    lcd.clear();
    latch5 = HIGH;}
  lcd.setCursor(0,0);
  lcd.print("HOME         PROG#  ");
  lcd.setCursor(0,1);
  lcd.print("P1:    VIN:  VOUT:  ");
  lcd.setCursor(0,2);
  lcd.print("HLT:      MT:      ");
  lcd.setCursor(0,3);
  lcd.print("STEP:    TIME:   min");
  lcd.setCursor(3,1);
  if (outputState1[0] == 0){
    lcd.print("OFF");}
  else if(outputState1[0] == 1){
    lcd.print("ON");}
  lcd.setCursor(11,1);
  for (int i = 0; i <= 2; i++){
     if (inputState1[i] == 0){
       lcd.print(i);
       }
     }
  lcd.setCursor(18,1);
  for (int j = 3; j <= 5; j++){
    if (inputState1[j] == 0){
       lcd.print(j);
     }
   }
  
  
  lcd.setCursor(4,2);
  lcd.print(HLTInput);
  lcd.setCursor(13,2);
  lcd.print(MTInput);
  
  
  lcd.setCursor(5,3);
  lcd.print(currentStep);
  lcd.setCursor(14,3);
  lcd.print(timeRemaining);
  return;
  }
   
void menuscreen(){
  latch1 = LOW;
  latch2 = HIGH;
  if (latch3 == LOW){
    lcd.clear();
    latch3 = HIGH;}
  lcd.setCursor(0,0);
  lcd.print("MENU");
  if (latch4 == LOW){
    lcd.setCursor(0,1);
    lcd.print(">");
    latch4 = HIGH;}
  lcd.setCursor(1,1);
  lcd.print("MAN CON   CALL PROG");
  lcd.setCursor(1,2);
  lcd.print("RUN PROG  EDIT PROG");
  lcd.setCursor(16,3);
  lcd.print("EXIT");
  
  attachInterrupt(0, rotary, CHANGE);
  
        if (encoder0Pos < 0){
          encoder0Pos = 4;}
        if (encoder0Pos > 4){
          encoder0Pos = 0;}
        if ((encoder0Pos - 1) > -1){
          lcd.setCursor(menuselectCol[encoder0Pos -1],menuselectRow[encoder0Pos -1]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) < 5){
          lcd.setCursor(menuselectCol[encoder0Pos +1],menuselectRow[encoder0Pos +1]);
          lcd.print(" ");
        }
        if ((encoder0Pos - 1) < 0){
          lcd.setCursor(menuselectCol[encoder0Pos +4], menuselectRow[encoder0Pos +4]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) > 4){
          lcd.setCursor(menuselectCol[encoder0Pos -4], menuselectRow[encoder0Pos -4]);
          lcd.print(" ");
        }  
        
         lcd.setCursor(menuselectCol[encoder0Pos],menuselectRow[encoder0Pos]);
         lcd.print(">");
      //delay(5);   
      
   
   if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 4)){
        latch1 = HIGH;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        detachInterrupt(0);
        //encoder0Pos = 0;
        //encoder0PinALast = HIGH;
        //n = HIGH;
        //lcd.clear();
        delay(100);
      }      
   if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 0)){         
        latch1 = LOW;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = LOW;
        latch8 = HIGH;
        encoder0Pos = 0;
        //encoder0PinALast = HIGH;
        //n = HIGH;
        //lcd.clear();
        delay(100);
       }  
 
 return;
}



void manualControl(){
  if (encoder0Pos < 0){
          encoder0Pos = 5;}
  if (encoder0Pos > 5){
          encoder0Pos = 0;}
  lcd.setCursor(manconCol[encoder0Pos],manconRow[encoder0Pos]);
         lcd.print(">");
  //latch1 = LOW;
  //latch2 = LOW;
  //latch6 = LOW;
  //latch8 = HIGH; 
  if (latch9 == LOW){
    lcd.clear();
    latch9 = HIGH;}
  lcd.setCursor(0,0);
  lcd.print("MANUAL CONTROL");
  if (latch4 == LOW){
    lcd.setCursor(0,1);
    lcd.print(">");
    latch4 = HIGH;}
  lcd.setCursor(1,1);
  lcd.print("PUMP:     VALVE CON");
  lcd.setCursor(6,1);
  if (outputState1[0]==0){
    lcd.print("OFF");}
  else if (outputState1[0]==1){
    lcd.print("ON ");}
  lcd.setCursor(1,2);
  lcd.print("MT:       HLT:     ");
  lcd.setCursor(4,2);
  if (latch14 == LOW){
    lcd.print("OFF");}
    else if (latch14 == HIGH){
      lcd.print("ON ");}
  lcd.setCursor(15,2);
  if (latch16 == LOW){
    lcd.print("OFF");}
    else if (latch16 == HIGH){
      lcd.print("ON ");}
  lcd.setCursor(1,3);
  lcd.print("SET TEMPS      BACK");
  
        if ((encoder0Pos - 1) > -1 ){
          lcd.setCursor(manconCol[encoder0Pos -1],manconRow[encoder0Pos -1]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) < 6){
          lcd.setCursor(manconCol[encoder0Pos +1],manconRow[encoder0Pos +1]);
          lcd.print(" ");
        }
        if ((encoder0Pos - 1) < 0){
          lcd.setCursor(manconCol[encoder0Pos +5], manconRow[encoder0Pos +5]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) > 5){
          lcd.setCursor(manconCol[encoder0Pos -5], manconRow[encoder0Pos -5]);
          lcd.print(" ");
        }          
        
         
      //delay(40);   
        if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 5)){
        latch1 = LOW;
        latch2 = HIGH;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = LOW;
        latch7 = LOW;
        latch8 = LOW;        
        encoder0Pos = 0;
        //encoder0PinALast = HIGH;
        //n = HIGH;
        //lcd.clear();
        delay(100);
      }
        if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 0) && (outputState1[0] == 1)){
        lcd.setCursor(6,1);
        lcd.print("OFF");
        outputState1[0] = 0;
        //Serial.println(outputState1[0]);
        //Serial.println(y1);
        //Serial.println(shiftOUT1);
        delay(100);
      }
        if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 0) && (outputState1[0] == 0)){
        lcd.setCursor(6,1);
        lcd.print("ON ");
        outputState1[0] = 1;
        //Serial.println(outputState1[0]);
        //Serial.println(y1);
        //Serial.println(shiftOUT1);  
        delay(100);
       }
        else if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 2) && (latch14 == LOW)){
          lcd.setCursor(4,2);
          lcd.print("ON ");
         latch14 = HIGH;
        }
         else if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 2) && (latch14 == HIGH)){
          lcd.setCursor(4,2);
          lcd.print("OFF");
         latch14 = LOW;
         latch15 = LOW;
         outputState2[5] = 0;
        }
        else if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 3) && (latch16 == LOW)){
          lcd.setCursor(15,2);
          lcd.print("ON ");
          latch16 = HIGH;
        }
        else if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 3) && (latch16 == HIGH)){
          lcd.setCursor(15,2);
          lcd.print("OFF");
          latch16 = LOW;
          latch17 = LOW;
          outputState2[6] = 0;
          
      }
        
        if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 1)){
        latch1 = LOW;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = HIGH;
        latch7 = LOW;
        latch8 = LOW;
        latch9 = LOW;
        encoder0Pos = 0;
        //encoder0PinALast = HIGH;
        //n = HIGH;
      }
      if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 4)){
        latch1 = LOW;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = LOW;
        latch7 = LOW;
        latch8 = LOW;
        latch9 = LOW;
        latch10= HIGH;
        latch11= LOW;
        encoder0Pos = 0;
      }       
  return;
}




void valveControl(){
  if (encoder0Pos < 0){
          encoder0Pos = 6;
        }
  if (encoder0Pos > 6){
          encoder0Pos = 0;
        }
  //latch1 = LOW;
  //latch2 = LOW;
  //latch6 = HIGH;
  if (latch7 == LOW){
    lcd.clear();
    latch7 = HIGH;}
  lcd.setCursor(0,0);
  lcd.print("VALVE CONTROL");
  if (latch4 == LOW){
    lcd.setCursor(0,1);
    lcd.print(">");
    latch4 = HIGH;
  }
  lcd.setCursor(1,1);
  lcd.print("V1:     V2:");
  lcd.setCursor(4,1);
  if ((outputState1[7]==1) || (inputState1[6]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[1]==1) || (inputState1[0]==0)){
      lcd.print("OPN");
    }
  lcd.setCursor(12,1);
  if ((outputState1[8]==1) || (inputState1[7]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[2]==1) || (inputState1[1]==0)){
      lcd.print("OPN");
    }
  lcd.setCursor(1,2);
  lcd.print("V3:     V4:");
  lcd.setCursor(4,2);
  if ((outputState1[9]==1) || (inputState1[8]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[3]==1) || (inputState1[2]==0)){
      lcd.print("OPN");
    }
  lcd.setCursor(12,2);
  if ((outputState1[10]==1) || (inputState1[9]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[4]==1) || (inputState1[3]==0)){
      lcd.print("OPN");
    }
  
  lcd.setCursor(1,3);
  lcd.print("V5:     V6:");
  lcd.setCursor(4,3);
  if ((outputState1[11]==1) || (inputState1[10]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[5]==1) || (inputState1[4]==0)){
      lcd.print("OPN");
    }
  lcd.setCursor(12,3);
  if ((outputState1[12]==1) || (inputState1[11]==0)){
    lcd.print("CLS");
  }
    else if ((outputState1[6]==1) || (inputState1[5]==0)){
      lcd.print("OPN");
    }
  lcd.setCursor(16,3);
  lcd.print("BACK");
  
        if ((encoder0Pos - 1) > -1){
          lcd.setCursor(valveconCol[encoder0Pos -1],valveconRow[encoder0Pos -1]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) < 7){
          lcd.setCursor(valveconCol[encoder0Pos +1],valveconRow[encoder0Pos +1]);
          lcd.print(" ");
        }
        if ((encoder0Pos - 1) < 0){
          lcd.setCursor(valveconCol[encoder0Pos +6], valveconRow[encoder0Pos +6]);
          lcd.print(" ");
        }
        if ((encoder0Pos + 1) > 6){
          lcd.setCursor(valveconCol[encoder0Pos -6], valveconRow[encoder0Pos -6]);
          lcd.print(" ");
        }  
        lcd.setCursor(valveconCol[encoder0Pos],valveconRow[encoder0Pos]);
        lcd.print(">");
  
   if ((digitalRead(encoder0PinC) == LOW) && (outputState1[encoder0Pos + 1] == 0) && (encoder0Pos != 6) && (inputState1[encoder0Pos] == 1)){   
     lcd.setCursor((valveconCol[encoder0Pos]+4), valveconRow[encoder0Pos]);
     lcd.print("OPN");
     outputState1[encoder0Pos + 1] = 1;
     outputState1[encoder0Pos + 7] = 0;
     delay(100);
   }      
 
    else if ((digitalRead(encoder0PinC) == LOW) && (outputState1[encoder0Pos +7] == 0) && (encoder0Pos != 6)  && (inputState1[encoder0Pos + 6] == 1)){
         lcd.setCursor((valveconCol[encoder0Pos] +4), valveconRow[encoder0Pos]);
        lcd.print("CLS");
        outputState1[encoder0Pos + 1] = 0;
        outputState1[encoder0Pos + 7] = 1;
        delay(100);
    }
    
      else if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 6)){
        latch1 = LOW;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = LOW;
        latch7 = LOW;
        latch8 = HIGH;
        latch9 = LOW;
        
        encoder0Pos = 0;
        //encoder0PinALast = HIGH;
        //n = HIGH;
        //delay(40);
      }   
  return;
}

void updateoutputs(){
  
  for (m = 1; m < 13; m++){
    if (inputState1[m-1] == 0){
    outputState1[m] = 0;  
    }
  }
 y1 = (outputState1[0] * 128);   
 y2 = (outputState1[1] * 64);    
 y3 = (outputState1[2] * 32);
 y4 = (outputState1[3] * 16);
 y5 = (outputState1[4] * 8);
 y6 = (outputState1[5] * 4);
 y7 = (outputState1[6] * 2);
 y8 = (outputState1[7] * 1);
 y9 = (outputState1[8] * 128);
 y10 = (outputState1[9] * 64);
 y11 = (outputState1[10] * 32);
 y12 = (outputState1[11] * 16);
 y13 = (outputState1[12] * 8);
 y14 = (outputState2[5] * 4);
 y15 = (outputState2[6] * 2);
 y16 = (0 * 1);  
      
    shiftOUT1 = (byte)(y1 + y2 + y3 + y4 + y5 + y6 + y7 + y8);
    shiftOUT2 = (byte)(y9 + y10 + y11 + y12 + y13 + y14 + y15 + y16);
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, shiftOUT2); 
    shiftOut(dataPin, clockPin, LSBFIRST, shiftOUT1);
    digitalWrite(latchPin, HIGH);
    //m = 0;
    //Serial.println(shiftOUT1, BIN);
    //delay(500);
    //Serial.println(shiftOUT2, BIN);
    //delay(500);
    return;
}

void settemps(){
  if (encoder0Pos < 0){
          encoder0Pos = 2;}
  if (encoder0Pos > 2){
          encoder0Pos = 0;}
  if (latch12 == LOW && latch13 == LOW){
        lcd.setCursor(settempCol[encoder0Pos],settempRow[encoder0Pos]);
        lcd.print(">");
        }
  //latch1 = LOW;
  //latch2 = LOW;
  //latch6 = HIGH;
  if (latch11 == LOW){
    lcd.clear();
    latch11 = HIGH;}
  lcd.setCursor(0,0);
  lcd.print("SET TEMPERATURE");
  if (latch4 == LOW){
    lcd.setCursor(0,1);
    lcd.print(">");
    latch4 = HIGH;}
  lcd.setCursor(1,1);
  lcd.print("SET MT TEMP     ");
  lcd.setCursor(13,1);
  lcd.print(MTsetpoint);  
  lcd.setCursor(1,2);
  lcd.print("SET HLT TEMP     ");
  lcd.setCursor(14,2);
  lcd.print(HLTsetpoint);
  lcd.setCursor(16,3);
  lcd.print("BACK");
  
        if (((encoder0Pos - 1) > -1) && latch12 == LOW && latch13 == LOW){
          lcd.setCursor(settempCol[encoder0Pos -1],settempRow[encoder0Pos -1]);
          lcd.print(" ");
        }
        if (((encoder0Pos + 1) < 3) && latch12 == LOW && latch13 == LOW){
          lcd.setCursor(settempCol[encoder0Pos +1],settempRow[encoder0Pos +1]);
          lcd.print(" ");
        }
        if (((encoder0Pos - 1) < 0) && latch12 == LOW && latch13 == LOW){
          lcd.setCursor(settempCol[encoder0Pos +2], settempRow[encoder0Pos +2]);
          lcd.print(" ");
        }
        if (((encoder0Pos + 1) > 2) && latch12 == LOW && latch13 == LOW){
          lcd.setCursor(settempCol[encoder0Pos -2], settempRow[encoder0Pos -2]);
          lcd.print(" ");
        }  
        
if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 0) && (latch13 == LOW)){ 
  //lcd.print(" ");
  latch12 = HIGH;
  encoder0Pos = 0;
  encoderDir = 0;
  delay(100);
}
  if (latch12 == HIGH){
 
    lcd.setCursor(settempCol[3], settempRow[3]);
  MTsetpoint = MTsetpoint + encoderDir;
  encoderDir = 0;
  //lcd.blink();
  lcd.print(MTsetpoint);
  //delay(10);
}
if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 1) && (latch12 == LOW)){ 
  //lcd.print(" ");
  latch13 = HIGH;
  encoder0Pos = 0;
  encoderDir = 0;
  delay(100);
}
  if (latch13 == HIGH){
    lcd.setCursor(settempCol[4], settempRow[4]);
  HLTsetpoint = HLTsetpoint + encoderDir;
  encoderDir = 0;
  //lcd.blink();
  lcd.print(HLTsetpoint);
  //delay(10);
}
  if (latch12 == HIGH && (digitalRead(encoder0PinC) == LOW)){
    encoder0Pos = 0;
    encoderDir = 0;
    latch12 = LOW;

     delay(100);
  }
  if (latch13 == HIGH && (digitalRead(encoder0PinC) == LOW)){
    encoder0Pos = 0;
    encoderDir = 0; 
    latch13 = LOW;
     
     delay(100);
  }  
if ((digitalRead(encoder0PinC) == LOW) && (encoder0Pos == 2)){ 
        latch1 = LOW;
        latch2 = LOW;
        latch3 = LOW;
        latch4 = LOW;
        latch5 = LOW;
        latch6 = LOW;
        latch7 = LOW;
        latch8 = HIGH;
        latch9 = LOW;
        latch10 = LOW;
        latch11 = LOW;
        
        encoder0Pos = 0;
        delay(100);
}


return;
}
void PIDcontrolMT(){
  if (latch17 == LOW){
    windowStartTime = millis();
    MTPID.SetMode(AUTOMATIC);
    latch17 = HIGH;
    
  }
  //MTInput = analogRead(0);
  MTPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (MTOutput > now - windowStartTime) outputState2[5] = 1;
  else (outputState2[5] = 0); 
  
return;
}

void PIDcontrolHLT(){
  if (latch15 == LOW){
    windowStartTime = millis();
    HLTPID.SetMode(AUTOMATIC);
    latch15 = HIGH;
  }
  
  //HLTInput = analogRead(1);
  HLTPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (HLTOutput > now - windowStartTime) outputState2[6] = 1;
  else (outputState2[6] = 0); 
  
return;
}
   
void updateinputs(){

    MTInput = analogRead(0);
    HLTInput = analogRead(1);
    digitalWrite(loadPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(clockPinIN, HIGH);
    digitalWrite(loadPin, LOW);
    shiftIN1 = shiftIn(dataPinIN, clockPinIN, MSBFIRST); 
    delay(5);
    shiftIN2 = shiftIn(dataPinIN, clockPinIN, MSBFIRST);
    delay(5);
    
    
  for (k = 0; k < 8; k++){
    inputState1[k] = bitRead(shiftIN1,k);
  }
  for (l = 0; l < 8; l++){
    inputState1[l +8] = bitRead(shiftIN2,l);
  }
  
//Serial.println("SHIFTIN1");
//Serial.println(shiftIN1, BIN);
//delay(500);
//Serial.println("SHIFTIN2");
//Serial.println(shiftIN2, BIN);
//delay(500);
return;
}
void rotary(){
    n = digitalRead(encoder0PinA);  
    int encoder0Prev = encoder0Pos;
    if ((encoder0PinALast == HIGH) && (n == LOW)) {
      if (digitalRead(encoder0PinB) == HIGH) {
        encoder0Pos --;   
      } else {
        encoder0Pos ++;
      }
    }
  if (encoder0Pos - encoder0Prev > 0){
    encoderDir = 1;
  }
  else {if (encoder0Pos - encoder0Prev < 0){
    encoderDir = -1;
  }
  }
  encoder0PinALast = n;

}
//-------------------------------------------------------------------------------------------------------------------
void timer1(){
  if (timersetup == LOW){ //if the skip bit is low
  starttime = millis(); //set the start time to current millis
  endtime = starttime + timer1set; //set the end time
  timersetup = HIGH; //set the skip bit high
  }
  else if (endtime < millis()){ //if the timer is done
  timer1done = HIGH; //set the timer done bit high
  starttime = 0; //zero out the start time
  endtime = 0; //zero out the end time
  timerlatch = LOW; //disable the latch for timer 1
  }
  else if (endtime > millis()){
        //now = millis() / 1000;
        timeRemaining = ((endtime - millis())/60000); //calculate the countdown
  int timerON = HIGH; //set the timer on bit to high
  }
return;
}
