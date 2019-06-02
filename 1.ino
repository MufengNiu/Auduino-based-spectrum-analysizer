/*
 *   This is a spectrum analysizer project based on arduino Due.  
 *   Created 02/06/2019 by MUFENG NIU
 */



#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>

#define blueToothSerial Serial2 
#define SAMPLES_TAKEN 64            //
#define FREQ_RANGE 10000            // From 0 to 10000/2 HZ is measured
#define  xlim 8                    // Total number of  columns
#define  ylim 8                     // Total number of  rows 
#define NUM_LEDS 64
#define MATRIX_IN 9

arduinoFFT FFT = arduinoFFT();                                    
Adafruit_NeoPixel strip(NUM_LEDS, MATRIX_IN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of leds
// Argument 2 = pin number in
// Argument 3 and 4 = display type 


double vReal[SAMPLES_TAKEN];
double vImag[SAMPLES_TAKEN];
char avgs[xlim];
unsigned int sampling_period_us;
unsigned long microseconds;
int status  = 0;          // 0 for on , 1 for off    Global valuables enable BT control
uint32_t colour = strip.Color(150,0,150);  
// default color purple
// Green:(0,255,0)  
// Red:(255,0,0)  
// Blue:(0,0,255)  
// OFF : (0,0,0)




void setup() {
 
    Serial.begin(115200);   // Highest speed
    sampling_period_us = round(1000000*(1.0/FREQ_RANGE));
    strip.begin();           // initialize RGB matrix 
    strip.show();            // enable matrix 
    strip.setBrightness(30); // setting brightness    
   // setupBTConnection();
    
    attachInterrupt(3,colorShift,FALLING );     
 }
 
void loop() {
   
  BTControl();
  SerialControl();
   
   for(int i=0; i<SAMPLES_TAKEN; i++)
  {
      microseconds = micros(); 
      int value =analogRead(A0) ; 
      //Serial.println (analogRead(A0));
      vReal[i]= value-750;                      // Sampling the data, storing in vReal[]
      vImag[i] = 0;      

      while(micros() < (microseconds + sampling_period_us)){
      }
   }
    // Applying FFT algorithms
    FFT.Windowing(vReal, SAMPLES_TAKEN, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES_TAKEN, FFT_FORWARD);
    // Applying FFT algorithms
    
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES_TAKEN);
    //Calculating magnitude, store data in vReal

    
    //From 0hz to 5000hz, diveded into 8 ranges
    //Calculating average of every 4 elements, store average in avgs[8]
    int count=0;
    for(int j=0; j<32; j=j+4)  
    {
      avgs[count] = 0;
      for (int k=0 ; k< 4 ; k++) {
          avgs[count] = avgs[count] + vReal[j+k];
      }
      avgs[count] = avgs[count]/4; 
      count++;
    }
    
    avgs[0]=  avgs[1]/1.5; // filting out noise line
    
    //Send to display according average value
    for(int i=0; i<xlim; i++){
     
      avgs[i] = constrain(avgs[i],0,60); //Setting the limits of avgs to 0 and 60
               
      avgs[i] = map(avgs[i], 0, 60, 0, ylim); // Sampling data to 0 and 8  

      if(status == 0){
          for(int j=0; j<8; j++)
         {
           int index = 8*i+j ;
           if (avgs[i]>j){ 
            strip.setPixelColor(index, colour); 
           }else{
            strip.setPixelColor(index, strip.Color(0,0,0));
            }
         }  
           strip.show();
      } else{
        off();
         }
      }
 }

// Turning the matrix off
void off() {
  for(int i=0; i<strip.numPixels(); i++) 
  { 
    strip.setPixelColor(i, strip.Color(0,0,0));         
  }
  strip.show();
}


void showColomn(int index, int height)
{
  
  for(int i = 0; i <height;i++){
       strip.setPixelColor(index, colour);
       //delay(50);
       index = index + 8;
  }
  strip.show();
  
  for(int i = 0; i <=height;i++)
  { 
    strip.setPixelColor(index, strip.Color(0,0,0));  
    //delay(50);    
    index = index - 8;                  
  }
   strip.show();
}

//Bluetooth Control
void BTControl(){
  char recChar;
  if(blueToothSerial.available()) { 
    recChar = blueToothSerial.read();
     if (recChar  == 'o'){
        status = 0;      // Turn on
     }
     if (recChar  == 'f') {
        status = 1;     // Turn off
      }
     if (recChar  == 'r') {
        colour = strip.Color(255,0,0); //Set colour to red
     }
     if (recChar  == 'g') {
        colour = strip.Color(0,255,0); //Set colour to green
     }
     if (recChar  == 'b') {
        colour = strip.Color(0,0,255); //Set colour to blue
     }
  }

}


void SerialControl(){
  char recChar;
  if(Serial.available()) { 
    recChar = Serial.read();
    
     if (recChar  == 'o'){
        status = 0;      // Turn on
        Serial.println("Turn on");
     }
     if (recChar  == 'f') {
        status = 1;     // Turn off
        Serial.println("Turn off");
      }
     if (recChar  == 'r') {
        colour = strip.Color(255,0,0); //Set colour to red
        Serial.println("Set colour to red");
     }
     if (recChar  == 'g') {
        colour = strip.Color(0,255,0); //Set colour to green
        Serial.println("Set colour to green");
     }
     if (recChar  == 'b') {
        colour = strip.Color(0,0,255); //Set colour to blue
        Serial.println("Set colour to blue");
     }
  }
}



void colorShift(){

        colour = strip.Color( random(0,255)  , random(0,255) ,random(0,255) );  
}



void setupBTConnection()
{
    blueToothSerial.begin(38400);                           // Set baud rate
    blueToothSerial.print("\r\n+STWMOD=0\r\n");             // Set  slave mode
    blueToothSerial.print("\r\n+STNA=Dancing Lights\r\n");  // Set  name 
    blueToothSerial.print("\r\n+STOAUT=1\r\n");             // Allow pair
    blueToothSerial.print("\r\n+STAUTO=0\r\n");             // Disable auto connection
    blueToothSerial.print("\r\n+STPIN =112233\r\n");        // Set Pin
    delay(2000);                                           
    blueToothSerial.print("\r\n+INQ=1\r\n");                // make the slave bluetooth inquirable
    Serial.println("The slave bluetooth is inquirable!");
    delay(2000);                                            // This delay is required.
    blueToothSerial.flush();
}
