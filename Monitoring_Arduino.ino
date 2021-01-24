//THIS PROGRAM HAS CHANGES FOR BOTH 3 sensor android app and all senosr android app
//For all sensor android app - we use the character 'a'
//for three sensor android app - we use the character 'n' (n means not all)

//A2- EMG
//A0-ECG
#include "MAX30105.h"
#include "heartRate.h"
#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>
#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include<SoftwareSerial.h>


MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int  HB=10, EMG=21, BP=10, RP=12, temp=27;
int pulsePin = A1;                 // Pulse Sensor purple wire connected to analog pin A0
int blinkPin = 13;                // pin to blink led at each beat
 int count;
const int LED=13;
const int GSR=A3;  //not sure though.. currently, I'm dealing with ecg and emg signals only.. so, I'm sure about them
int threshold=0;
int sensorValue;

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse

volatile int rate[10];                      // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

void setup()
{ 
  Serial.begin(115200);             // we agree to talk fast!
  long sum=0;
  //Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);
  serialOutput();
  pinMode(blinkPin, OUTPUT);        // pin that will blink to your heartbeat!
  interruptSetup();   
   pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);// sets up to read Pulse Sensor signal every 2mS
  // IF YOU ARE POWERING The Pulse Sensor AT VOLTAGE LESS THAN THE BOARD VOLTAGE,
  // UN-COMMENT THE NEXT LINE AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //   analogReference(EXTERNAL);
  pinMode(3, INPUT); // Setup for leads off detection LO +
  pinMode(5, INPUT); // Setup for leads off detection LO -
  for(int i=0;i<500;i++)
  {
  sensorValue=analogRead(GSR);
  sum += sensorValue;
  delay(5);
  }
  threshold = sum/500;
  // Serial.print("threshold =");
 //  Serial.println(threshold);
   mlx.begin();
}


//send a char from Phone to arduino (press "ecg" button) & receive values on Phone (print ecg values on console)
/*
"x" for stop
*/
char receiveddata;
char receiveddata1;


void loop() {
  receiveddata = Serial.read();
  if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
    receiveddata1 = receiveddata;
  }
  while(receiveddata1 == 'c'){
    if((digitalRead(3)==1)||(digitalRead(5)==1)){
      Serial.println('!'); //no data
    }
    else {
      Serial.println(analogRead(A0));
    }
    delay(20);  // earlier delay was of 1ms
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
  while(receiveddata1 == 'm'){
    Serial.println(analogRead(A2));
    delay(20);
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
  while(receiveddata1 == 'g'){
    Serial.println(analogRead(GSR));
    delay(20);
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
  while(receiveddata1 == 'p'){
      long irValue = particleSensor.getIR();

  if (irValue < 50000)
    Serial.print("?"); //no finger detected

  else if (checkForBeat(irValue) == true){
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
    
  }

  Serial.print(beatAvg);

  Serial.println();
    delay(20);
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
  while(receiveddata1 == 't'){
    Serial.print(mlx.readObjectTempF());
    Serial.println();
    delay(100);  // ealier the delay was 20 ms
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
  while(receiveddata1 == 'n'){ //for only 3 sensors.. format is _,_/_/n
    if((digitalRead(3)==1)||(digitalRead(5)==1)){
      Serial.print('!');
      Serial.print(",");
       Serial.print(mlx.readObjectTempF());
      Serial.print("/");   

      long irValue = particleSensor.getIR();

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      Serial.println(beatAvg);
        
      delay(20);
      receiveddata = Serial.read();
      if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
        receiveddata1 = receiveddata;  
      }
    }
    else{
      Serial.print(analogRead(A0));
      Serial.print(",");
       Serial.print(mlx.readObjectTempF());
      Serial.print("/");
      
      long irValue = particleSensor.getIR();

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      Serial.println(beatAvg);
      
      delay(20);  
      receiveddata = Serial.read();
      if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' ||receiveddata=='x'){
        receiveddata1 = receiveddata;
      }    
    }
  }
  while(receiveddata1 == 'a'){ //for all sensors.. format is _,_/_:_/n
    if((digitalRead(3)==1)||(digitalRead(5)==1)){
      Serial.print('!');
      Serial.print(",");
      Serial.print(analogRead(A2));
      Serial.print("/");
      Serial.print(analogRead(GSR));
      Serial.print(":"); 

      long irValue = particleSensor.getIR();

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      Serial.println(beatAvg);
        
      delay(20);
      receiveddata = Serial.read();
      if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
        receiveddata1 = receiveddata;  
      }
    }
    else{
      Serial.print(analogRead(A0));
      Serial.print(",");
      Serial.print(analogRead(A2));
      Serial.print("/");
      Serial.print(analogRead(GSR));
      Serial.print(":");
      
      long irValue = particleSensor.getIR();

      if (checkForBeat(irValue) == true)
      {
        //We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();
    
        beatsPerMinute = 60 / (delta / 1000.0);
    
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
          rateSpot %= RATE_SIZE; //Wrap variable
    
          //Take average of readings
          beatAvg = 0;
          for (byte x = 0 ; x < RATE_SIZE ; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }
      Serial.println(beatAvg);
      
      delay(20);  
      receiveddata = Serial.read();
      if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
        receiveddata1 = receiveddata;
      }    
    }
  }
  while(receiveddata1 == 'x'){
    //nothing is sent on mobile
    receiveddata = Serial.read();
    if(receiveddata=='c' || receiveddata=='m' || receiveddata=='g' || receiveddata=='p' || receiveddata=='t' || receiveddata=='a' || receiveddata=='n' ||receiveddata=='x'){
      receiveddata1 = receiveddata;
    }
  }
//
//  
//    if (QS == true) // A Heartbeat Was Found
//  {
//    // BPM and IBI have been Determined
//    // Quantified Self "QS" true when arduino finds a heartbeat
//    serialOutputWhenBeatHappens(); // A Beat Happened, Output that to serial.
//    QS = false; // reset the Quantified Self flag for next time
//  }
//
//  delay(1000); //  take a break2
}





void interruptSetup()
{
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED
}

void serialOutput()
{ // Decide How To Output Serial.
  if (serialVisual == true)
  {
    arduinoSerialMonitorVisual('-', Signal);   // goes to function that makes Serial Monitor Visualizer
  }
  else
  {
    sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
  }
}

void serialOutputWhenBeatHappens()
{
  if (serialVisual == true) //  Code to Make the Serial Monitor Visualizer Work
  {
         Serial.print("\n");

//        Serial.print(analogRead(A0));
//        
//        Serial.print(" "); 
//         Serial.print(BPM);
//        Serial.print(" ");  
//        Serial.print(analogRead(A3));
//        Serial.print(" ");
//        Serial.print(RP);
//        Serial.print(" ");
//        Serial.print(BP);
//        Serial.print(" ");
//        Serial.print(mlx.readObjectTempC());
//        Serial.print(" ");
        
  }
  else
  {
    sendDataToSerial('B', BPM);  // send heart rate with a 'B' prefix
    sendDataToSerial('Q', IBI);  // send time between beats with a 'Q' prefix
  }
}

void arduinoSerialMonitorVisual(char symbol, int data )
{
  const int sensorMin = 0;      // sensor minimum, discovered through experiment
  const int sensorMax = 1024;    // sensor maximum, discovered through experiment
  int sensorReading = data; // map the sensor range to a range of 12 options:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 11);
  // do something different depending on the
  // range value:
}


void sendDataToSerial(char symbol, int data )
{
  Serial.print(symbol);
  Serial.println(data);
}

ISR(TIMER2_COMPA_vect) //triggered when Timer2 counts to 124// Interrupt service Routine(ISR)
{
  cli();                                      // disable interrupts while we do this // Command line interface(cli)
  Signal = analogRead(pulsePin);              // read the Pulse Sensor
  sampleCounter += 2;                         // keep track of the time in mS with this variable
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
  //  find the peak and trough of the pulse wave
  if (Signal < thresh && N > (IBI / 5) * 3) // avoid dichrotic noise by waiting 3/5 of last IBI
  {
    if (Signal < T) // T is the trough
    {
      T = Signal; // keep track of lowest point in pulse wave
    }
  }

  if (Signal > thresh && Signal > P)
  { // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 300)
  { // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) )
    {
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(blinkPin, HIGH);               // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if (secondBeat)
      { // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for (int i = 0; i <= 9; i++) // seed the running total to get a realisitic BPM at startup
        {
          rate[i] = IBI;
        }
      }

      if (firstBeat) // if it's the first time we found a beat, if firstBeat == TRUE
      {
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }
      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable

      for (int i = 0; i <= 8; i++)
      { // shift data in the rate array
        rate[i] = rate[i + 1];                // and drop the oldest IBI value
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values
      BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }
  }

  if (Signal < thresh && Pulse == true)
  { // when the values are going down, the beat is over
    digitalWrite(blinkPin, LOW);           // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp / 2 + T;                  // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500)
  { // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }

  sei();                                   // enable interrupts when youre done!
}// end isr

    
