/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

//#include <PID_v1.h>

#define PIN_INPUT A2
#define BRAKE_PIN 9
#define DIRECTION_PIN 12
#define SPEED_PIN 3
#define OPEN_PIN 4
#define CLOSED_PIN 5
#define OVERRIDE_PIN 6

//Define Variables we'll be connecting to
double SPL_Limit, SPL_Threshold, InputSPL, OutputPWM, peakSPL, sampleRate;

//Specify the links and initial tuning parameters
double Kp=20, Ki=0, Kd=0;
int direction = 1;
//PID myPID(&InputSPL, &OutputPWM, &Setpoint, Kp, Ki, Kd, DIRECT);


// Microphone input
//int soundLevelTest = 60;
double mVin = 0;


// Set microphone calibration variables
double Voff = 396.0; // millivolts
const double p0 = 0.00002; // Pa8
const double sensitivity = 6.31; //mV/Pa
const double A_filt = 0.502; // A-filter gain
//const double R1 = 9890; // Ohms
const double R1 = 4658; // Ohms
//const double R2 = 179500; //Ohms
const double R2 = 1011000; //Ohms
const double G = R2/R1; // Pre-amp gain
const double S_total = p0*sensitivity*A_filt*G; //Overall Gain

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  pinMode(BRAKE_PIN, OUTPUT);
  digitalWrite(BRAKE_PIN, LOW);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(SPEED_PIN, OUTPUT);  

  pinMode(OPEN_PIN,INPUT_PULLUP);
  pinMode(CLOSED_PIN, INPUT_PULLUP);
  pinMode(OVERRIDE_PIN, INPUT_PULLUP); // Ground Pin 6 to stop the motor

  SPL_Limit = 100;
  SPL_Threshold = 65;
  sampleRate = 50;
  InputSPL = getInputLevel();
  Serial.print("1st-bit resolution: ");
  Serial.println(20*log10(4.9/S_total));
  Serial.print("S_total: ");
  Serial.println(S_total*1000);
  //turn the PID on
//  myPID.SetMode(AUTOMATIC);

  controlMotor(-1); // close the HPD to start
  delay(sampleRate);
}

void loop()
{
  if (digitalRead(OVERRIDE_PIN) == HIGH) {
    stopMotor();
    Serial.println("OVERRIDE");      
    delay(sampleRate);
    return;
  }
  Serial.println("");
  InputSPL = getInputLevel();
  //myPID.Compute();
  Serial.print("Current SPL: ");
  Serial.println(InputSPL);
  Serial.print("Device Status: ");

  maybeRecord(InputSPL);

  if (InputSPL > SPL_Limit) {    // If it's too loud, close it
    if(digitalRead(CLOSED_PIN) == LOW) {
      stopMotor();
      Serial.println("CLOSED");
    }
    else {
      Serial.println("too loud...");
      controlMotor(InputSPL - SPL_Limit);
      
    }
  }
  else if(InputSPL <= SPL_Threshold) {    // If it's too quiet, open it
    if (digitalRead(OPEN_PIN) == LOW) {
      stopMotor();
      Serial.println("OPEN");
    }
    else {
      Serial.println("too quiet...");        
      controlMotor(InputSPL - SPL_Threshold);
      while(digitalRead(OPEN_PIN) == HIGH){}
      stopMotor();
      delay(20*sampleRate);
    }
  } 
  else { // We're in a safe range 
    Serial.println("Ok...");
    stopMotor();
  }
  
  if (InputSPL > peakSPL) {
    peakSPL = InputSPL;
    Serial.print("Peak SPL:");
    Serial.println(peakSPL);
  }
  
  delay(sampleRate);
}

void controlMotor(double dB_diff) {    
    digitalWrite(BRAKE_PIN, LOW);
  
    Serial.print("Difference: ");
    Serial.println(dB_diff);
    
    if (dB_diff < 0) {
      // too quiet
      direction = 1;
      digitalWrite(DIRECTION_PIN, LOW);
      
    } else {
      // too loud
      direction = -1;
      digitalWrite(DIRECTION_PIN, HIGH);
    }
    
    dB_diff = dB_diff * direction;
    OutputPWM = 200;     // Constant speed 
    // (we don't have far to go)
    
    Serial.print("OutputPWM: ");  
    Serial.println((int) OutputPWM);
    analogWrite(SPEED_PIN, (int) OutputPWM);
}

void stopMotor() {
  analogWrite(SPEED_PIN, 0);
  digitalWrite(BRAKE_PIN, HIGH);
  Serial.println("STOPPING");
}

double getInputLevel() {
  mVin = analogRead(PIN_INPUT)*4.9;
  // Serial.print("mVin: ");
  // Serial.print(mVin);
  mVin = mVin - Voff;
  if (mVin <= 0)
    mVin = S_total;
  // Serial.print("... mVin - Voff: ");
  // Serial.println(mVin);
  double dB = (20*log10(mVin/(S_total)));
//  dB = 100;
  return dB;
}

// double=64bytes
// int=4bytes
// 68 bytes total

// at 3600*8 records, that's 195kB, which is acceptable
struct NoiseLevelRecord {
  double noiseLevel; // in dB
  int timeSpan; // in ms, duration of this noise (a rough estimate)
  int timeStamp; // in ms, when this noise occurred
};


double totalTime = 8*3600; // in seconds, the total time the wearer is expected to be exposed to noise
const int MS_IN_HR = 3600 * 1000;
const int RECORDS_PER_SECOND = 1; // how frequently the noise level will be recorded
const int RECORD_INTERVAL = 1000/RECORDS_PER_SECOND; // in ms
long lastTimeRecorded = -1;

// total records = 28800 = totalTime*RECORDS_PER_SECOND
// TODO this is definitely going to exceed space in RAM, consider using flash memory
struct NoiseLevelRecord noiseLevelRecords[28800]; // considering a linked list, to allow variable length
double cumulativeExposure[28800]; // save compute time by doing a running total
int numRecords = 0;

void recordNoiseLevel(double noiseLevel, int timeSpan, int timeStamp) {
  struct NoiseLevelRecord record;
  record.noiseLevel = noiseLevel;
  record.timeSpan = timeSpan;
  record.timeStamp = timeStamp;

  noiseLevelRecords[numRecords] = record;
  cumulativeExposure[numRecords] = computeRecord(record);
  numRecords++;
}

// internalNoiseLevel in dB
void maybeRecord(double noiseLevel) {
  long currentTime = millis(); // this is appropriate for demo only. This value overflows after 50 days (either need to reset hardware every day, or find other solution)
  long timeSinceLastRecord = currentTime - lastTimeRecorded; // ms

  if (numRecords >= 28800) {
    return; // return if storage capacity is full
  }

  if (lastTimeRecorded == -1) {
    recordNoiseLevel(noiseLevel, 0, currentTime);
    lastTimeRecorded = currentTime;
  } else if (timeSinceLastRecord > RECORD_INTERVAL) {
    recordNoiseLevel(noiseLevel, timeSinceLastRecord, currentTime);
    lastTimeRecorded = currentTime;
  }
}
int baseLevel = 85; // dB (allowed 8hrs at this noise level)
int scaling = 3; // dB (half exposure time for every x dB over baseLevel)
int basePermissibleTime = 8; // the number of hours 

// returns the equivalent number of hours
double calculateEquivalentExposure() {
  double sum = 0; // percentage of way to 8hr X dB equivalent exposure
  for (int i=0; i<numRecords; i++) {
    struct NoiseLevelRecord record = noiseLevelRecords[i];
    sum += computeRecord(record);
  }

  return sum*basePermissibleTime;
}

// get the % of total exposure time for a given record
double computeRecord(struct NoiseLevelRecord record) {
    double permissibleDuration = ((double)basePermissibleTime)/(pow(2, (record.noiseLevel-baseLevel)/scaling));

    double timeSpanHrs = ((double)record.timeSpan)/((double)MS_IN_HR);
    return timeSpanHrs/permissibleDuration;
}


const float FREQ = 1/((float)RECORDS_PER_SECOND); // in Hz
int cumulativeExposureLC = 0; // lc, used for predictNoiseLevel1 only
int timeElapsed = 0; // tc


// predict by assuming current noise level continues for the entire day, drive down towards 85dB. Must assume 8hr day
void adamsPredictAndControl() {
  // TODO separate this function into a recording call (records every X milliseconds) and a prediction function
  // TODO use the actual way of getting 8hr equivalent exposure? Is this it? It should have an option for 3dB or 5dB tradeoff
  double internalNoiseLevel = getInputLevel(); // in dB
  cumulativeExposureLC += (int)(FREQ * pow(10.0, internalNoiseLevel*0.1)); // this is different - I'm using s^-1 here instead of hr^-1

  double remainingTime = totalTime - timeElapsed;
  if (remainingTime < 0 ) {
    remainingTime = 0; // honestly, this is a huge flaw in our prediction idea. We can't always predict the length of each work day without help from the worker
  }
  double projected8hrExposure = 10 * log10((cumulativeExposureLC + remainingTime*pow(10, internalNoiseLevel*0.1))/totalTime);
  if (projected8hrExposure > 85 || internalNoiseLevel > 100) {
    // close
  } else if (projected8hrExposure < 70 || internalNoiseLevel < 65) {
    // open up
  } else {
    // stay in current state
  }
}

// The next several prediction models predict by recording the 8hr equivalent exposure so far, then interpolating those data points
// to get the 8hr equivalent exposure at 8hrs (or whatever the expected operating time is)
// if not enough data points are available, disable prediction (only matters in the first few seconds of operation)

// projecting using beginning of day and current time 
double predictNoiseLevel2() {
  if (numRecords < 2) {
    return -1;
  }

  int firstTime = noiseLevelRecords[0].timeStamp;
  int secondTime = noiseLevelRecords[numRecords].timeStamp;

  double slope = cumulativeExposure[numRecords]/(secondTime - firstTime);

  return totalTime*1000*slope;
}

//take beginning, halfway, and current
double predictNoiseLevel3() {
  return -1;
}

// take beginning, 1/3, 2/3, and current
double predictNoiseLevel4() {
  return -1;
}

