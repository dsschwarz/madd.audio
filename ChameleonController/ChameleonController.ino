#define LEFT_MIC_IN 15 // Mic in Pin
#define RIGHT_MIC_IN 16 // Mic in Pin TODO dschwarz
// Output Pins 
#define LEFT_1 0
#define LEFT_2 1
#define RIGHT_1 2
#define RIGHT_2 3
//#define DEMO_PIN_OPEN 5
//#define DEMO_PIN_CLOSE 6

const int WINDOW = 500; // length of envelope window time in millis
const int STORED_PAST_DB = 10; // The number of records to store (overwrites oldest record). This gives us a 5s memory
const int UPPER_THRESHOLD = 85; // db, when to close
const int LOWER_THRESHOLD = 75; // db, when to open
const int MAX_EXPOSURE = 85; // dB, Maximum daily exposure
// these must be minimum 15dB apart (or based on the actual attenuation of the device) If they're too close, the device will be stuck in an open/shut loop
const double vref = 1; // reference votage (for dB calculation)
const int DEACTIVATION_DELAY = 400; // After the device closes, time in millis before turning off driver. This is intentionally high for the demo, since battery life is not too much of an issue, but we really need to be sure the device is closed


struct EarData {
  int micPin;
  int output1;
  int output2;
  
  int timeLastClosed = 0; // time in millis when the device was last closed. After 100ms, the driver should be turned off
  int lastResetTime = 0; // time in millis when WINDOW started
  double currentMaxDB = 0; // NOTE: this could also be stored in the array. Keeping it separate for now to be explicit
  int storedPastDB[STORED_PAST_DB];
  int currentIndex = 0; // the current record to write to
  int numRecords = 0; // the number of records. Values above STORED_PAST_DB are meaningless
  bool isClosed = true; // true if the device has been closed more recently than it has been opened (doesn't gaurantee the plug is sealed yet though)
  bool isDeactivated = true; 
  
} leftEar, rightEar;

long int secondsElapsed = 0;
int minutesElapsed = 0;
int hoursElapsed = 0;

// NOTE: Deprecated for demo
unsigned long int ce = 0; // strictly increasing numerator for calculating cumulativeExposure
double cumulativeExposure = 0;
double pastExposure = 0; // The most recent cumulativeExposure
int exposureInterval = 5; // Minutes. How often we recalculate cumulativeExposure

// microphone calibration
const double p0 = 0.00002; // Reference Pressure
const double sens = 1000 * pow(10,(-53/20)); //in mV/Pa - convert the sensitivity in dBV/Pa to mV/Pa
const double A_filt = 0.502; // A-filter gain
const double R1 = 4658; // Ohms
const double R2 = 98600; //Ohms
const double G = 1 + R2/R1; // Pre-amp gain
const double S_total = p0*sens*A_filt*2*G; //Overall Gain


// for testing
int startTime;
int timeAcceleration = 3600; // 8 hours passes in 8 seconds
int vinPrev = 12; // DEBUG VARIABLE (for input simulation)


void setup() {
  Serial.begin(9600);
  leftEar.lastResetTime = millis();
  rightEar.lastResetTime = millis();
  Serial.print("Initializing with time stamp: ");
  Serial.println(leftEar.lastResetTime);

  pinMode(LEFT_MIC_IN, INPUT);
  pinMode(RIGHT_MIC_IN, INPUT);
  
  pinMode(LEFT_1, OUTPUT);
  pinMode(LEFT_2, OUTPUT);
  pinMode(RIGHT_1, OUTPUT);
  pinMode(RIGHT_2, OUTPUT);
  
//  pinMode(DEMO_PIN_OPEN,INPUT_PULLUP);
//  pinMode(DEMO_PIN_CLOSE,INPUT_PULLUP);

  leftEar.micPin = LEFT_MIC_IN;
  leftEar.output1 = LEFT_1;
  leftEar.output2 = LEFT_2;

  rightEar.micPin = RIGHT_MIC_IN;
  rightEar.output1 = RIGHT_1;
  rightEar.output2 = RIGHT_2;  

  Serial.println("## BEGINNING TEST ##");
  //openDevice(&leftEar);
  
  testOutput(&rightEar);
  testOutput(&leftEar);
}

void loop() {
  int currentTime = millis();

//  if (digitalRead(DEMO_PIN_OPEN) == LOW) {
//    Serial.println("## OVERRIDE open begin ##");
//    while (digitalRead(DEMO_PIN_OPEN) == LOW) {
//      openDevice(&rightEar);
//      openDevice(&leftEar);
//    }
//    Serial.println("## OVERRIDE open end ##");
//  }
//
//  if (digitalRead(DEMO_PIN_CLOSE) == LOW) {
//    Serial.println("## OVERRIDE close begin ##");
//    while (digitalRead(DEMO_PIN_CLOSE) == LOW) {
//      closeDevice(&rightEar);
//      closeDevice(&leftEar);
//    }
//    Serial.println("## OVERRIDE close end ##");
//  }
  
  controlEar(&leftEar);
  controlEar(&rightEar);

  // Timekeeping - we don't expect drarstically over 8hrs uptime, so only keep track of hours
  if (currentTime/1000 - secondsElapsed > 1) {
    secondsElapsed++;
      if (secondsElapsed > 60) {
        minutesElapsed++;
        secondsElapsed = 0;
        if (minutesElapsed > 60) {
          hoursElapsed++;
          minutesElapsed = 0;
      }
    }
  }

}

void controlEar(struct EarData* earData) {  
  double Vin = getInput(earData->micPin);
  double dBspl = V_to_dBspl(Vin);
  int currentTime = millis();

  // Logs the max output over a window
  recordDB(dBspl, currentTime, earData);


  // Calculate the cumulative exposure
  // This only runs every exposureInterval
  // DISABLED FOR DEMO
  // calcCumulativeExposure(dBspl); // TODO: synchronize this routine with the recordDB routine

  // Conrol Logic
 if (!earData->isClosed && shouldClose(earData)) {
      closeDevice(earData);
 } else if (earData->isClosed && shouldOpen(earData)) {
      openDevice(earData);
 } else if (!earData->isDeactivated && earData->isClosed && currentTime - earData->timeLastClosed > DEACTIVATION_DELAY) {
  // if device closed 200ms ago, turn off driver to save power
  deactivateDriver(earData);
 }
}

double getInput(int pin) {
  double mVin = analogRead(pin);
  if (mVin < 0) {
    Serial.print("Invalid read!!\n");
  }
   // Serial.println(mVin);
  return mVin;
}

double V_to_dBspl(double vin) {
    return (20*log10(vin/(S_total)));
}

void recordDB(double dB, int currentTime, EarData* earData) {
  int elapsedTime = currentTime - earData->lastResetTime;

  // Serial.print("Elapsed time: ");
  // Serial.print(elapsedTime);
  // Serial.print("\n");

  // Check if the current level is louder than the previous //
  // If so, then set the max dB for this time window //
  if (dB > earData->currentMaxDB) {
    earData->currentMaxDB = dB;
  }

  // If we've exceeded the window time frame
  if (elapsedTime >= WINDOW) {
    Serial.print("Stats for ear with mic: ");
    Serial.println(earData->micPin);
    Serial.print("Max dB: ");
    Serial.println(earData->currentMaxDB);
    Serial.print("Closed: ");
    Serial.println(earData->isClosed);
//    Serial.print("Projected Exposure: ");
//    Serial.println(LeqLinearProjection());
    Serial.print("\n##################### Next window ####################################\n");
    earData->lastResetTime = currentTime;
    earData->storedPastDB[earData->currentIndex] = (int)earData->currentMaxDB;
    earData->currentMaxDB = 0;
    earData->currentIndex = (earData->currentIndex+1)%STORED_PAST_DB;

    if (earData->numRecords < STORED_PAST_DB) {
      earData->numRecords++;
    }
  }
}


bool shouldClose(EarData* earData) {
  // Is the current volume over the threshold?
  if (earData->currentMaxDB > UPPER_THRESHOLD) {
    return true;
  }

  // is any volume in recent memory over the threshold?
  for (int i = 0; i < earData->numRecords; i++) {
    if (earData->storedPastDB[i] > UPPER_THRESHOLD) {
      return true; 
    }
  }

// DISABLED FOR DEMO
  // If we linearly project the current exposure trend, will we be over 85dB?
//  if (LeqLinearProjection() > MAX_EXPOSURE) {
//    return true;
//  }
  
  // otherwise, we shouldn't close
  return false;
}

bool shouldOpen(EarData* earData) {

  // if any volume in recent memory over the lower threshold, stay closed
  for (int i = 0; i < earData->numRecords; i++) {
    if (earData->storedPastDB[i] > LOWER_THRESHOLD) {
      return false;
    }
  }

  // If the current volume is over the lower threshold, stay closed
  if (earData->currentMaxDB > LOWER_THRESHOLD) {
    return false;
  }

  // If we linearly project the current exposure trend, will we be over 85dB? - stay closed
//  if (LeqLinearProjection() > MAX_EXPOSURE) {
//    return false;
//  }
  
  return true; // should open if no past sound exceeds the lower threshold (it's been quiet)
}

void openDevice(EarData* earData) {
  Serial.print("Opening device\n");
  digitalWrite(earData->output1, HIGH);
  digitalWrite(earData->output2, LOW);
  earData->isClosed = false;
  earData->isDeactivated = false;
}

void closeDevice(EarData* earData) {
  Serial.print("Closing device\n");
  digitalWrite(earData->output1, LOW);
  digitalWrite(earData->output2, HIGH);
  earData->isClosed = true;
  earData->isDeactivated = false;
  earData->timeLastClosed = millis();
}

void deactivateDriver(EarData* earData) {
  Serial.print("Deactivating driver\n");
  digitalWrite(earData->output1, LOW);
  digitalWrite(earData->output2, LOW);
  earData->isDeactivated = true;
}

// Randomly varies the input signal
double getTestInput() {
  int vin = vinPrev + random(-5, 5);
  if (vin < 0 || vin > 200) {
    vin = 12;
  }
  vinPrev = vin;
  return vin;
}

// Steps through all output options
void testOutput(EarData* earData){
    Serial.println("Off");
    deactivateDriver(earData);
    delay(1000);

    Serial.println("FWD");
    openDevice(earData);
    delay(2000);

    Serial.println("BACK");
    closeDevice(earData);
    delay(2000);

    Serial.println("Off");
    deactivateDriver(earData);
    delay(2000);
}

int pseudoGaussian(int n, int range) {
  int rand = 0;
  for (int i = 0; i <= n; i++)
  {
    rand += random(-range, range);
  }
  return rand;
}

void calcCumulativeExposure(double dB){
  // run this every <exposureInterval> minutes
  if (minutesElapsed > 0  
      && (minutesElapsed % exposureInterval == 0) 
        && secondsElapsed == 0) 
  {
    pastExposure = cumulativeExposure;
    ce += exposureInterval * pow(10, (0.1 * dB));
    cumulativeExposure = 10*log10(ce/minutesElapsed);
  }
}
// Returns the linearly projected Leq given the past two inputs
double LeqLinearProjection() {
  double slope = (pastExposure - cumulativeExposure)/ exposureInterval;
  double yint = cumulativeExposure - (slope * minutesElapsed);
  return slope * (8 * 60 - minutesElapsed) + yint;
}
