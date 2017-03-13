#define MIC_IN 15 // Mic in Pin
// Output Pins 
#define LEFT_1 1
#define LEFT_2 2
#define RIGHT_1 3
#define RIGHT_2 4

const int WINDOW = 500; // length of envelope window time in millis
const int STORED_PAST_DB = 10; // The number of records to store (overwrites oldest record). This gives us a 5s memory
const int UPPER_THRESHOLD = 85; // db, when to close
const int LOWER_THRESHOLD = 70; // db, when to open
const int MAX_EXPOSURE = 85; // dB, Maximum daily exposure
// these must be minimum 15dB apart (or based on the actual attenuation of the device) If they're too close, the device will be stuck in an open/shut loop
const double vref = 1; // reference votage (for dB calculation)

int lastResetTime = 0; // time in millis when WINDOW started
double currentMaxDB = 0; // NOTE: this could also be stored in the array. Keeping it separate for now to be explicit
int storedPastDB[STORED_PAST_DB];
int currentIndex = 0; // the current record to write to
int numRecords = 0; // the number of records. Values above STORED_PAST_DB are meaningless
long int secondsElapsed = 0;
int minutesElapsed = 0;
int hoursElapsed = 0;

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
bool isClosed = false;
int vinPrev = 12; // DEBUG VARIABLE (for input simulation)


void setup() {
  Serial.begin(9600);
  lastResetTime = millis();
  Serial.print("Initializing with time stamp: ");
  Serial.println(lastResetTime);
}

void loop() {

  // TODO: Make functional for 2 ears

  // Input
  double Vin = getTestInput();
  double dBspl = V_to_dBspl(Vin);
  // Serial.println(dBspl);

  // Logs the max output over a window
  recordDB(dBspl);
  // Serial.println(currentMaxDB);


  // Calculate the cumulative exposure
  // This only runs every exposureInterval
  calcCumulativeExposure(dBspl); // TODO: synchronize this routine with the recordDB routine

  // Conrol Logic
 if (shouldClose() && !isClosed) {
      closeDevice();
 } else if (shouldOpen() && isClosed) {
      openDevice();
 }

  // Timekeeping - we don't expect drarstically over 8hrs uptime, so only keep track of hours
  if (millis()/1000 - secondsElapsed > 1) {
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

double getInput() {
  double mVin = analogRead(MIC_IN);
  if (mVin < 0) {
    Serial.print("Invalid read!!\n");
  }
   // Serial.println(mVin);
  return mVin;
}

double V_to_dBspl(double vin) {
    return (20*log10(vin/(S_total)));
}

void recordDB(double dB) {
  int currentTime = millis();
  int elapsedTime = currentTime - lastResetTime;

  // Serial.print("Elapsed time: ");
  // Serial.print(elapsedTime);
  // Serial.print("\n");

  // Check if the current level is louder than the previous //
  // If so, then set the max dB for this time window //
  if (dB > currentMaxDB) {
    currentMaxDB = dB;
  }

  // If we've exceeded the window time frame
  // 
  if (elapsedTime >= WINDOW) {
    // Serial.print("Max dB: ");
    // Serial.println(currentMaxDB);
    // Serial.print("Closed: ");
    // Serial.print(isClosed);
    Serial.print("\n##################### Next window ####################################\n");
    lastResetTime = currentTime;
    storedPastDB[currentIndex] = (int)currentMaxDB;
    currentMaxDB = 0;
    currentIndex = (currentIndex+1)%STORED_PAST_DB;

    if (numRecords < STORED_PAST_DB) {
      numRecords++;
    }
  }
}


int shouldClose() {
  // Is the current volume over the threshold?
  if (currentMaxDB > UPPER_THRESHOLD) {
    return true;
  }

  // is any volume in recent memory over the threshold?
  for (int i = 0; i < numRecords; i++) {
    if (storedPastDB[i] > UPPER_THRESHOLD) {
      return true; 
    }
  }

  // If we linearly project the current exposure trend, will we be over 85dB?
  if (LeqLinearProjection() > MAX_EXPOSURE) {
    return true;
  }
  
  // otherwise, we shouldn't close
  return false;
}

int shouldOpen() {

  // if any volume in recent memory over the lower threshold, stay closed
  for (int i = 0; i < numRecords; i++) {
    if (storedPastDB[i] > LOWER_THRESHOLD) {
      return false;
    }
  }

  // If the current volume is over the lower threshold, stay closed
  if (currentMaxDB > LOWER_THRESHOLD) {
    return false;
  }

  // If we linearly project the current exposure trend, will we be over 85dB? - stay closed
  if (LeqLinearProjection() > MAX_EXPOSURE) {
    return false;
  }
  
  return true; // should open if no past sound exceeds the lower threshold (it's been quiet)
}

void openDevice() {
  Serial.print("Opening device\n");
  digitalWrite(LEFT_1, HIGH);
  digitalWrite(LEFT_2, LOW);
  isClosed = false;
}

void closeDevice() {
  Serial.print("Closing device\n");
  digitalWrite(LEFT_1, LOW);
  digitalWrite(LEFT_2, HIGH);
  isClosed = true;
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
void testOutput(){
    Serial.println("Off");
    digitalWrite(LEFT_1, LOW);
    digitalWrite(LEFT_2, LOW);
    delay(500);

    Serial.println("FWD");
    digitalWrite(LEFT_1, HIGH);
    digitalWrite(LEFT_2, LOW);
    delay(500);

    Serial.println("Off");
    digitalWrite(LEFT_1, LOW);
    digitalWrite(LEFT_2, HIGH);
    delay(500);

    Serial.println("BACK");
    digitalWrite(LEFT_1, HIGH);
    digitalWrite(LEFT_2, HIGH);
    delay(500);
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