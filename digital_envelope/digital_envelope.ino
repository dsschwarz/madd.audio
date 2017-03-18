// #define i/o pins
#define MIC_IN 14 // Mic in is Pin 14

const int STORED_PAST_DB = 8; // The number of records to store (overwrites oldest record)
const int WINDOW = 500; // length of envelope window time in millis
const int UPPER_THRESHOLD = 80; // db, when to close
const int LOWER_THRESHOLD = 65; // db, when to open
// these must be minimum 15dB apart (or based on the actual attenuation of the device) If they're too close, the device will be stuck in an open/shut loop
const double vref = 1; // reference votage (for dB calculation)

int lastResetTime = 0; // time in millis when WINDOW started
double currentMaxDB = 0; // NOTE: this could also be stored in the array. Keeping it separate for now to be explicit
int storedPastDB[STORED_PAST_DB];
int currentIndex = 0; // the current record to write to
int numRecords = 0; // the number of records. Values above STORED_PAST_DB are meaningless

// microphone calibration
// TODO update these values for the new circuit
const double p0 = 0.00002; // Pa8
const double sensitivity = 2.24; //mV/Pa
const double A_filt = 0.502; // A-filter gain
const double R1 = 4658; // Ohms
const double R2 = 98600; //Ohms
const double G = R2/R1; // Pre-amp gain. about 20x for this circuit
const double S_total = p0*sensitivity*A_filt*G; //Overall Gain


// for testing
int startTime;
int timeAcceleration = 3600; // 8 hours passes in 8 seconds
int isClosed;


void setup() {
  lastResetTime = millis();
  Serial.print("Initializing with time stamp: ");
  Serial.println(lastResetTime);

  // for testing
  startTime = lastResetTime;
  isClosed = 0;
}

void loop() {
  double dB = V_to_dB(getInput()); // TODO: implement fake getInput() for testing, otherwise hook up to the old getInput method.
  Serial.print("dB is: ");
  Serial.println(dB);
  recordDB(dB);

  if (shouldOpen()) {
    openDevice();
  } else if (shouldClose()) {
    closeDevice();
  }
}

double getInput() {
  
  double mVin = analogRead(MIC_IN);
  Serial.print("mVin: ");
  Serial.println(mVin);
  
  if (mVin <= 0) {
    Serial.println("Invalid read!!");
    Serial.print(mVin);
  }
  double dB = (20*log10(mVin/(S_total)));
//  dB = 100;
  return dB;
}

double testGetInput() {
  int totalSecondsElapsed = (millis() - startTime) * timeAcceleration / 1000; // in seconds

  // call one of the test functions
  double noiseLevel = stepper(totalSecondsElapsed);

  if (isClosed) {
    return noiseLevel - 15;
  } else {
    return noiseLevel;
  }  
}

double V_to_dB(double vin) {
  return 20 * log10(vin/vref);
}

void recordDB(double dB) {
  int currentTime = millis();

  // Check if the current level is louder than the previous 
  if (dB > currentMaxDB) {
    currentMaxDB = dB;
  }

  Serial.print("Elapsed time: ");
  Serial.println(currentTime - lastResetTime);
  if ((currentTime - lastResetTime) > WINDOW) {
    Serial.println("##################### Next window #####################################");
    lastResetTime = currentTime;
    storedPastDB[currentIndex] = (int)currentMaxDB;
    currentMaxDB = 0;
    currentIndex = (currentIndex+1)%STORED_PAST_DB;

    if (numRecords < STORED_PAST_DB) {
      numRecords++;
    }
  }

  Serial.print("Current max db: ");
  Serial.println(currentMaxDB);

  delay(10);
}


int shouldClose() {
  for (int i = 0; i < numRecords; i++) {
    if (storedPastDB[i] > UPPER_THRESHOLD) {
      return 1; // should close if any past sound has been too loud
    }
  }

  if (currentMaxDB > UPPER_THRESHOLD) {
    return 1;
  }
  
  return 0;
}

int shouldOpen() {
  for (int i = 0; i < numRecords; i++) {
    if (storedPastDB[i] > LOWER_THRESHOLD) {
      return 0;
    }
  }

  if (currentMaxDB > LOWER_THRESHOLD) {
    return 0;
  }
  
  return 1; // should open if no past sound exceeds the lower threshold (it's been quiet)
}

void openDevice() {
  Serial.println("Opening device");
  isClosed = 0;
}

void closeDevice() {
  Serial.println("Closing device");
  isClosed = 1;
}

double stepper(int secondsElapsed) {
  if (secondsElapsed % 100) {
    return 90; 
  } else {
    return 70;
  }
}

