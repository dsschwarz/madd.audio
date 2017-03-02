#define STORED_PAST_DB 8 // The number of records to store (overwrites oldest record
// these must be minimum 15dB apart (or based on the actual attenuation of the device) If they're too close, the device will be stuck in an open/shut loop
#define UPPER_THRESHOLD 80 // db, when to close
#define LOWER_THRESHOLD 65 // db, when to open
#define INTERVAL 500 // interval time in millis

int lastReset = 0; // time in millis when interval started
double currentMaxDB = 0; // NOTE: this could also be stored in the array. Keeping it separate for now to be explicit
int storedPastDB[STORED_PAST_DB];
int currentIndex = 0; // the current record to write to
int numRecords = 0; // the number of records. Values above STORED_PAST_DB are meaningless



// for testing
int startTime;
int timeAcceleration = 3600; // 8 hours passes in 8 seconds
int isClosed;

double stepper(int secondsElapsed) {
  if (secondsElapsed % 100) {
    return 90; 
  } else {
    return 70;
  }
}

double getInput() {
  int totalSecondsElapsed = (millis() - startTime) * timeAcceleration / 1000; // in seconds

  // call one of the test functions
  double noiseLevel = stepper(totalSecondsElapsed);

  if (isClosed) {
    return noiseLevel - 15;
  } else {
    return noiseLevel;
  }
}

void setup() {
  lastReset = millis();
  Serial.print("Initializing with time stamp: ");
  Serial.println(lastReset);

  // for testing
  startTime = lastReset;
  isClosed = 0;
}

void recordDB(double dB) {
  int currentTime = millis();

  if (dB > currentMaxDB) {
    currentMaxDB = dB;
  }
  
  if (currentTime - lastReset > INTERVAL) {
    lastReset = currentTime;
    storedPastDB[currentIndex] = (int)currentMaxDB;
    currentMaxDB = 0;
    currentIndex = (currentIndex+1)%STORED_PAST_DB;

    if (numRecords < STORED_PAST_DB) {
      numRecords++;
    }
  }
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

void loop() {
  double dB = getInput(); // TODO: implement fake getInput() for testing, otherwise hook up to the old getInput method.
  recordDB(dB);

  if (shouldOpen()) {
    openDevice();
  } else if (shouldClose()) {
    closeDevice();
  }
}
