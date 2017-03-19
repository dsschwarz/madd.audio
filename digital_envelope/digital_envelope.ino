// #define i/o pins
#define MIC_IN 15 // Mic in Pin
// Output Pins 
#define LEFT_1 1
#define LEFT_2 2
#define RIGHT_1 3
#define RIGHT_2 4

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
int gnd_ref = 0;

int vinPrev = 10; // DEBUG VARIABLE (for input simulation)

// microphone calibration
const double p0 = 0.00002; // Reference Pressure
const double sensitivity = 1000 * pow(10,(-53/20)); //in mV/Pa - convert the sensitivity in dBV/Pa to mV/Pa
const double A_filt = 0.502; // A-filter gain
const double R1 = 4658; // Ohms
//const double R2 = 179500; //Ohms
const double R2 = 98600; //Ohms
const double G = 1 + R2/R1; // Pre-amp gain
const double S_total = p0*sensitivity*A_filt*G; //Overall Gain


// for testing
int startTime;
int timeAcceleration = 3600; // 8 hours passes in 8 seconds
int isClosed;


void setup() {
  Serial.begin(9600);
  lastResetTime = millis();
  Serial.print("Initializing with time stamp: ");
//  Serial.println(lastResetTime);
  // for testing
  startTime = lastResetTime;
  isClosed = 0;
  gnd_ref = analogRead(GND);
  
}

void loop() {
  double dB = V_to_dB(getTestInput()); // TODO: implement fake getInput() for testing, otherwise hook up to the old getInput method.
//  Serial.print("dB is: ");
  Serial.println(dB);
  recordDB(dB);

  testOutput();

//  if (shouldOpen()) {
//    openDevice();
//  } else if (shouldClose()) {
//    closeDevice();
//  }
}

double getInput() {
  gnd_ref = analogRead(GND);
  double mVin = analogRead(MIC_IN);
  if (mVin < 0) {
    Serial.println("Invalid read!!");
  }
//   Serial.println(gnd_ref);
   Serial.println(mVin);
  return mVin;
}

double getTestInput() {
//  int totalSecondsElapsed = (millis() - startTime) * timeAcceleration / 1000; // in seconds
  int vin = vinPrev + random(-2, 3);
  if (vin < 0 || vin > 150) {
    vin = 10;
  }
  vinPrev = vin;
//  Serial.println(vin);
  if (isClosed)
    vin -= 2;
  return vin;
}

void testOutput(){
    Serial.println("Off");
    digitalWrite(LEFT_BIT_1, LOW);
    digitalWrite(LEFT_BIT_3, LOW);
    delay(500);

    Serial.println("FWD");
    digitalWrite(LEFT_BIT_1, HIGH);
    digitalWrite(LEFT_BIT_3, LOW);
    delay(500);

    Serial.println("Off");
    digitalWrite(LEFT_BIT_1, LOW);
    digitalWrite(LEFT_BIT_3, HIGH);
    delay(500);

    Serial.println("BACK");
    digitalWrite(LEFT_BIT_1, HIGH);
    digitalWrite(LEFT_BIT_3, HIGH);
    delay(500);
}

double V_to_dB(double vin) {
//  return 20 * log10(vin/vref);
    return (20*log10(vin/(S_total)));
}

void recordDB(double dB) {
  int currentTime = millis();
  // Check if the current level is louder than the previous 
  if (dB > currentMaxDB) {
    currentMaxDB = dB;
  }

//  Serial.print("Elapsed time: ");
//  Serial.println(currentTime - lastResetTime);
  if ((currentTime - lastResetTime) > WINDOW) {
//    Serial.println("##################### Next window #####################################");
    lastResetTime = currentTime;
    storedPastDB[currentIndex] = (int)currentMaxDB;
    currentMaxDB = 0;
    currentIndex = (currentIndex+1)%STORED_PAST_DB;

    if (numRecords < STORED_PAST_DB) {
      numRecords++;
    }
  }

//  Serial.print("Current max db: ");
//  Serial.println(currentMaxDB);
//  delay(10);
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
  digitalWrite(LEFT_BIT_1, HIGH);
  digitalWrite(LEFT_BIT_3, LOW);
  isClosed = 0;
}

void closeDevice() {
  Serial.println("Closing device");
  digitalWrite(LEFT_BIT_1, LOW);
  digitalWrite(LEFT_BIT_3, HIGH);
  isClosed = 1;
}

// removed "stepper" function - Adam

