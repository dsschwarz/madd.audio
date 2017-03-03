#define MIC_IN 14

void setup() {
  Serial.begin(9600);
  
  pinMode(MIC_IN, INPUT);
}

void loop() {

  double  in = analogRead(MIC_IN);
  Serial.println(in);



}
