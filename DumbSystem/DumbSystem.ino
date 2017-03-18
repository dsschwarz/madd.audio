#define MIC_IN 15
#define LEFT_1 1
#define LEFT_2 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MIC_IN, INPUT);
  pinMode(LEFT_1, OUTPUT);
  pinMode(LEFT_2, OUTPUT);
} 

void loop() {
  // put your main code here, to run repeatedly:
  double  in = analogRead(MIC_IN);
  Serial.println(in);

  if (in > 50){
      digitalWrite(LEFT_1,HIGH);
      digitalWrite(LEFT_2,LOW);
  } else {
    digitalWrite(LEFT_1,LOW);
    digitalWrite(LEFT_2,HIGH);
  }

  delay(500);
}
