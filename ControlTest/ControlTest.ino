#define LEFT_1 2
#define LEFT_2 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEFT_1, OUTPUT);
  pinMode(LEFT_2, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(LEFT_1,HIGH);
  digitalWrite(LEFT_2,LOW);
  delay(1000);

  digitalWrite(LEFT_1,LOW);
  digitalWrite(LEFT_2,HIGH);
  delay(1000);
}
