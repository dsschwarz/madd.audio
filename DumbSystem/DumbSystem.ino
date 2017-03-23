#define MIC_IN 15
#define LEFT_1 0
#define LEFT_2 1
#define RIGHT_1 2
#define RIGHT_2 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LEFT_1, OUTPUT);
  pinMode(LEFT_2, OUTPUT);
  pinMode(RIGHT_1, OUTPUT);
  pinMode(RIGHT_2, OUTPUT);
  
} 

void loop() {
//Serial.println("Opening");
//    digitalWrite(LEFT_1,HIGH);
//    digitalWrite(LEFT_2, LOW);
    digitalWrite(LEFT_1,LOW);
    digitalWrite(LEFT_2, HIGH);
    digitalWrite(RIGHT_1,LOW);
    digitalWrite(RIGHT_2, HIGH);
  delay(5000);
    digitalWrite(LEFT_1,HIGH);
    digitalWrite(LEFT_2, LOW);
    digitalWrite(RIGHT_1,HIGH);
    digitalWrite(RIGHT_2, LOW);
  delay(5000);
}
