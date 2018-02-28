int count = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("r");
  Serial.println(count);
  Serial.print("t");
  Serial.println(count+10);
  Serial.print("v");
  Serial.println(count+100);
  count=count+1;
  if (count==50){
    count= 0;
  }
  delay(500);
}
