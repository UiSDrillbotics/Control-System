float x = 0;
float v = 0;
const float pi = 3.1415;
float y = 0;
float y2 = 0;
float y3 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  v = x*pi/180;
  y = sin(v);
  y2 = cos(v);
  y3 = tan(v);
  Serial.print("r");  
  Serial.println(y);
  Serial.print("t");
  Serial.println(y2);
  Serial.print("v");
  Serial.println(y3);
  x = x+2.57;
  delay(100);
}