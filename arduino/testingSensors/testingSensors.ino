#define VRx A0    
#define VRy A1
#define B1 10     //Red button connected to D12
#define B2 8     //Blue button connected to D11
int xPosition;
int yPosition;
int zPos;
int offsetX;
int offsetY;
int mapX;
int mapY;



void setup() {
  Serial.begin(115200);
  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  pinMode(B1, INPUT);
  digitalWrite(B1, HIGH);
  pinMode(B2, INPUT);
  digitalWrite(B2, HIGH);
  offsetX = analogRead(VRx);
  offsetY = analogRead(VRy);

}

void loop() {
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  //if the red button is pressed, go up
  if (digitalRead(B1)==LOW && digitalRead(B2)==HIGH){
    zPos = 1;
    }
  //if the blue button is pressed, go down
  else if (digitalRead(B1)==HIGH && digitalRead(B2)==LOW){
    zPos = -1;
    }
  else{
    zPos = 0;
    }
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);

  Serial.println(zPos);
  //Serial.println(B2);

  delay(20);
}
