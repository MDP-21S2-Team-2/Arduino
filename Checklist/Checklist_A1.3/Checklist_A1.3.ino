void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:
    char RPIChar;
    if (Serial.available()>0){
        RPIChar = Serial.read();
        RPIChar += 1;
        Serial.write(RPIChar);
    }
      
}
