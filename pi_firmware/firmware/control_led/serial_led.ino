
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  if(Serial.available() > 0) {
    String command = Serial.readStringUntil("\n");
    if(command == "1"){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Led ON");
    }else if(command == "0"){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Led OFF");
    }
  }

 
  
}