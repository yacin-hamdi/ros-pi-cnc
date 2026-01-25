const int stepPinx = 2;
const int dirPinx = 3;
const int stepPiny = 4;
const int dirPiny = 5;

const int stepsPerRevolution = 50;

void setup() {
  Serial.begin(115200);
  pinMode(stepPinx, OUTPUT);
  pinMode(dirPinx, OUTPUT);
  pinMode(stepPiny, OUTPUT);
  pinMode(dirPiny, OUTPUT);

  digitalWrite(stepPinx, LOW);
  digitalWrite(stepPiny, LOW);
  digitalWrite(dirPinx, HIGH);
  digitalWrite(dirPiny, HIGH);
}

void loop() {
 if(Serial.available() > 0) {
  char axis, sign;
  long steps;
  int stepPin, dirPin;
  String cmd = Serial.readStringUntil("\n");
  cmd.trim();
  // Command in the format axis,dir,speed(x,+,100)
  if(sscanf(cmd.c_str(), "%c,%c,%ld", &axis, &sign, &steps) == 3){
     Serial.println("parsed successfully");
     Serial.println(axis);
     Serial.println(sign);
  }

  if(axis == 'x'){
    stepPin = stepPinx;
    dirPin = dirPinx;
    Serial.println("moving axis x");
  } else if(axis == 'y'){
    stepPin = stepPiny;
    dirPin = dirPiny;
    Serial.println("moving axis y");
  }

  bool direction = (sign == '+') ? HIGH: LOW;
  moveStepper(stepPin, dirPin, steps, 1000, direction);
 }
  
}

void moveStepper(
  int stepPin,
  int dirPin,
  long steps,
  int pulse_us,
  bool direction
) {
  digitalWrite(dirPin, direction);

  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulse_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulse_us);
  }
}