// L298N Motor Driver Pins
#define enA 5   
#define in1 6   
#define in2 7   
#define in3 8   
#define in4 9  
#define enB 10  


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    switch (command) {
      case 'd': //Right
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enA, 127); // Adjust speed if needed
        analogWrite(enB, 191);
        break;

      case 'a': // left
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enA, 127);
        analogWrite(enB, 191);
        break;

      case 'b': // back
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enA, 127);
        analogWrite(enB, 191);
        break;

      case 'w': // forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enA, 127);
        analogWrite(enB, 191);
        //Serial.println("Right");
        break;

      case 's': // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        digitalWrite(enA, LOW);
        digitalWrite(enB, LOW);
        break;
    }
  }
}