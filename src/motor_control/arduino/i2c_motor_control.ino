#include <Wire.h>

#define SLAVE_ADDRESS1 0x04
#define SLAVE_ADDRESS2 0x05
int number = 0;
int state = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS1);
  //Wire.begin(SLAVE_ADDRESS2);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){
  while(Wire.available()) {
    number = Wire.read();
    
    Serial.println(number);

    digitalWrite(11, LOW);
    digitalWrite(6, LOW);
    analogWrite(5, number);
    analogWrite(10, number);
    
  }
}

// callback for sending data
void sendData(){
  Wire.write(number);
}
