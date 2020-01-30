#define MagF            51                     // electromagnet pin

#define InL1            13                      // motor pin
#define InL2            9                       // motor pin  

#define InR1            7                       // motor pin
#define InR2            4                       // motor pin 

void setup(){
  // Initialize the Serial Monitor
  Serial.begin(9600);
  // Initialize the pin-mode of motor pins 
  motorInit();
  // Initialize the pin-mode of electromagnet pin
  MagInit();
}
// data array which stores the data frame received by the receiver Xbee
unsigned char data[15];

/**
 * control array stores the direction of the bot
 * control[0]: 1->forward, 0->neutral & -1->backward 
 * control[1]: 1->left, 0->neutral & -1->right
 */
int control[2];

void loop(){
  
  bool check =  readXBee(data); // read the data from the XBee if it is successful check = true else check = false  
  
  getJoystickData(control, data); // Process the data-frame received and determine appropriate values for control[0] & control[1]
  
  controlMotors(control); // Give command to the motor driver according to the values in control array
  
  controlMagnet(data); 
  
  if(check) // Debugging message
    Serial.println("Success");  
  
  delay(1000);
}

/**
 * Function Name: readXBee
 * Functionality: To read the data from the XBee and update values in data array
 * Arguments: unsigned char array data[15]
 * Return Value: bool denoting whether the reading process was successful or not
 * Example: readXBee(data)
 */
bool readXBee(unsigned char data[15]){
  // Init temporary array to store the data frame
  unsigned char tmpData[15], tmpByte;
  int checkSum = 0x83;
  // Discard all the bytes until the occurance of first 0x7E  
  while(Serial.available() && Serial.read() != 0x7E);
  // The full data frame length must be available
  if (Serial.available() < 19)  return false;
  // Check the length of the data frame, it must be 16 bytes long
  tmpByte = Serial.read();
  if (tmpByte != 0x00)  return false;
  tmpByte = Serial.read();
  if (tmpByte != 0x10)  return false;
  // Check the frame type
  tmpByte = Serial.read();
  if (tmpByte != 0x83)  return false;
  // Read 15 bytes and store it in the array
  for (int i = 0; i < 15; ++i){
    tmpData[i] = Serial.read();
    checkSum += (int)tmpData[i];
  }
  // Read checkSum byte
  tmpByte = Serial.read();
  
  // Flush the serial
  while(Serial.available())
    Serial.read();

  // Check whether the checkSum obtained by our calculation (stored as 8byte int) 
  // is same as the last byte on the same frame
  if (tmpByte == 0xFF - (0xFF & (unsigned char)checkSum)){
    // Copy tmpData to data array as the reading was successful
    for (int i = 0; i < 15; ++i){
      data[i] = tmpData[i];
    }
    return true;
  }
  else return false;
}

/**
 * Function Name: controlMotors
 * Functionality: To control the motors according to the control array
 * Arguments: int array control[2]
 * Return Value: None
 * Example: controlMotors(control)
 */
void controlMotors(int control[2]){
  if(control[0] == 1){ // Forward
    motorForwardL();
    motorForwardR();
  }
  else if(control[0] == -1){ // Backward
    motorBackwardL();
    motorBackwardR();
  }
  else if(control[1] == -1){ // Right
    motorForwardL();
    motorBackwardR();
  }
  else if(control[1] == 1){ // Left
    motorBackwardL();
    motorForwardR();
  }
  else{ //Stop
    motorStopL();
    motorStopR(); 
  }
}

/**
 * Function Name: controlMagnet
 * Functionality: To control the magnet according to the data array
 * Arguments: unsigned char array data[15]
 * Return Value: None
 * Example: controlMagnet(data)
 */
void controlMagnet(unsigned char data[15]){
  // Use bit-masking to get the value of DO3 pin on XBee
  int eMState = data[8] & 8 ;
  if(eMState == 0)
    MagPick();
  else
    MagDrop();
}

/**
 * Function Name: getJoystickData
 * Functionality: convert the analog joystick values to control array
 * Arguments: int array control[2] & unsigned char array data[15]
 * Return Value: None
 * Example: getJoystickData(control, data)
 */
void getJoystickData(int control[2], unsigned char data[15]){
  // Calculate the Analog value at AO1 using two bytes data[11] (LSB) and data[12] (MSB)
  int AD1 = data[11]*255+data[12];
  // Limiting AD1
  if(AD1 > 1023) AD1 = 1023;
  
  // Calculate the Analog value at AO2 using two bytes data[13] (LSB) and data[14] (MSB)
  int AD2 = data[13]*255+data[14];
  // Limiting AD2
  if(AD2 > 1023) AD2 = 1023;

  // Thresolding 
  if(AD1 >= 400 && AD1 <= 750) control[0] = 0;
  else if(AD1 < 400) control[0] = 1;
  else if(AD1 > 750) control[0] = -1;

  // Debugging statements
  Serial.print("C1 = ");   
  Serial.print(control[0]);
  Serial.print(" ");
  Serial.print("AD1 = ");    
  Serial.println(AD1);

  // Thresolding
  if(AD2 >= 400 && AD2 <= 750) control[1] = 0;
  else if(AD2 < 400) control[1] = 1;
  else if(AD2 > 750) control[1] = -1;
  
  // Debugging statements
  Serial.print("C2 = ");    
  Serial.print(control[1]);
  Serial.print(" ");
  Serial.print("AD2 = ");    
  Serial.println(AD2);
}

// ==================== HELPER FUNCTIONS ===========================

// Initialize the pin-modes of the motor-pins (InL1, InL2, InR1, InR2)
void motorInit(){
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
}

// Set the left motor to move forward
void motorForwardL(){
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

// Set the right motor to move forward
void motorForwardR(){
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}

// Set the left motor to move backward
void motorBackwardL(){
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}

// Set the right motor to move backward
void motorBackwardR(){
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

// Stop left motor
void motorStopL(){
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, LOW);
}

// Stop right motor
void motorStopR(){
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}

// Initialize the pin-mode of electromagnet pin MagF and initialize it to LOW(0V)
void MagInit(){
  pinMode(MagF, OUTPUT);
  digitalWrite(MagF, LOW);
}

// switch on the electromagnet
void MagPick(void)  {
  digitalWrite(MagF, HIGH);
}

// switch off the electromagnet
void MagDrop(void)  {
  digitalWrite(MagF, LOW);
}
