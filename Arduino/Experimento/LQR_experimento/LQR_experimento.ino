// I2C libray communication
#include <Wire.h>

// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>

// ESP32 BLUE LED pin
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// NIDEC PWM config
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

// NIDEC pins: Reaction Wheel Motor
#define BRAKE1            14 //Yellow wire (Start/Stop)
#define NIDEC1_PWM        27 //Write wire  (PWM)
#define DIR1              16 //Green wire  (Forward/Reverse) 
#define ENCA_1            17 //Brown wire  (Signal A)  
#define ENCB_1            25 //Orange      (Signal B)  
#define NIDEC1_PWM_CH      1

// NIDEC pins: Traction Wheel Motor
#define BRAKE2           18 //Yellow wire (Start/Stop)
#define NIDEC2_PWM       19 //Write wire  (PWM)
#define DIR2             23 //Green wire  (Forward/Reverse) 
#define ENCA_2           5  //Brown wire // purple (Signal A)
#define ENCB_2           13 //Orange   (Signal B)
#define NIDEC2_PWM_CH     0

// Encoder vars
ESP32Encoder NIDEC1_ENC;
ESP32Encoder NIDEC2_ENC;

// Kalman Filter vars
float Q_angle = 0.001; // Angular data confidence
float Q_bias  = 0.005; // Angular velocity data confidence
float R_meas  = 1.0;
float roll_angle = 0.0;
float bias = 0.0;
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// Control vars
//         theta        dthehta         psi            dpsi
float K1 = -66,    K2 = -3.8,      K3 = -0.71,      K4 = -2.4;
float U = 0;
int pwm = 0;
float theta = 0, theta_dot = 0;            // System states
float psi = 0, psi_dot = 0;
float Ts = 0.01, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function
float Tc = 20, currentTc = 0.0, previousTc = 0.0;
float pulse2rad = 2*3.1415/3630;
bool start = true;
float disturbio = 0;

#define DADOSMAX 3000
float dados[DADOSMAX][7];
int count = 0;

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);                   // make sure your Serial Monitor is also set at this baud rate.

  NIDECsetup();
  IMUsetup();

  pinMode(INTERNAL_LED,OUTPUT);
  digitalWrite(INTERNAL_LED,HIGH);  // Turn on blue led
  delay(1500);                      // Wait for the system to stabilize
  for (int i=1; i<= 400; i++){      // Wait for the Kalman filter stabilize
    IMUread();
    delay(5);
  }
  currentT = millis();
  digitalWrite(INTERNAL_LED,LOW);  
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:
  equilibra();
  printa();
}

void equilibra() {

  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;
    
    IMUread();

    if (abs(roll_angle) < 2) start = true;

    if (abs(roll_angle) < 20 && start) {
      digitalWrite(BRAKE1, HIGH);
      digitalWrite(BRAKE2, HIGH);

      theta += theta_dot * Ts;
      psi_dot = NIDEC1_ENC.getCount()*pulse2rad/Ts;
      psi += psi_dot*Ts;
           
      U = K1*theta + K2*theta_dot + K3*psi + K4*psi_dot;
      if (currentT/1000.0 > 21 && currentT/1000.0 < 91.5) disturbio = 5;
      // else disturbio = 0;
      pwm = (U + disturbio)*21.3;
      if (NIDEC1_ENC.getCount() == 0) pwm += random(20)-10; // sometimes the motor stops
      MOTOR1cmd(-pwm);

      NIDEC1_ENC.clearCount();       

    } else if (start) {
      digitalWrite(BRAKE1, LOW); // stop reaction wheel
      digitalWrite(BRAKE2, LOW); // stop traction wheel
      digitalWrite(INTERNAL_LED,HIGH);
      delay(5000);       // time to lift the bike
      digitalWrite(INTERNAL_LED,LOW);
      for (int i=1; i<= 400; i++){ //Wait for the Kalman Filter stabilize
        IMUread();
        delay(5);
      }
      previousT = millis();
      theta = 0.0;
      psi = 0.0;
      start = false;
      NIDEC1_ENC.clearCount();
    }

    if (currentT/1000.0 > 20) {
      dados[count][0] = currentT/1000.0;
      dados[count][1] = theta;
      dados[count][2] = theta_dot;
      dados[count][3] = psi;
      dados[count][4] = psi_dot;
      dados[count][5] = U;
      dados[count][6] = disturbio;
      if(count < DADOSMAX-1) count++;
    }
    
  }   

}

void printa(){
  currentTc = millis();
  if ((currentTc - previousTc)/1000.0 >= Tc && currentTc/1000.0 > 65) {
    previousTc = currentTc;

    for (int i = 0; i < DADOSMAX; i++) {
      for (int j = 0; j < 7; j++) { // Mostrando apenas os 10 primeiros elementos por linha
        Serial.print(dados[i][j], 8);
        if(j != 6) Serial.print(" ");      
      }
      Serial.println();
    }

    Serial.println("===");

  }
}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

void NIDECsetup(){
  pinMode(BRAKE1, OUTPUT);
  digitalWrite(BRAKE1, HIGH);

  pinMode(BRAKE2, OUTPUT);
  digitalWrite(BRAKE2, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(NIDEC1_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC1_PWM, NIDEC1_PWM_CH);
  MOTOR1cmd(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(NIDEC2_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC2_PWM, NIDEC2_PWM_CH);
  MOTOR2cmd(0);

  // Encoders setup:
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  // Encoder 1:
	NIDEC1_ENC.attachFullQuad(ENCB_1, ENCA_1);
  NIDEC1_ENC.clearCount();
  // Encoder 2:
	NIDEC2_ENC.attachFullQuad(ENCB_2, ENCA_2);
  NIDEC2_ENC.clearCount();
}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  ax=Wire.read()<<8|Wire.read();   // X-axis value: 16384.0; 
  ay=Wire.read()<<8|Wire.read();   // Y-axis value: 16384.0;     
  az=Wire.read()<<8|Wire.read();   // Z-axis value: 16384.0;  
  temp=Wire.read()<<8|Wire.read();      
  gx=Wire.read()<<8|Wire.read();  
  gy=Wire.read()<<8|Wire.read();  
  gz=Wire.read()<<8|Wire.read();  
  //accelerometer angles in degrees (or rads)
  float ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  //float ay_angle = atan2(ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  // float az_angle = atan2(sqrt(ax*ax + az*az), az) * 57.3; // yaw (useless)
  // gyro measurements in degress (or rads)
  gx =  gx / 131; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gy =  gy / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gz =  gz / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  
  // begin: Kalman filter - Roll Axis (X)
  roll_angle += (gx - bias) * Ts;
  
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //
  roll_angle += K[0] * (ax_angle - roll_angle); 
  bias       += K[1] * (ax_angle - roll_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 

  theta_dot = (gx - bias)/57.3; // Unbiased gyro speed
  
  // //  Complementary filter   
  // roll_anglec = 0.98 * (roll_anglec + gx * Ts) + 0.02 * ax_angle;
  // pitch_anglec = 0.98 * (pitch_anglec + gy * Ts) + 0.02 * ay_angle;
  // yaw_anglec = 0.98 * (yaw_anglec + gz * Ts) + 0.02 * az_angle;

}  

// NIDEC functions
void MOTOR1cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR1, LOW);
  }
  ledcWrite(NIDEC1_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void MOTOR2cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR2, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR2, LOW);
  }
  ledcWrite(NIDEC2_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}
