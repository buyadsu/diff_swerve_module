void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


// // A class to compute the control signal
// class SimplePID{
//   private:
//     float kp, kd, ki, umax; // Parameters
//     float eprev, eintegral; // Storage

//   public:
//   // Constructor
//   SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

//   // A function to set the parameters
//   void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
//     kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
//   }

//   // A function to compute the control signal
//   void evalu(int value, int target, float deltaT, int &pwr, int &dir){
//     // error
//     int e = target - value;
  
//     // derivative
//     float dedt = (e-eprev)/(deltaT);
  
//     // integral
//     eintegral = eintegral + e*deltaT;
  
//     // control signal
//     float u = kp*e + kd*dedt + ki*eintegral;
  
//     // motor power
//     pwr = (int) fabs(u);
//     if( pwr > umax ){
//       pwr = umax;
//     }
  
//     // motor direction
//     dir = 1;
//     if(u<0){
//       dir = -1;
//     }
  
//     // store previous error
//     eprev = e;
//   }
  
// };

// // How many motors
// #define NMOTORS 2
// // Pins
// #define ENCA 2
// #define ENCB 3
// #define PWM 5
// #define IN1 6
// #define IN2 7

// // // globals
// // long prevT = 0;
// // int posPrev = 0;
// // // Use the "volatile" directive for variables
// // // used in an interrupt
// volatile int pos_i = 0;
// volatile float velocity_i = 0;
// volatile long prevT_i = 0;
// volatile float deltaT_i = 0;

// float v2Filt = 0;
// float v2Prev = 0;

// float eintegral = 0;

// // PID class instances
// SimplePID pid[NMOTORS];

// void setup() {
//   Serial.begin(115200);

//   pinMode(ENCA,INPUT);
//   pinMode(ENCB,INPUT);
//   pinMode(PWM,OUTPUT);
//   pinMode(IN1,OUTPUT);
//   pinMode(IN2,OUTPUT);

//   attachInterrupt(digitalPinToInterrupt(ENCA),
//                   readEncoder<0>,RISING);
// }

// void loop() {

//   // read the position and velocity
//   float velocity2 = 0;
//   float deltaT = 0;
//   noInterrupts(); // disable interrupts temporarily while reading
//   // pos = pos_i;
//   velocity2 = velocity_i;
//   deltaT = deltaT_i;
//   interrupts(); // turn interrupts back on

//   long currT = micros();
//   // prevT = currT;

//   // Convert count/s to RPM
//   float v2 = velocity2/600.0*60.0;

//   // Low-pass filter (25 Hz cutoff)
//   v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
//   v2Prev = v2;

//   // Set a target
//   float vt = 100*(sin(currT/1e6)>0);

//   // Compute the control signal u
//   float kp = 5;
//   float ki = 10;
//   float e = vt-v2Filt;
//   eintegral = eintegral + e*deltaT;
  
//   float u = kp*e + ki*eintegral;

//   // Set the motor speed and direction
//   int dir = 1;
//   if (u<0){
//     dir = -1;
//   }
//   int pwr = (int) fabs(u);
//   if(pwr > 255){
//     pwr = 255;
//   }
//   setMotor(dir,pwr,PWM,IN1,IN2);

//   // Serial.print(vt);
//   // Serial.print(" ");
//   // Serial.print(v1Filt);
//   // Serial.println();
//   // delay(1);
// }

// void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
//   analogWrite(pwm,pwmVal); // Motor speed
//   if(dir == 1){ 
//     // Turn one way
//     digitalWrite(in1,HIGH);
//     digitalWrite(in2,LOW);
//   }
//   else if(dir == -1){
//     // Turn the other way
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,HIGH);
//   }
//   else{
//     // Or dont turn
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,LOW);    
//   }
// }

// template <int j>
// void readEncoder() {
//   int b = digitalRead(ENCB);  // Read the second encoder pin to determine direction
//   int increment = (b > 0) ? 1 : -1;        // Determine direction based on the second pin

//   // Update position count for motor `j`
//   pos_i += increment;

//   // Compute velocity with method 2
//   long currT = micros();
//   deltaT_i = ((float) (currT - prevT_i))/1.0e6;
//   velocity_i = increment/deltaT_i;
//   prevT_i = currT;

//   // // Calculate velocity for motor `j`
//   // long currT = micros();
//   // float deltaT = ((float)(currT - prevT[j])) / 1.0e6;  // Time difference in seconds
//   // velocity[j] = increment / deltaT;                     // Counts per second
//   // prevT[j] = currT;                                     // Update previous time for motor `j`
// }