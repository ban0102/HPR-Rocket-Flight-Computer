// Supports only Teensy 4.1

#include <PWMServo.h>
#include "declarations.h"

//Servo Setup
PWMServo canardYaw1; 
PWMServo canardYaw2;
PWMServo canardPitch3;
PWMServo canardPitch4;
PWMServo actionServo5;
PWMServo actionServo6;
PWMServo actionServo7;
PWMServo actionServo8;

void beep(uint8_t pin_value);
void SetupPins();
void CheckIfTestModeEnabled();
void SetupADC();
void SetupSensors();
void PrintFlightProfile();
void PrintBuzzerEnabledInfo();
void ReadBatteryVoltage();
void PrintBatteryVoltage();
void SetupServos();
void PrintSensorCalibrationValuesFromEEPROM();
void ReadSensorsForSomeTime();
void InitActiveStabilizationSystem();

//any routines called in the main flight loop need to be as fast as possible, so set them as "always inline" for the compiler
inline void getDCM2DRotn(void) __attribute__((always_inline));
inline void getQuatRotn(void) __attribute__((always_inline));

void setup() {
  Serial.begin(38400);
  delay(500);
  Serial.println(F("Setup - START"));

  rocketState.SetRocketState(ROCKET_IDLE);
  SetupPins();
  CheckIfTestModeEnabled();
  delay(50);
  PrintFlightProfile();
  SetupADC();
  PrintBuzzerEnabledInfo();
  PrintBatteryVoltage();
  SetupSensors();
  SetupServos();
  PrintSensorCalibrationValuesFromEEPROM();

  if(settings.testMode){Serial.println(F("Hold Rocket Vertical"));}
  //wait for the rocket to be installed vertically
  //beep(HIGH);
  //delay(settings.setupTime);  // 10 seconds the set the rocket in vertical position
  //beep(LOW);
  //delay(500);

  ReadSensorsForSomeTime();
  InitActiveStabilizationSystem();
    
  Serial.println(F("Setup - END"));  // Go to waiting for arming and launch
}//end setup

void loop(){
    //caluclate the partial rotation
    dx += gyro.x * fltTime.dt;
    dy += gyro.y * fltTime.dt;
    dz += gyro.z * fltTime.dt;

    //if active stablization is on, then use the more stable rotation algorithm
    if(settings.stableRotn || settings.stableVert){
      getDCM2DRotn(dx, dy, dz, gyro.gainZ); 
      dx = dy = dz = 0L;
    }
    
    //update the quaternion rotation if we are not using active stabilization
    else if(fltTime.timeCurrent - lastRotn > rotnRate){
      getQuatRotn(dx, dy, dz, gyro.gainZ);
      dx = dy = dz = 0L;
      lastRotn = fltTime.timeCurrent;
    }
 
    //run event logic
    //checkEvents();

    //update the canards throws if stabilization enabled
    //if(settings.stableRotn || settings.stableVert){
      //uint32_t controlTime = micros();
      //if (controlTime - timeLastControl >= controlInterval) {
        //if active stabilization is activated, set the canards
        //if((settings.stableVert || settings.stableRotn) /*&& !events.apogee && events.boosterBurnout*/){setCanards();}
        //if RTB is on and we are post apogee, then set canards to return to launch point
        //if(settings.flyBack && events.apogeeFire /*&& !events.mainDeploy*/){setRTB();}
        //update control timer
        //timeLastControl = controlTime;}}
    
    /*
    //Read the battery voltage
    if((fltTime.timeCurrent - lastVolt > voltRate) || pyroFire){
      voltReading = analogRead(pins.batt);
      voltage = (float)(voltReading)*3.3*3.2*adcConvert;
      if(pins.batt == pins.pyro4Cont){voltage *= (2.72/3.2);}//on old units we used to pull the battery voltage through one of the pyro channels which had diodes
      writeVolt = true;
      lastVolt = fltTime.timeCurrent;}
    */
  //}//end of liftoff flag

}//end void main loop

void beep(uint8_t pin_value)
{
  if(settings.buzzerEnabled)
  {
    digitalWrite(pins.buzzer, pin_value);
  }
  else
  {
    digitalWrite(pins.buzzer, LOW);
  }
}

void SetupPins()
{
  pinMode(pins.buzzer, OUTPUT);
  digitalWrite(pins.buzzer, LOW);

  pinMode(pins.testGnd, OUTPUT);
  
  // To check later if the test mode button is being held
  digitalWrite(pins.testGnd, LOW);
  delay(50);
}

void CheckIfTestModeEnabled()
{
  if(digitalRead(pins.testRead) != LOW || settings.testMode){
    settings.testMode = true; 
    Serial.println(F("------------------------------------------------------------"));
    Serial.println(F("Test mode !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("------------------------------------------------------------"));
  }

  // Beep if in test mode
  if (settings.testMode){
    beep(HIGH);
    delay(1000);
    beep(LOW);
  }
}

void SetupADC()
{
  // Setup the ADC for sampling the battery and ADXL377 if present
  // 12 bit resolution for Teensy 4.0 & 4.1
  analogReadResolution(12);
  adcConvert = 1/4096;
  ADCmidValue = 2048;
}

void SetupSensors()
{
  Serial.println(F("Setup sensors - BEGIN"));
  //beginAccel();
  Serial.println(F("ACC - OK"));
  //beginGyro();
  Serial.println(F("GYRO - OK"));
  //beginMag();
  Serial.println(F("MAG - OK"));
  //beginBaro();
  Serial.println(F("BARO - OK"));
  Serial.println(F("Setup sensors - END"));
}

void PrintFlightProfile()
{
  if(settings.testMode){
    Serial.print(F("Flight Profile: "));  Serial.print(settings.fltProfile);

    if(settings.fltProfile == SINGLE_STAGE)
    {
      Serial.println(F(" - SINGLE_STAGE"));
    }
    else 
    {
      Serial.println(F(" - Unknown flight profile"));
    }
  }
}

void PrintBuzzerEnabledInfo()
{
  Serial.println(settings.buzzerEnabled ? F("Buzzer - ENABLED") : F("Buzzer - DISABLED"));
}

void ReadBatteryVoltage()
{
  voltReading = analogRead(pins.batt);
  voltage = (float)(voltReading)*3.3*3.2*adcConvert;
}

void PrintBatteryVoltage()
{
  ReadBatteryVoltage();

  Serial.print(F("Battery voltage: "));
  Serial.println(voltage);
}

void SetupServos()
{
  // Setup servos if enabled
  if(settings.stableRotn || settings.stableVert){

    if(settings.testMode){Serial.println(F("Active stabilizatio - ENABLED"));}

    //attach the servos
    canardYaw1.attach(pins.servo1);
    canardYaw2.attach(pins.servo2);
    canardPitch3.attach(pins.servo3);
    canardPitch4.attach(pins.servo4);

    //read the trim settings from EEPROM
    servo1trim = 90;
    servo2trim = 90;
    servo3trim = 90;
    servo4trim = 90;
    if(settings.testMode){
        Serial.print(F("Servo1 Trim: "));Serial.println(servo1trim);
        Serial.print(F("Servo2 Trim: "));Serial.println(servo2trim);
        Serial.print(F("Servo3 Trim: "));Serial.println(servo3trim);
        Serial.print(F("Servo4 Trim: "));Serial.println(servo4trim);
    }
     
    //Test canards
    canardYaw1.write(120-servo1trim);
    canardYaw2.write(120-servo2trim);
    canardPitch3.write(120-servo3trim);
    canardPitch4.write(120-servo4trim);
    delay(1000);
    canardYaw1.write(60-servo1trim);
    canardYaw2.write(60-servo2trim);
    canardPitch3.write(60-servo3trim);
    canardPitch4.write(60-servo4trim);
    delay(1000);
    canardYaw1.write(90-servo1trim);
    canardYaw2.write(90-servo2trim);
    canardPitch3.write(90-servo3trim);
    canardPitch4.write(90-servo4trim);}

  //otherwise disable the servos and ensure that stray voltages do not cause any attached servos to move
  else{
    pinMode(pins.servo1, OUTPUT);
    pinMode(pins.servo2, OUTPUT);
    pinMode(pins.servo3, OUTPUT);
    pinMode(pins.servo4, OUTPUT);
    pinMode(pins.servo5, OUTPUT);
    pinMode(pins.servo6, OUTPUT);
    pinMode(pins.servo7, OUTPUT);
    pinMode(pins.servo8, OUTPUT);

    digitalWrite(pins.servo1, LOW);
    digitalWrite(pins.servo2, LOW);
    digitalWrite(pins.servo3, LOW);
    digitalWrite(pins.servo4, LOW);
    digitalWrite(pins.servo5, LOW);
    digitalWrite(pins.servo6, LOW);
    digitalWrite(pins.servo7, LOW);
    digitalWrite(pins.servo8, LOW);
  }
}

void PrintSensorCalibrationValuesFromEEPROM()
{
  // Output the read values
  if(settings.testMode){
    /*
     Serial.println(F("Calibrataion values read from EEPROM:"));
     Serial.print(F("accel.biasX: "));Serial.println(accel.biasX);
     Serial.print(F("accel.biasY: "));Serial.println(accel.biasY);
     Serial.print(F("accel.biasZ: "));Serial.println(accel.biasZ);
     Serial.print(F("gyro.biasX: "));Serial.println(gyro.biasX);
     Serial.print(F("gyro.biasY: "));Serial.println(gyro.biasY);
     Serial.print(F("gyro.biasZ: "));Serial.println(gyro.biasZ);
     Serial.print(F("mag.biasX: "));Serial.println(mag.biasX);
     Serial.print(F("mag.biasY: "));Serial.println(mag.biasY);
     Serial.print(F("mag.biasZ: "));Serial.println(mag.biasZ);
     Serial.print(F("baro.tempOffset: "));Serial.println(baro.tempOffset, 1);
     Serial.print(F("baro.pressOffset: "));Serial.println(baro.pressOffset, 2);
     */
  }
}

void ReadSensorsForSomeTime()
{
if(settings.testMode){Serial.println(F("Sampling Sensors"));}
  //Sample sensors for 3 seconds to determine the offsets and initial values
  //all gyros must be calibrated at the pad except the LSM6DSOX

  /*
  while(micros() - calibrationStart < sampTime){
    //accelerometer
    if(micros() - accel.timeLastSamp > accel.timeBtwnSamp){
      getAccel();
      getGyro();
      getMag();
    }
  }//end sample period
  */

  /*
  if(settings.testMode){
    Serial.println(F("Sampling complete"));
    Serial.print(F("Gyro Offsets: "));
    Serial.print(gyro.biasX);Serial.print(F(", "));
    Serial.print(gyro.biasY);Serial.print(F(", "));
    Serial.println(gyro.biasZ);
    Serial.print(F("Accel X0,Y0,Z0: "));
    Serial.print(accel.x0);Serial.print(F(", "));
    Serial.print(accel.y0);Serial.print(F(", "));
    Serial.println(accel.z0);
    Serial.print(F("HighG X0,Y0,Z0: "));
    Serial.print(highG.x0);Serial.print(F(", "));
    Serial.print(highG.y0);Serial.print(F(", "));
    Serial.println(highG.z0);
    Serial.print(F("Mag X0,Y0,Z0: "));
    Serial.print(mag.x0);Serial.print(F(", "));
    Serial.print(mag.y0);Serial.print(F(", "));
    Serial.println(mag.z0);
  }
  */
}

void InitActiveStabilizationSystem()
{
  //Compute the acceleromter based rotation angle
  if (accel.y0 >= 0) {yawY0 = asinf(min(1, (float)accel.y0 / (float)g)) * RAD_TO_DEG;}
  else {yawY0 = asinf(max(-1, (float)accel.y0 / (float)g)) * RAD_TO_DEG;}

  if (accel.x0 >= 0) {pitchX0 = asinf(min(1, (float)accel.x0 / (float)g)) * RAD_TO_DEG;}
  else {pitchX0 = asinf(max(-1, (float)accel.x0 / (float)g)) * RAD_TO_DEG;}

  //update quaternion rotation
  getQuatRotn(pitchX0*1000000/gyro.gainZ, yawY0*1000000/gyro.gainZ, 0, gyro.gainZ);

  //use DCM2D if we are deploying control surfaces
  if(settings.stableRotn || settings.stableVert){getDCM2DRotn(pitchX0*1000000/gyro.gainZ, yawY0*1000000/gyro.gainZ, 0, gyro.gainZ);}
  //Output the initial rotation conditions reltative to the Earth
  if(settings.testMode){
    Serial.println(F("Rotation Computation Complete"));
    Serial.print(F("Yaw: "));Serial.println(yawY0, 2);
    Serial.print(F("Pitch: "));Serial.println(pitchX0, 2);
    Serial.print(F("Off Vertical: "));Serial.println(((float)offVert)*.1, 1);}
}





