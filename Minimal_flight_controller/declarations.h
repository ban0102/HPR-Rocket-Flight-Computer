//Header file for all major declared variables and global variables

const uint8_t SINGLE_STAGE = 1;

//-----------------------------------------
//User Settings
//-----------------------------------------
struct {
  bool testMode = true;
  uint8_t fltProfile = SINGLE_STAGE;  // Single stage
  bool buzzerEnabled = true;
  byte serialDebug = 1;

  //-------safety thresholds----------------
  int altThreshold = 120; //120m = 400ft
  int maxAngle = 45; //degrees

  //-------active stabilization----------------
  bool stableRotn = true;
  bool stableVert = true;
} settings;

//-----------------------------------------
// Rocket states
//-----------------------------------------

const uint8_t ROCKET_IDLE = 0;
const uint8_t ROCKET_ARMED = 1;
const uint8_t ROCKET_IN_FLIGHT = 2;
const uint8_t ROCKET_AFTER_FLIGHT = 3;

class Rocket_State{
  private:
    uint8_t rocketState = ROCKET_IDLE;
    bool isReadyForFlight = false;

  public:
    uint8_t GetRocketState() {return rocketState;}
    void SetRocketState(uint8_t newRocketState) {rocketState = newRocketState;}

    bool IsReadyForFlight() {return isReadyForFlight;}
    void SetReadyForFlight(bool newIsReadyForFlight) {isReadyForFlight = newIsReadyForFlight;}
} rocketState;

//-----------------------------------------
//Sensor variables
//-----------------------------------------
byte rawData[14];
typedef struct{
  byte addr;
  int16_t ADCmax;
  float gainX;
  float gainY;
  float gainZ;
  int16_t rawX;
  int16_t rawY;
  int16_t rawZ;
  int8_t dirX;
  int8_t dirY;
  int8_t dirZ;
  char orientX;
  char orientY;
  char orientZ;
  int16_t *ptrX;
  int16_t *ptrY;
  int16_t *ptrZ;
  int8_t *ptrXsign;
  int8_t *ptrYsign;
  int8_t *ptrZsign;
  int32_t sumX0 = 0L;
  int32_t sumY0 = 0L;
  int32_t sumZ0 = 0L;
  int16_t x0;
  int16_t y0;
  int16_t z0;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t biasX;
  int16_t biasY;
  int16_t biasZ;
  uint32_t timeLastSamp = 0UL;
  uint32_t timeBtwnSamp = 10000UL;
  boolean newSamp = false;
} sensorData;
sensorData accel;
sensorData mag;
sensorData gyro; 

//-----------------------------------------
//GPIO pin mapping
//-----------------------------------------
struct{
  uint8_t buzzer = 23;
  uint8_t testGnd = 2;
  uint8_t testRead = 5;
  uint8_t batt = 40;
  uint8_t servo1 = 3;
  uint8_t servo2 = 4;
  uint8_t servo3 = 7;
  uint8_t servo4 = 6;
  uint8_t servo5 = 39;
  uint8_t servo6 = 37;
  uint8_t servo7 = 36;
  uint8_t servo8 = 35;
} pins;

//-----------------------------------------
//Altitude & Baro Sensor variables
//-----------------------------------------
struct{
  float rawAlt = 0.0F;
  float Alt = 0.0F;
  float baseAlt = 10.0F;
  float maxAlt = 0.0F;
  float smoothAlt = 0.0F;
  float Vel = 0.0F;
  float maxVel = 0.0F;
  float pressure;
  float temperature;
  unsigned long timeLastSamp = 0UL;
  unsigned long timeBtwnSamp = 50000UL;
  bool newSamp = false;
  bool newTemp = false;
  float seaLevelPressure = 1013.25;
  float tempOffset;
  float pressOffset;
} baro;

//-----------------------------------------
//Magnetometer Variables
//-----------------------------------------
int16_t magTrigger = 0;
uint8_t magCalibrateMode = 0;
long magRoll = 0UL;
int magOffVert = 0;
int magPitch = 0;
int magYaw = 0;
//-----------------------------------------
//gyro & rotation variables
//-----------------------------------------
long dx = 0L;
long dy = 0L;
long dz = 0L;
float yawY0;
float pitchX0;
int pitchX;
int yawY;
long rollZ = 0;
const float mlnth = 0.000001;
int offVert = 0;
bool rotationFault = false;
//-----------------------------------------
//Rotation timing variables
//-----------------------------------------
unsigned long lastRotn = 0UL;//time of the last call to update the rotation
unsigned long rotnRate = 10000UL;//100 updates per second
//-----------------------------------------
//velocity calculation variables
//-----------------------------------------
float accelVel = 0.0F;
float accelAlt = 0.0F;
float maxVelocity = 0.0F;
float fusionVel = 0.0F;
float fusionAlt = 0.0F;
float thresholdVel = 44.3F;
uint32_t clearRailTime = 250000UL;

//-----------------------------------------
//Active Stabilization
//-----------------------------------------
char userInput;
int8_t servo1trim = 0;
int8_t servo2trim = 0;
int8_t servo3trim = 0;
int8_t servo4trim = 0;
int8_t servo5trim = 0;
int8_t servo6trim = 0;
int8_t servo7trim = 0;
int8_t servo8trim = 0;
const uint32_t controlInterval = 50000UL;
uint32_t timeLastControl = 0UL;
float Kp = 0.0F;
float Ki = 0.0F;
float Kd = 0.0F;
//-----------------------------------------
//debug
//-----------------------------------------
boolean GPSecho = false;

float adcConvert = 0.000015259;
uint16_t ADCmidValue = 32768;

//-----------------------------------------
//digital accelerometer variables
//-----------------------------------------
int g = 1366;

struct timerList{
  unsigned long liftoff = 0UL;
  unsigned long detectLiftoffTime = 500000UL; //0.5s
  unsigned long apogee = 0UL;
  unsigned long mainDeploy = 0UL;
  unsigned long tmClock = 0UL;
  unsigned long tmClockPrev = 0UL;
  unsigned long timeCurrent = 0UL;
  unsigned long dt = 0UL;
};
struct timerList fltTime;

//-----------------------------------------
//Non-Event Booleans
//-----------------------------------------
bool rotnOK = true;

float voltage = 0.0F;
uint16_t voltReading;

