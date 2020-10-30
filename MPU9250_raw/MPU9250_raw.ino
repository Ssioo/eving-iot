#include "Wire.h"
// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9250.h"
#include "SoftwareSerial.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
#define MPU9250_I2C_ADDRESS 0x68
#define SENDING_INTERVAL 1000
#define SENSOR_READ_INTERVAL 50

unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

MPU9250 accelgyro;
I2Cdev I2C_M;
uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float heading;
float tiltheading;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define sample_num_mdate 1000
volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];
static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;
volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;
volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;
float temperature;
float pressure;
float atm;
float altitude;

byte headerAcc[4] = {1,0,0,60};
byte headerGyro[4] = {1,0,0,61};
byte headerTilt[4] = {1,0,0,62};
byte headerMag[4] = {1,0,0,63};
byte headerSignal[4] = {2,0,0,0};
byte headerSignalCalib[4] = {3,0,0,0};
byte footer[4] = {0,0,0,0};
long count = 0;

SoftwareSerial HM10(7,8);

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // initialize serial communication
  Serial.begin(115200);
  HM10.begin(115200);
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

  pinMode(11, INPUT);
  pinMode(10, INPUT);
  
  delay(1000);
  Serial.println(" ");
  Mxyz_init_calibrated ();
}

void loop()
{
  delay(50);
  String dataStr = getAccelGyro_Data();
  String compStr = getCompassDate_calibrated(); // compass data has been calibrated here
  //getHeading(); //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  sendToRemote(count, 10, 0.0);
  sendToRemote(count, 11, 0.0);
  sendToRemote(count, 12, getTiltHeading());
  sendToRemote(count, 13, 0.0);

  if (digitalRead(11) == HIGH) {
    // button clicked
    sendToRemote(count, 100, 0.0);
  }
  if (digitalRead(10) == HIGH) {
    sendToRemote(count, 101, 0.0);
  }
  count += 1;
 
  
  while (HM10.available()) {
    byte dataFromHM = HM10.read();
    Serial.write(dataFromHM);
  }
  while (Serial.available()) {
    byte dataToBT = Serial.read();
    HM10.write(dataToBT);
  }
}

void sendToRemote(int count, int type, float tiltRes) {
  if (type == 10) {
    HM10.write((byte*) &headerAcc, 4);
    HM10.write((byte*) &Axyz[0], 4);
    HM10.write((byte*) &Axyz[1], 4);
    HM10.write((byte*) &Axyz[2], 4);
  }
  else if (type == 11) {
    HM10.write((byte*) &headerGyro, 4);
    HM10.write((byte*) &Gxyz[0], 4);
    HM10.write((byte*) &Gxyz[1], 4);
    HM10.write((byte*) &Gxyz[2], 4);
  }
  else if (type == 12) {
    HM10.write((byte*) &headerTilt, 4);
    HM10.write((byte*) &tiltRes, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
  }
  else if (type == 13) {
    HM10.write((byte*) &headerMag, 4);
    HM10.write((byte*) &Mxyz[0], 4);
    HM10.write((byte*) &Mxyz[1], 4);
    HM10.write((byte*) &Mxyz[2], 4);
  }
  else if (type == 100) {
    HM10.write((byte*) &headerSignal, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
  } else if (type == 101) {
    HM10.write((byte*) &headerSignalCalib, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
  }
  else {
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
    HM10.write((byte*) &footer, 4);
  }
  HM10.write((byte*) &count, 4);
}

void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

float getTiltHeading()
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));
  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  float result = 180 * atan2(yh, xh) / PI;
  if (yh < 0) result += 360;
  return result;
}

void Mxyz_init_calibrated ()
{
  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print(" ");
  Serial.println(F("During calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print(" ");
  Serial.println(" ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");
  get_calibration_Data ();
  Serial.println(" ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print(" ");
  Serial.print(my_centre);
  Serial.print(" ");
  Serial.println(mz_centre);
  Serial.println(" ");
}

void get_calibration_Data ()
{
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();
    if (mx_sample[2] >= mx_sample[1]) mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1]) my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1]) mz_sample[1] = mz_sample[2];
    if (mx_sample[2] <= mx_sample[0]) mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0]) my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0]) mz_sample[0] = mz_sample[2];
  }
  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];
  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];
  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;
}

void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

String getAccelGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
  String resultAcc = "\"acc\":{\"x\":\"" + String(Axyz[0]) + "\",\"y\":\"" + String(Axyz[1]) + "\",\"z\":\"" + String(Axyz[2]) + "\"}";
  String resultGyro = "\"gyro\":{\"x\":\"" + String(Gxyz[0]) + "\",\"y\":\"" + String(Gxyz[1]) + "\",\"z\":\"" + String(Gxyz[2]) + "\"}";
  String result = resultAcc + "," + resultGyro;
  //Serial.println(result);
  return result;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);
  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

String getCompassDate_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
  String result = "\"comp\":{\"x\":\"" + String(Mxyz[0]) + "\",\"y\":\"" + String(Mxyz[1]) + "\",\"z\":\"" + String(Mxyz[2]) + "\"}";
  return result;
}
