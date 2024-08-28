#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <string.h>
#include <Servo.h>
#include <SharpIR.h>
#include "Wire.h"

// ---- biblioteca mpu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container

unsigned long timer = 0;

// Define model and input pin:
SharpIR SharpSensor = SharpIR(A0, 1080);

#define debug 10
#define base_port 11411 // usar outros números para ligar mais robôs na mesma porta

//----------------------
#define top_cmd_vel "/bob1/cmd_vel"
#define top_servo   "/bob1/ir_motor"
#define top_sharp   "/bob1/ir_sensor"
#define top_imu     "/bob1/heading"
#define top_renc    "/bob1/raw/right_ticks"
#define top_lenc    "/bob1/raw/left_ticks"
//----------------------

Servo servo;
int pos = 0;
int dir = 0;
//----------------------
// Ponte-H
int IN1 = 3;
int IN2 = 1;
int IN3 = 16;
int IN4 = 13;
int LED = 2;
int SRV = 0;
int EC1 = 12;
int EC2 = 14;
//----------------------

int EC1_count = 0;
int EC2_count = 0;

struct config_t
{
    const char *ssid = "Manifesto";
    const char *password = "proletario837";

    uint16_t serverPort = base_port;
    int serverIP[4] = {192, 168, 82, 255}; // "ip do computador com roscore"
                                          //  int serverIP[4] = {10, 0, 2, 15}; // "ip do computador com roscore"
    int robot = 0;
    char *topic_servo = top_servo;
    char *topic_lenc = top_lenc;
    char *topic_renc = top_renc;
    char *topic_cmd_vel = top_cmd_vel;
    char *topic_sharp = top_sharp;
    char *topic_imu = top_imu;
} configuration;

std_msgs::Float32 sharp_msg, lenc_msg, renc_msg, yaw_msg, imu_msg;
ros::Publisher pub_lenc(configuration.topic_lenc, &lenc_msg);
ros::Publisher pub_renc(configuration.topic_renc, &renc_msg);
ros::Publisher pub_sharp(configuration.topic_sharp, &sharp_msg);
ros::Publisher pub_imu(configuration.topic_imu, &imu_msg);
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel(configuration.topic_cmd_vel, &odometry_cb);
ros::Subscriber<std_msgs::UInt16> sub_servo(configuration.topic_servo, servo_cb);

void odometry_cb(const geometry_msgs::Twist &msg)
{
    float forward, lateral, wheelL, wheelR, wlength = 0.115, wradius = 0.0685 / 2, reduction = 48;
    forward = msg.linear.x;
    lateral = msg.angular.z;

    wheelR = ((forward / wradius) + (lateral * wlength) / (2 * wradius)) * reduction;
    wheelL = ((forward / wradius) - (lateral * wlength) / (2 * wradius)) * reduction;

    sharp_msg.data = wheelR;

    if (wheelR >= 0)
    {
        analogWrite(IN2, wheelR);
        analogWrite(IN1, 0);
    }
    else
    {
        analogWrite(IN2, 0);
        analogWrite(IN1, abs(wheelR));
    }
    if (wheelL >= 0)
    {
        analogWrite(IN3, wheelL);
        analogWrite(IN4, 0);
    }
    else
    {
        analogWrite(IN3, 0);
        analogWrite(IN4, abs(wheelL));
    }
}

void servo_cb(const std_msgs::UInt16 &cmd_msg)
{
    servo.write(cmd_msg.data); // set servo angle, should be from 0-180
}

void setupWiFi()
{ // connect to ROS server as as a client
    if (debug)
    {
        Serial.print("Connecting wifi to ");
        Serial.println(configuration.ssid);
    }
    WiFi.begin(configuration.ssid, configuration.password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        if (debug)
            Serial.print(".");
    }
    if (debug)
    {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
}

ros::NodeHandle nh;
void loopconnect()
{

  if (nh.connected()) {
    Serial.println("Connected");
    // Say hello
 
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(1000);
}


void IRAM_ATTR ISR_EC1()
{
    EC1_count++;
}
void IRAM_ATTR ISR_EC2()
{
    EC2_count++;
}

void setup_imu()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // turn on the DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

float get_yaw()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return 0.0; // Retorna 0 se o DMP não estiver pronto

    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

        // display Euler angles in radians
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t"); // print yaw
        Serial.print(ypr[0]);
        Serial.print("\t");
        Serial.print(ypr[1]); // print pitch
        Serial.print("\t");
        Serial.println(ypr[2]); // print roll

        // return yaw
        return ypr[0];
    }

    // Retorna 0 se não for possível ler um pacote FIFO
    return 0.0;
}
void setup()
{

    configuration.serverPort = configuration.serverPort + configuration.robot;
    IPAddress server(configuration.serverIP[0], configuration.serverIP[1], configuration.serverIP[2], configuration.serverIP[3]); // Set the rosserial socket server IP address
    setupWiFi();

    // configure ros communication
    nh.getHardware()->setConnection(server, configuration.serverPort);
    nh.initNode();
    nh.subscribe(sub_cmd_vel);
    nh.subscribe(sub_servo);
    nh.advertise(pub_lenc);
    nh.advertise(pub_renc);
    nh.advertise(pub_sharp);
    nh.advertise(pub_imu);

    // configure GPIO's
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(LED, OUTPUT);

    // configure servo motor and put it at "position 0"
    servo.attach(SRV);
    servo.write(55);

    // configure interruption pins for encoder
    pinMode(EC1, INPUT);
    attachInterrupt(EC1, ISR_EC1, RISING);
    pinMode(EC2, INPUT);
    attachInterrupt(EC2, ISR_EC2, RISING);

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial)
        ;

    // setup and calibration of the imu sensor
    setup_imu();
}

void loop()
{

    lenc_msg.data = EC1_count;
    pub_lenc.publish(&lenc_msg);

    renc_msg.data = EC2_count;
    pub_renc.publish(&renc_msg);

    sharp_msg.data = SharpSensor.getDistance() * 1.8;
    pub_sharp.publish(&sharp_msg);

    imu_msg.data = get_yaw();
    pub_imu.publish(&imu_msg);

    nh.spinOnce();
}