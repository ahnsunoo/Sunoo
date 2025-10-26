#include <MsTimer2.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>


float pitch = 0.0; // imu/data 토픽의 orientation.y 값을 저장할 변수

ros::NodeHandle  nh;
geometry_msgs::Twist cmd_vel;   //모터 명령 수신을 위한 변수(수신)

/*실질적으로 우리가 변화시키는 변수*/
/*imu, gps, object detection을 통해 만든 알고리즘의 목적은 적당한 속도와 회전 각도이므로 두 변수를 pub한다*/
int velocity = 0;
int steer_angle = 0;


void imu_callback(const std_msgs::Float32& pitch_msg)
{
  // Float32 메시지의 data 필드에서 pitch 값을 바로 가져옴
  pitch = pitch_msg.data;
}


void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
  velocity = (int)msg.linear.x;       // 속도제어. *.cpp에서 만든 알고리즘을 통해 publish된 velocity값이다.
  steer_angle = (int)msg.angular.z;   // steering motor 각도 제어. *.cpp에서 만든 알고리즘을 통해 publish된 steer_angle값이다.




  steer_angle=steer_angle*(-1);







  if (velocity >= 80) velocity  = 80; // 목표속도 제한
  if (velocity <= -80) velocity = -80; // 목표속도 제한 -> 차량이 낼 수 있는 속도의 최대값은 36이 최대이기 때문에 여유를 두고 30으로 제한함
} //알고리즘을 통해 나온 명령 값이 아두이노로 전달된다.

ros::Subscriber<geometry_msgs::Twist> cmd_sub("teleop_cmd_vel", cmd_vel_callback); //(teleop_cmd_vel토픽)알고리즘을 통해 정해진 velocity와 steer_angle을 cmd_vel_callback함수를 통해 받는 서브스크라이버
ros::Subscriber<std_msgs::Float32> imu_sub("imu/pitch", imu_callback); // '/imu/pitch' 토픽을 구독하도록 변경

// Front Motor Drive
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7

void motor_control(int motor_pwm_output)
{
  if (motor_pwm_output > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor_pwm_output);

    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor_pwm_output);
  }
  else if (motor_pwm_output < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor_pwm_output);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor_pwm_output);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);

    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}

#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23
signed long encoder1count = 0;
signed long encoder2count = 0;

void initEncoders() {

  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);

  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, HIGH);

  SPI.begin();

  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD, LOW);       // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD, HIGH);      // Terminate SPI conversation

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD, LOW);       // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD, HIGH);      // Terminate SPI conversation
}

long readEncoder(int encoder_no)
{
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;

  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);   // Begin SPI conversation
  // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
  // Calculate encoder count
  count_value = ((long)count_1 << 24) + ((long)count_2 << 16) + ((long)count_3 << 8 ) + (long)count_4;

  return count_value;
}

void clearEncoderCount(int encoder_no) {
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW); // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW); // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH); // Terminate SPI conversation
}

///////// Motor PID //////////
#define velo_Kp  3.5   //10     // 10    // 5
#define velo_Ki  0  //0.25   // 0.1   // 0.5
#define velo_Kd  3.5   //15     // 15    // 0

long dt = 0;
long Curr_val, old_val, d_val;
int velo_val = 0;  // min = 3, max = 8

long st, end;
float Error, d_Error, old_Error, Sum_Error;
int PWM = 0;

unsigned long n_close = 0;

void velo_control_callback()
{
  Curr_val = -1 * readEncoder(2);
  //velocity_count = (float)(ENC1 - old_ENC1) * 88.5 / (300.0 * 0.02);
  d_val = Curr_val - old_val;
  old_val = Curr_val;
}

unsigned int n_count = 1;
int dr_val = 1;

//  ENC Counter during sampling time 10ms = 100hz

void Velo_PID_Control()
 {
  end = millis();
  if ((end - st) >= 10) {

    velo_control_callback();

    Error = velo_val - d_val;
    //Serial.println((String)"error : " + Error);

    //지금 속도와 원하는 속도의 차이
    d_Error = Error - old_Error;
    //이전의 오차와 지금 오차의 차이
    Sum_Error += Error;

    dt = end - st;

    PWM = velo_Kp * Error + velo_Ki * Sum_Error + velo_Kd * d_Error;
    PWM=(-1)*PWM;

    //if(pitch<-100)
    //{
    //    PWM=PWM-80;
    //}
    //else if(pitch>100)
    //{
      //  PWM=PWM+80;
    //}
    // PWM에 제한을 걸기
    //if(PWM>180)
    //{
    //    PWM=180;
    //}
    //else if(PWM<-180)
    //{
       // PWM=-180;
    //}
    //Serial.println();

    motor_control(PWM);

    st = millis();
    old_Error = Error;
  }
}


///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////
#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define NEURAL_ANGLE 0
#define LEFT_STEER_ANGLE -20
#define RIGHT_STEER_ANGLE 20
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10


double error, error_old;
double error_s, error_d, error_i;
int pwm_output;

int sensorValue = 0;        // value read from the pot
int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)
int Steering_Angle = NEURAL_ANGLE;

void steer_motor_control(int motor_pwm)
{
  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void PID_Control()
{
  error = Steering_Angle - Steer_Angle_Measure;

  error=error*(-1);
  error_i += error;
  error_d = error - error_old;
  error_i = (error_i >=  100) ?  100 : error_i;
  error_i = (error_i <= -100) ? -100 : error_i;
  //Serial.println(error);
///  Serial.println((String)"error : " + error);
  // 실제조향각 에러 출력




    float Kp =5.4; //5.3;
    float Ki = 0.0;
    float Kd = 6; //0.8;


    //float Kp = 4.5;
    //float Ki = 0.32;
    //float Kd = 0.8;

    pwm_output = Kp * error + Kd * error_d + Ki * error_s;
    pwm_output = (pwm_output >=  200) ?  200 : pwm_output;
    pwm_output = (pwm_output <= -200) ? -200 : pwm_output;

    pwm_output=pwm_output*(-1);

//   / Serial.println((String)"pwm_output : " + pwm_output);

    if(fabs(error)<2)
    {
      pwm_output=0;
      steer_motor_control(pwm_output);
    }
    else
    {
      if(fabs(pwm_output)<55)
      {
        if(pwm_output>0)
        {
          steer_motor_control(55);
        }
        else
        {
          steer_motor_control(-55);
        }
      }
      else
      {
        steer_motor_control(pwm_output);
      }
    }

  error_old = error;




}

void steering_control()
{
  if (Steering_Angle >= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;
  if (Steering_Angle <= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;

  PID_Control(); //오차 제어
  /*steer 각도를 조절하는 함수이다. PID_Control함수에서 사용되는 steer_motor_control의 인자 motor_pwm은*/
}

void control_callback()
{
  static boolean output = HIGH;

  digitalWrite(13, output);
  output = !output;


  // read the analog in value:
  sensorValue = analogRead(Steering_Sensor);
//  /Serial.println("sensor value");
//  /Serial.println(sensorValue);

  //조향각 출
  // map it to the range of the analog out:
  //Steer_Angle_Measure = map(sensorValue, 336, 858, LEFT_STEER_ANGLE, RIGHT_STEER_ANGLE);
  Steer_Angle_Measure = map(sensorValue, 946, 336, RIGHT_STEER_ANGLE, LEFT_STEER_ANGLE);
// / Serial.println(Steer_Angle_Measure);
  Steering_Angle = NEURAL_ANGLE + steer_angle; //cmd_sub를 통해 받는 steer_angle값을 사용한다


}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  // 115200
  pinMode(13, OUTPUT);
  // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR2_ENB, OUTPUT);


  initEncoders();          // initialize encoder
  clearEncoderCount(1);
  clearEncoderCount(2);

  //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM

  error = error_i = error_d = error_old = 0.0;
  Error = Sum_Error = d_Error = old_val = 0;

  pwm_output = 0;

  nh.initNode();

  nh.subscribe(cmd_sub); //velocity와 steer_angle값을 받아오는 서브스크라이버. 계속해서 velocity와 steer_angle값을 받아옴.
  nh.subscribe(imu_sub); // 새로 추가한 IMU 구독자 등록

  MsTimer2::set(100, control_callback); // 500ms period
  MsTimer2::start();
  st = millis();

}

void loop() {
  // put your main code here, to run repeatedly:

  velo_val = velocity;
  Velo_PID_Control();
  steering_control(); //회전 제어 함수
  //encoder1count = readEncoder(1);
  ///encoder2count = readEncoder(2);


  //Serial.println(sensorValue);
 // Serial.println(Steer_Angle_Measure);


  //Serial.print("Encoder 1 : "); Serial.println(encoder1count);
  //Serial.print("Encoder 2 : "); Serial.println(encoder2count);



  //cmd_vel 수신을 다시 pub하여 확인 하기 위한 루틴
//  cmd_vel.linear.x = velocity;
//  cmd_vel.angular.z = steer_angle;
  //cmd_pub.publish(&cmd_vel);
  //delay(10); //딜레이가 실제로 자동차에 반영되는지 확인 필요함. 만약 딜레이가 자동차 값 전달에 영향을 준다면 값 업데이트와 속도 및 회전 각도 유지에 최적인 시간을 찾아야 함.
// / Serial.print("steer_angle_measure : ");
//  /Serial.println(Steer_Angle_Measure);
  nh.spinOnce();
}
