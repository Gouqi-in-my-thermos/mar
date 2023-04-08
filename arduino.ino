#include <MsTimer2.h>  //定时中断
/////////编码器引脚////////
#define ENCODER_L 3  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 12
#define ENCODER_R 2
#define DIRECTION_R 11

//定义引脚名称
#define PWMA 5  //10为模拟引脚，用于PWM控制
#define AIN1 7
#define AIN2 6
#define PWMB 10  //11为模拟引脚，用于PWM控制
#define BIN1 8
#define BIN2 9
#define STBY 4  //2、4、8、12、7为数字引脚，用于开关量控制

int PwmA, PwmB;

float L = 0.168;
float r = 0.0325;
float rad_per_sec = 30.67;
float ppr = 15000;
float pi = 3.1415926;

volatile long Velocity_L, Velocity_R;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;  //左右轮速度
float vx = 0, wz = 0;

float linear_speed, angular_speed;

void control() {
  sei();  //全局中断开启
  Velocity_Left = Velocity_L;
  Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_Right = Velocity_R;
  Velocity_R = 0;  //读取右轮编码器数据，并清零
}
void setup() {
  Serial.begin(115200);

  //TB6612电机驱动模块控制信号初始化
  pinMode(AIN1, OUTPUT);  //控制电机A的方向，(AIN1, AIN2)=(1, 0)为正转，(AIN1, AIN2)=(0, 1)为反转
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);  //控制电机B的方向，(BIN1, BIN2)=(0, 1)为正转，(BIN1, BIN2)=(1, 0)为反转
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);  //A电机PWM
  pinMode(PWMB, OUTPUT);  //B电机PWM
  pinMode(STBY, OUTPUT);  //TB6612FNG使能, 置0则所有电机停止, 置1才允许控制电机

  //初始化TB6612电机驱动模块
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);


  pinMode(ENCODER_L, INPUT);    //编码器引脚
  pinMode(DIRECTION_L, INPUT);  //编码器引脚
  pinMode(ENCODER_R, INPUT);    //编码器引脚
  pinMode(DIRECTION_R, INPUT);  //编码器引脚

  delay(200);                                                         //延时等待初始化完成
  attachInterrupt(digitalPinToInterrupt(2), READ_ENCODER_R, RISING);  //开启外部中断 编码器接口1
  attachInterrupt(digitalPinToInterrupt(3), READ_ENCODER_L, RISING);  //开启外部中断 编码器接口2
  MsTimer2::set(10, control);                                         //使用Timer2设置10ms定时中断
  MsTimer2::start();                                                  //中断使能
}

void SetPWM(int motor, int pwm) {
  if (motor == 1 && pwm >= 0)  //motor=1代表控制电机A，pwm>=0则(AIN1, AIN2)=(1, 0)为正转
  {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  } else if (motor == 1 && pwm < 0)  //motor=1代表控制电机A，pwm<0则(AIN1, AIN2)=(0, 1)为反转
  {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  } else if (motor == 2 && pwm >= 0)  //motor=2代表控制电机B，pwm>=0则(BIN1, BIN2)=(0, 1)为正转
  {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, pwm);
  } else if (motor == 2 && pwm < 0)  //motor=2代表控制电机B，pwm<0则(BIN1, BIN2)=(1, 0)为反转
  {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, -pwm);
  }
}

void set_velocity(float vx, float wz) {
  float vl = vx - wz * L / 2;
  float vr = vx + wz * L / 2;
  float wl = vl / r;
  float wr = vr / r;
  int pwm_l = (wl / rad_per_sec) * 255;
  int pwm_r = (wr / rad_per_sec) * 255;
  SetPWM(1, pwm_l);
  SetPWM(2, pwm_r);
}

void get_velocity(int right_count, int left_count, float &vx, float &wz) {
  float wr = (2.0 * pi * (right_count / ppr)) * 100.0;
  float wl = (2.0 * pi * (left_count / ppr)) * 100.0;
  vx = (wr * r + wl * r) / 2.0;
  wz = (wr * r - wl * r) / L;
}


void loop() {
  if (Serial.available() >= 16) {
    float start_zero;
    float c_vx, c_wz;
    float check_sum;
    Serial.readBytes((unsigned char *)&start_zero, sizeof(float));
    Serial.readBytes((unsigned char *)&c_vx, sizeof(float));       // 读取第一个浮点数
    Serial.readBytes((unsigned char *)&c_wz, sizeof(float));       // 读取第二个浮点数
    Serial.readBytes((unsigned char *)&check_sum, sizeof(float));  // 读取第二个浮点数
    if (fabs(check_sum - c_vx - c_wz) < 0.01) {
      set_velocity(c_vx, c_wz);
    }
  }

 get_velocity(Velocity_Right, Velocity_Left, vx, wz);
float zero = 0;
  float check_sum;
  check_sum = vx + wz;
  Serial.write((uint8_t *)&zero, sizeof(float));
  Serial.write((uint8_t *)&vx, sizeof(float));
  Serial.write((uint8_t *)&wz, sizeof(float));

  Serial.write((uint8_t *)&check_sum, sizeof(float));

}

void READ_ENCODER_L() {
  if (digitalRead(DIRECTION_L) == LOW) Velocity_L++;
  else Velocity_L--;
}
void READ_ENCODER_R() {
  if (digitalRead(DIRECTION_R) == LOW) Velocity_R--;
  else Velocity_R++;
}