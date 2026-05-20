#include <stdio.h>
#include "IReight_model.hpp"

extern uint8_t g_new_package_flag;

// ************************ 电机参数配置 ************************
int Left_motor_go=8;       // 左电机前进引脚(IN1)
int Left_motor_back=9;     // 左电机后退引脚(IN2)
int Right_motor_go=10;     // 右电机前进引脚(IN3)
int Right_motor_back=11;   // 右电机后退引脚(IN4)
int base_pwm=100;        // 基础速度（直线行驶速度）
int base_LowPwm=60;
// ******************************************* *******************


// ************************ 打印间隔控制变量 ************************
unsigned long lastPrintTime = 0;    // 记录上一次打印的时间戳
const unsigned long PRINT_INTERVAL = 10; // 打印间隔（单位ms），可自行修改
// 比如：500=0.5秒/次，1000=1秒/次，2000=2秒/次，数值越大间隔越长
// ******************************************************************


// ************************ 误差值计算使用变量 ************************
//传感器坐标
const float SENSOR_X[IR_Num] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
//定义传感器阈值和替换权重值
const unsigned long sensor_threshold=250;
const unsigned long replace_weight=0;
// ****************************************************************


// ************************ PID计算使用变量 ************************
//  PID参数（工程简化后的Kp/Ki/Kd，直接调试）
float Kp =33;   // 比例系数
float Ki = 0;   // 积分系数（已融入T）
float Kd = 10;   // 微分系数（已融入T）
// 偏差历史值（保存最近3次）
float e_k = 0.0;   // 本次偏差
float e_k_1 = 0.0;  // 上一次偏差（k-1）
float e_k_2 = 0.0;  // 上上次偏差（k-2）
// 控制量输出
float delta_u = 0.0; // 本次增量
float u_k = 0.0;     // 本次输出（PWM调节增量）

float u_max = 150.0;  // 输出上限（防止PWM突变）
float u_min = -150.0; // 输出下限
// ***********************************************************************


// ************************ 位置偏差计算函数 ************************
float calculatePositionError() {
  float sumXiVi = 0.0, sumVi = 0.0;
  // 修复：循环条件改为IR_NUM（与数组长度一致）
  for (int i = 0; i < IR_Num; i++) {
    // 核心需求：模拟值<250则权重为10，否则用原值
    float weight;
    if (IR_Data_Anglo[i] < sensor_threshold) {
      weight = replace_weight; // 小于250，权重替换
    } else {
      weight = (float)IR_Data_Anglo[i]; // 大于等于250，用原值（显式转float避免警告）
    }
    sumXiVi += SENSOR_X[i] * weight;
    sumVi += weight;
  }
if (sumVi == 0) return 0.0;
  return sumXiVi / sumVi;
}
// ******************************************************************


// ************************ 增量式PID计算函数 ************************
// 输入：本次偏差err_now
// 输出：本次控制量增量delta_u
float pid_calc(float e_k) {
  // 1. 计算增量（严格对应离散化公式）
  delta_u = Kp * (e_k - e_k_1)   // 比例项
          + Ki * e_k            // 积分项
          + Kd * (e_k - 2*e_k_1 + e_k_2); // 微分项
  // 2. 计算本次输出（绝对控制量）
  u_k = u_k + delta_u;

  u_k=constrain(u_k,u_min,u_max);
  // 4. 更新偏差历史值（为下一次计算做准备）
  e_k_2 = e_k_1; // e_k-1 → e_k-2
  e_k_1 = e_k;  // e_k → e_k-1

  return u_k; // 返回最终控制量
}
// ******************************************************************




// ************************ PID电机控制函数（小偏差时候使用error<1） ************************
void pid_tracking(float u_pwm,float speed) {

  float left_pwm=speed + u_pwm;
  float right_pwm=speed - u_pwm; 

  left_pwm=constrain(left_pwm,0,255);
  right_pwm=constrain(right_pwm,0,255);

  digitalWrite(Right_motor_go,HIGH);  // 右电机前进
  digitalWrite(Right_motor_back,LOW);     
  analogWrite(Right_motor_go,right_pwm);//PWM比例0~255调速，左右轮差异略增减
  analogWrite(Right_motor_back,0);

  digitalWrite(Left_motor_go,LOW);  // 左电机前进
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0);//PWM比例0~255调速，左右轮差异略增减
  analogWrite(Left_motor_back,left_pwm);

  //Serial.print(left_pwm);
  //Serial.print("\t");
  //Serial.print(right_pwm);
  //Serial.print("\t");
}
// ***********************************************************************

void setup()
{
    //初始化电机驱动IO为输出方式
    pinMode(Left_motor_go,OUTPUT); // PIN 8 (PWM)
    pinMode(Left_motor_back,OUTPUT); // PIN 9 (PWM)
    pinMode(Right_motor_go,OUTPUT);// PIN 10 (PWM) 
    pinMode(Right_motor_back,OUTPUT);// PIN 11 (PWM)


    serial_init(); // 串口初始化

    // 设置传感器模块为模拟信号输出模式
    SET_Eight_Mode(0, 1, 1);
    g_Amode_Data = 1; 
    g_Dmode_Data = 0; 

    delay(500);

    // 提示信息
    Serial.println("红外寻迹模块已设置为模拟模式 (115200)");
    Serial.println("等待接收模拟数据包...");
}

void loop()
{
    recv_data(); // 接收原始数据

    if (g_new_package_flag == 1)
    {
        g_new_package_flag = 0;

        if (g_Amode_Data == 1) {
          Deal_Usart_AData();
          float error_now=calculatePositionError();
          float u_now=pid_calc(error_now);
          int workSensor_num=0;
          float base_speed;
            for (int i = 0; i < IR_Num; i++) {
              if(IR_Data_Anglo[i]>1000){
                workSensor_num=workSensor_num+1;
              }
              //Serial.print("x");
              //Serial.print(i + 1);
              //Serial.print(":");
              //Serial.print(IR_Data_Anglo[i]);
              //Serial.print("\t");

            }
          if (workSensor_num>= 3||abs(error_now)>=2) {  // 急弯阈值
              u_now *= 1.5;            // 放大1.5倍
              u_now = constrain(u_now, u_min, u_max);
              base_speed=base_LowPwm;
          }
          else{
            base_speed=base_pwm;
          }
         
            pid_tracking(u_now,base_speed);

            // ************************ 仅新增：间隔打印逻辑 ************************
        unsigned long currentTime = millis(); // 获取当前时间戳（毫秒）
        if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
           // Deal_Usart_AData();
           // float error_now=calculatePositionError();
            //float u_now=pid_calc(error_now);
           // pid_tracking(u_now);

            
            

                //Serial.print("error=");
                Serial.print(error_now,2);
                Serial.print("\t");
                //Serial.print("u=");
                Serial.print(u_now,2);
                Serial.println(); // 换行
                // --- 原有打印逻辑结束 ---

                lastPrintTime = currentTime; // 更新上一次打印时间
        }
            // ***********************************************************************

        } else {
            Deal_Usart_Data();
        }
    }
  
}