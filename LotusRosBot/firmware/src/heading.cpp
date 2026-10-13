// heading.cpp — ดูคำอธิบายใน heading.h
#include <Arduino.h>
#include "heading.h"
#include "config.h"
#include "base.h"
#include <micro_ros_utilities/string_utilities.h>   // imu_interface.h ใช้ micro_ros_string_utilities_set
#include "imu.h"
#include "mag.h"

static IMU imu;
static MAG mag;

// ── สถานะการรวมมุม ───────────────────────────────────────────
static double        gyaw_acc     = 0.0; // มุมรวม (rad) ไม่ตัดรอบ
static double        gz_bias      = 0.0; // bias ของ gyro z (rad/s)
static int           gbias_n      = 0;   // จำนวนค่าที่ใช้หา bias ตอนเริ่ม (ครบ 50 = 1 วินาที)
static unsigned long gyaw_prev_us = 0;
static double        gyaw_pub_acc = 0.0; // มุมรวม ณ ตอนส่งครั้งก่อน (ใช้หาความเร็วหมุนเฉลี่ย)
static unsigned long gyaw_pub_us  = 0;

bool imuInit() { return imu.init(); }
void magInit() { mag.init(); }

void updateGyroYaw()
{
    unsigned long now_us = micros();
    double gz = imu.readGyroscope().z;              // rad/s (ยังไม่หัก bias · ไม่ตัดค่าเล็กทิ้ง การหมุนช้าจะได้ไม่หาย)
    if (gyaw_prev_us == 0) { gyaw_prev_us = now_us; return; }
    double dt = (now_us - gyaw_prev_us) * 1e-6;
    gyaw_prev_us = now_us;
    if (dt <= 0.0 || dt > 0.2) return;              // ข้ามช่วงสะดุด (เช่น เพิ่งต่อ agent ใหม่)

    if (gbias_n < 50) {                             // 1 วินาทีแรกหลังต่อ agent: หา bias (หุ่นต้องจอดนิ่ง)
        gz_bias += (gz - gz_bias) / (++gbias_n);
        return;
    }
    double w = gz - gz_bias;
    float rpm1, rpm2;
    baseLastRpm(rpm1, rpm2);
    bool still = (millis() - prev_cmd_time) > 500 && fabs(rpm1) < 0.5 && fabs(rpm2) < 0.5;
    if (still && fabs(w) < 0.05) {                  // จอดนิ่งจริง (ไม่มีคำสั่ง ล้อไม่หมุน gyro เบา): ไม่รวมมุม
        gz_bias += 0.02 * w;                        //   + ค่อย ๆ ปรับ bias ตามอุณหภูมิ (กันมุมไหลตอนจอด)
        return;
    }
    gyaw_acc += w * dt;
}

void imuBuildMessages(sensor_msgs__msg__Imu *imu_out, sensor_msgs__msg__MagneticField *mag_out)
{
    *imu_out = imu.getData();
    *mag_out = mag.getData();

#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_out->magnetic_field.x -= mag_bias[0];
    mag_out->magnetic_field.y -= mag_bias[1];
    mag_out->magnetic_field.z -= mag_bias[2];
#endif

    double roll, pitch, yaw;
    roll  = atan2(imu_out->linear_acceleration.y, imu_out->linear_acceleration.z);
    pitch = atan2(-imu_out->linear_acceleration.y,
                  sqrt(imu_out->linear_acceleration.y * imu_out->linear_acceleration.y +
                       imu_out->linear_acceleration.z * imu_out->linear_acceleration.z));
    yaw   = atan2(sin(gyaw_acc), cos(gyaw_acc));   // มุมจาก gyro ที่รวมบนบอร์ด (MPU6050 ไม่มีเข็มทิศ)

    double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);
    imu_out->orientation.x = cy * cp * sr - sy * sp * cr;
    imu_out->orientation.y = sy * cp * sr + cy * sp * cr;
    imu_out->orientation.z = sy * cp * cr - cy * sp * sr;
    imu_out->orientation.w = cy * cp * cr + sy * sp * sr;
    imu_out->orientation_covariance[8] = FW_YAW_COV;  // ป้าย: yaw รวมบนบอร์ดแล้ว

    // ความเร็วหมุน = ค่าเฉลี่ยตั้งแต่ส่งครั้งก่อน (แทนค่าที่อ่านครั้งเดียว) · จอดนิ่ง = 0
    unsigned long pub_us = micros();
    if (gyaw_pub_us != 0 && pub_us != gyaw_pub_us)
        imu_out->angular_velocity.z = (gyaw_acc - gyaw_pub_acc) / ((pub_us - gyaw_pub_us) * 1e-6);
    gyaw_pub_acc = gyaw_acc; gyaw_pub_us = pub_us;
}
