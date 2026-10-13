// ════════════════════════════════════════════════════════════════
//  heading — IMU (MPU6050) + มุมหมุน (yaw) จาก gyro ที่รวมบนบอร์ดทุก 20 ms
//
//  ทำไมต้องรวมบนบอร์ด: สาย USB 115200 ส่งข้อมูลไป Jetson ได้แค่ ~3 ครั้ง/วินาที
//  ถ้าให้ Jetson รวม gyro เอง (ค่าที่สุ่มอ่านทุก 0.3 วินาที) มุมจะคลาดทุกครั้งที่เลี้ยว
//  ที่นี่รวม 50 ครั้ง/วินาที แล้วส่งมุมไปใน /imu/data.orientation (yaw)
//  ป้าย orientation_covariance[8] = 0.0004 บอก odom_tf_relay.py ว่า yaw นี้รวมบนบอร์ดแล้ว
// ════════════════════════════════════════════════════════════════
#ifndef LRB_HEADING_H
#define LRB_HEADING_H

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#define FW_YAW_COV 0.0004        // ต้องตรงกับ FW_YAW_COV ใน ~/odom_tf_relay.py

bool imuInit();                  // false = ไม่พบ MPU6050
void magInit();                  // MPU6050 ไม่มีเข็มทิศ → ใช้ FakeMAG (ค่า 0)
void updateGyroYaw();            // เรียกทุก 20 ms จาก controlCallback
void imuBuildMessages(sensor_msgs__msg__Imu *imu_out, sensor_msgs__msg__MagneticField *mag_out);

#endif
