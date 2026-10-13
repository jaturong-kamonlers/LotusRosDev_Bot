// base.cpp — ดูคำอธิบายใน base.h
#include "base.h"
#include "config.h"
#include "actuators.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include <Preferences.h>

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

// ── ฮาร์ดแวร์ ────────────────────────────────────────────────
static Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
static Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

static Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
static Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

static PID motor1_pid(PWM_MIN, PWM_MAX, K_P_L, K_I_L, K_D_L);
static PID motor2_pid(PWM_MIN, PWM_MAX, K_P_R, K_I_R, K_D_R);

static float pid_gain[2][3] = { {K_P_L, K_I_L, K_D_L}, {K_P_R, K_I_R, K_D_R} };
static bool  pid_from_nvs = false;

static Kinematics kinematics(
    Kinematics::ROBOT_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO_L, MAX_RPM_RATIO_R,
    MOTOR_OPERATING_VOLTAGE, MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER, LR_WHEELS_DISTANCE
);

static Odometry odometry;

static Motor   *motors[2]   = {&motor1_controller, &motor2_controller};
static PID     *pids[2]     = {&motor1_pid,        &motor2_pid};
static Encoder *encoders[2] = {&motor1_encoder,    &motor2_encoder};

// ── สถานะ ────────────────────────────────────────────────────
geometry_msgs__msg__Twist twist_msg;
unsigned long prev_cmd_time = 0;
static unsigned long prev_odom_update = 0;
static float last_rpm1 = 0.0f, last_rpm2 = 0.0f;

void baseBeginEncoders() {
    // ต้องเรียกหลัง micro-ROS สร้าง entity เสร็จ ไม่งั้น micro-ROS init จะทับ PCNT → encoder หยุดนับ (2026-07-04)
    motor1_encoder.begin();
    motor2_encoder.begin();
}

void fullStop() {
    twist_msg.linear.x  = 0.0;
    twist_msg.linear.y  = 0.0;
    twist_msg.angular.z = 0.0;
    motor1_controller.brake();
    motor2_controller.brake();
}

void moveBase() {
    if ((millis() - prev_cmd_time) >= 200) {           // ไม่มี /cmd_vel ใหม่เกิน 0.2 วินาที → หยุด
        twist_msg.linear.x  = 0.0;
        twist_msg.linear.y  = 0.0;
        twist_msg.angular.z = 0.0;
        ledDefaultOn();
    }

    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z
    );

    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    last_rpm1 = current_rpm1; last_rpm2 = current_rpm2;

    if (twist_msg.linear.x == 0.0 && twist_msg.angular.z == 0.0) {
        fullStop();
        motor1_pid.reset();
        motor2_pid.reset();
    } else {
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, current_rpm2, 0.0f, 0.0f);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0f;
    prev_odom_update = now;
    odometry.update(vel_dt,
        current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
}

nav_msgs__msg__Odometry baseGetOdom() { return odometry.getData(); }

void baseLastRpm(float &rpm1, float &rpm2) { rpm1 = last_rpm1; rpm2 = last_rpm2; }

// ── โหมดคาลิเบรต ─────────────────────────────────────────────
void    baseMotorSpin(int m, int pwm)        { motors[m]->spin(pwm); }
void    baseMotorBrake(int m)                { motors[m]->brake(); }
void    baseMotorCoast(int m) {
    // brake() สั่งแค่ PWM = 0 แล้วปล่อยขาทิศค้าง → ไดรเวอร์แบบ PWM + 2 ขาทิศ (เช่น TB6612) = short brake
    //   ขั้วมอเตอร์ถูกต่อเข้าหากัน หมุนล้อด้วยมือจะฝืดมาก · coast = ขาทิศ LOW ทั้งคู่ → ล้อหมุนฟรี
    motors[m]->brake();
    if (m == 0) { digitalWrite(MOTOR1_IN_A, LOW); digitalWrite(MOTOR1_IN_B, LOW); }
    else        { digitalWrite(MOTOR2_IN_A, LOW); digitalWrite(MOTOR2_IN_B, LOW); }
}
int32_t baseEncRead(int m)                   { return encoders[m]->read(); }
void    baseEncWrite(int m, int32_t ticks)   { encoders[m]->write(ticks); }
float   baseMaxRPM()                         { return kinematics.getMaxRPM(); }
float   baseLinearAt(float rpm) {
    Kinematics::velocities v = kinematics.getVelocities(rpm, rpm, 0, 0);
    return v.linear_x;
}

// ── PID: อ่าน/ตั้ง/บันทึก ─────────────────────────────────────
void basePidGet(int m, float &kp, float &ki, float &kd) {
    kp = pid_gain[m][0]; ki = pid_gain[m][1]; kd = pid_gain[m][2];
}

void basePidSet(int m, float kp, float ki, float kd) {
    pid_gain[m][0] = kp; pid_gain[m][1] = ki; pid_gain[m][2] = kd;
    pids[m]->updateConstants(kp, ki, kd);
    pids[m]->reset();
}

bool basePidFromSaved() { return pid_from_nvs; }

bool basePidLoadSaved() {
    Preferences p;
    p.begin("lrbpid", true);
    bool has = p.getBool("set", false);
    if (has) {
        basePidSet(0, p.getFloat("kp1", K_P_L), p.getFloat("ki1", K_I_L), p.getFloat("kd1", K_D_L));
        basePidSet(1, p.getFloat("kp2", K_P_R), p.getFloat("ki2", K_I_R), p.getFloat("kd2", K_D_R));
    }
    p.end();
    pid_from_nvs = has;
    return has;
}

void basePidSave() {
    Preferences p;
    p.begin("lrbpid", false);
    p.putFloat("kp1", pid_gain[0][0]); p.putFloat("ki1", pid_gain[0][1]); p.putFloat("kd1", pid_gain[0][2]);
    p.putFloat("kp2", pid_gain[1][0]); p.putFloat("ki2", pid_gain[1][1]); p.putFloat("kd2", pid_gain[1][2]);
    p.putBool("set", true);
    p.end();
    pid_from_nvs = true;
}

void basePidClearSaved() {
    Preferences p;
    p.begin("lrbpid", false);
    p.clear();
    p.end();
    basePidSet(0, K_P_L, K_I_L, K_D_L);
    basePidSet(1, K_P_R, K_I_R, K_D_R);
    pid_from_nvs = false;
}

void basePidResetState() { motor1_pid.reset(); motor2_pid.reset(); }

void baseClosedLoop(float tgt_rpm1, float tgt_rpm2, float &rpm1, float &rpm2, int &pwm1, int &pwm2) {
    rpm1 = motor1_encoder.getRPM();
    rpm2 = motor2_encoder.getRPM();
    last_rpm1 = rpm1; last_rpm2 = rpm2;
    if (tgt_rpm1 == 0.0f && tgt_rpm2 == 0.0f) {       // เหมือน moveBase: เป้าเป็นศูนย์ทั้งคู่ = เบรก
        motor1_controller.brake(); motor2_controller.brake();
        motor1_pid.reset(); motor2_pid.reset();
        pwm1 = pwm2 = 0;
        return;
    }
    pwm1 = (int)motor1_pid.compute(tgt_rpm1, rpm1);
    pwm2 = (int)motor2_pid.compute(tgt_rpm2, rpm2);
    motor1_controller.spin(pwm1);
    motor2_controller.spin(pwm2);
}
