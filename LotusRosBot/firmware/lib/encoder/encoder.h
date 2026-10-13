/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef Encoder_h_
#define Encoder_h_

// [LotusRosBot] เหลือเฉพาะส่วน ESP32 (2026-10-13) — ต้นฉบับ: PJRC Encoder library (MIT) ผ่าน linorobot2_hardware
#ifdef ESP32
#include "Arduino.h"
#include <ESP32Encoder.h>
class Encoder
{
private:
	int counts_per_rev_ = -1;
        ESP32Encoder encoder_;
	unsigned long prev_update_time_;
        int64_t prev_encoder_ticks_;
	int pin1_ = -1;
	int pin2_ = -1;
	bool begun_ = false;
public:
	Encoder(int pin1, int pin2, int counts_per_rev, bool invert = false) {
		int temp_pin = pin1;
		if (pin1 < 0 || pin2 < 0) return; // unused encoder
		if(invert)
		{
			pin1 = pin2;
			pin2 = temp_pin;
		}
		counts_per_rev_ = counts_per_rev;
		pin1_ = pin1;
		pin2_ = pin2;
		// ไม่ attach PCNT ที่นี่! ถ้า attach ใน global constructor (รันก่อน setup)
		// แล้ว micro-ROS init ทีหลังจะ clobber PCNT → encoder หยุดนับใน ROS2 mode
		// (odom=0). ต้องเรียก begin() หลัง micro-ROS สร้าง entities เสร็จ
		// หรือใน calib mode ก่อนใช้งาน. ยืนยัน 2026-07-04 ด้วย calib-mode A/B test.
	}
	// attach PCNT — เรียกครั้งเดียวหลัง micro-ROS init (ROS2) หรือก่อนใช้งาน (calib)
	void begin() {
		if (counts_per_rev_ < 0 || begun_) return;
		ESP32Encoder::useInternalWeakPullResistors = UP;
		encoder_.attachSingleEdge(pin1_, pin2_);
		encoder_.clearCount();
		begun_ = true;
	}
	float getRPM() {
	        if (counts_per_rev_ < 0 || !begun_) return 0.0;
		int64_t encoder_ticks = encoder_.getCount();
		//this function calculates the motor's RPM based on encoder ticks and delta time
		unsigned long current_time = micros();
		unsigned long dt = current_time - prev_update_time_;

		//convert the time from milliseconds to minutes
		double dtm = (double)dt / 60000000;
		int64_t delta_ticks = encoder_ticks - prev_encoder_ticks_;

		//calculate wheel's speed (RPM)
		prev_update_time_ = current_time;
		prev_encoder_ticks_ = encoder_ticks;

		// double rpm = (delta_ticks / counts_per_rev_) / dtm;

		// Serial.print("delta_ticks:: ");
        // Serial.println(delta_ticks);
		// Serial.print("counts_per_rev_:: ");
        // Serial.println(counts_per_rev_);
		// Serial.print("delta_time_micros:: ");
        // Serial.println(dt);
		// Serial.print("delta_time_mins:: ");
        // Serial.println(dtm);
		// Serial.print("rpm:: ");
        // Serial.println(rpm);
		return (((double) delta_ticks / counts_per_rev_) / dtm);
	}
	inline int32_t read() {
	        if (counts_per_rev_ < 0 || !begun_) return 0;
		return  encoder_.getCount();
	}
        inline void write(int32_t p) {
	        if (counts_per_rev_ < 0 || !begun_) return;
	        encoder_.setCount(p);
	}
};
#else
#error "LotusRosBot firmware รองรับเฉพาะ ESP32"
#endif

#endif // Encoder_h_
