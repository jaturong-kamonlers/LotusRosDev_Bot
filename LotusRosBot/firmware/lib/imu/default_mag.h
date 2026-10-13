// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFAULT_MAG
#define DEFAULT_MAG
// [LotusRosBot] ตัดไดรเวอร์รุ่นที่หุ่นไม่ได้ใช้ออก (2026-10-13) — ต้นฉบับ: linorobot2_hardware (Apache-2.0) · MPU6050 ไม่มีเข็มทิศ → ใช้ FakeMAG

//include MAG base interface
#include "mag_interface.h"

//include sensor API headers

class FakeMAG: public MAGInterface
{
    private:
        geometry_msgs__msg__Vector3 mag_;

    public:
        FakeMAG()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readMagnetometer() override
        {
            return mag_;
        }
};
#endif
