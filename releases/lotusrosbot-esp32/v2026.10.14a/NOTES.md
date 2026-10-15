# linorobot2 v2026.10.14a (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-13 · git 851100c · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.14a_CURRENT.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: 14654db942f21ad576f6a173a74282bf7d7be2a7cb42e4d937095eaf2f7703ba (457840 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- ESP32 รวม gyro z เป็นมุม yaw 50 Hz ส่งใน /imu/data.orientation (ป้าย cov[8]=0.0004) + ค่าเฉลี่ย yaw rate
- เป็นตัวเดียวกับที่ FW_OLD=1 bash tools/flash.sh สร้าง (md5 cd739301…) — git tag fw-v2026.10.14a-before-lotusrosbot

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
