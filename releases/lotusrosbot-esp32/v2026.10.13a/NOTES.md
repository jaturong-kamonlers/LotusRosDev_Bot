# linorobot2 v2026.10.13a (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-13 · git 147d440 · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.13a_20261013_100749.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: 75dc3d4bfa1f92d7bedae8594447caac31ec35e8aac4629fa3741eeab7b46fa6 (457232 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- OLED แสดงข้อมูลเครือข่ายเฉพาะเมื่อได้รับทาง USB จริง
- netinfo ส่งเป็นระยะโดยไม่แตะขา DTR/RTS (แก้ ESP32 รีเซ็ต/บี๊บเอง)

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
