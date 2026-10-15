# linorobot2 v2026.10.12 (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-12 · git 24cac21 · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.12.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: b678ca4bafcc8569000b354987c0d5074c9818bee3f285b74ede496cdace7c13 (907392 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- CALIB เมนู 7 Count ticks — หมุนล้อด้วยมือ 10 รอบ วัด CPR จริง
- แสดงเวอร์ชันเฟิร์มแวร์บน OLED ตอนบูต
- โหมด ROS 2 เหมือน v2026.07.17 ทุกอย่าง (topic / QoS / ชื่อ node / timeout)

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
