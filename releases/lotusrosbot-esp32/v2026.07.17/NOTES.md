# linorobot2 v2026.07.17 (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-07-17 · git cebb5ab · ไฟล์ต้นทาง ~/fw_backup/firmware_build_20260717_cebb5ab.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: cd7a2ba1858d4182eb0ab2ad53c37eab0b7c94a6ec33d6f67bc9f2dd0fa7e13e (906672 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- มอเตอร์ + ESP32 ชุดใหม่ 17 ก.ค. — CPR 958/925
- ROS 2 Galactic
- กันค่า PCNT ของ encoder ถูกเขียนทับ (PCNT clobber)

## ข้อควรรู้

- ไฟล์ใหญ่ ~907 KB เพราะลิงก์ไลบรารี WiFi ไว้ด้วย (แต่ต่อผ่านสาย USB)
- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
