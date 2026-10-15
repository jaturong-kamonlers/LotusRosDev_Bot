# linorobot2 v2026.10.12d (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-12 · git 225f32b · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.12d_ssh.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: a6e5dcb82d3f7d18a6b2eab7624a857fe2f534b53f3c2193c4dab760901f8ebf (457136 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- CALIB เมนู 9 SSH login — แสดงคำสั่ง ssh user@ip + user/รหัสของ Jetson

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
