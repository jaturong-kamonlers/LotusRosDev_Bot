# linorobot2 v2026.10.12b (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-12 · git d616745 · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.12b_nowifi.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: aab78cfc8009f1d73524ca21c892a0390b375b28ee69a3892156281906ded716 (443408 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- build แบบ USB serial อย่างเดียว ไม่ลิงก์ WiFi — flash 900 → 443 KB, RAM 71 → 49 KB

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
