# linorobot2 v2026.10.12c (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-12 · git b813902 · ไฟล์ต้นทาง ~/fw_backup/firmware_v2026.10.12c_netinfo.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: 7ee5c7138927eddc3826a667ce2e13834b60075fa80c1d9f0bb60f1c3d59efa8 (455776 bytes)
- ชนิด: linorobot2 (แบบเดิม) · node /linorobot_base_node

## เปลี่ยนจากรุ่นก่อน

- แสดง SSID / รหัส / IP ของ Jetson บน OLED (รับจาก lotus_netinfo ทาง USB)
- CALIB เมนู 8 Network info

## ข้อควรรู้

- ชื่อ node เป็น /linorobot_base_node — ใช้กับสคริปต์รุ่นก่อน 13 ต.ค. ได้ · ros_proof.sh รับทั้งสองชื่อ
- ไม่มี yaw จาก ESP32 — ~/odom_tf_relay.py ต้องใช้ RELAY_YAW_SRC=gyro (โหมด fw จะไม่ได้มุม)
- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
