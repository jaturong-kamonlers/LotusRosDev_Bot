# LotusRosBot v2026.10.14c (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-13 · git 70a052f · ไฟล์ต้นทาง ~/fw_backup/firmware_lotusrosbot_v2026.10.14c_20261013_184904.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: 897d3a7e79ba5a18992e51ab484499c411a650a562d151cd70b2634aa9cda2f6 (501952 bytes)
- ชนิด: LotusRosBot · node /lotusrosbot_base

## เปลี่ยนจากรุ่นก่อน

- โหมดคาลิเบรตสั่งจากเว็บได้ (คำสั่ง @ ทาง USB) — ใช้คู่กับหน้า lrb_calib_web :8091
- เก็บค่า PID ไว้ใน NVS ของบอร์ด (ปรับจากเว็บแล้วจำค่าได้)
- PID มี anti-windup
- OLED เมนู 10 ทดสอบ PID แบบ step

## ข้อควรรู้

- PID เริ่มต้นตามค่าคอมไพล์ 0.35/0.40/0.10 ถ้ายังไม่เคยบันทึกค่าลง NVS
