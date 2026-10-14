# LotusRosBot v2026.10.14e (ESP32 + LotusRosDev Shield)

- build: git f253dd5 · pio env lotusrosbot · 2026-10-14
- เขียนที่ 0x10000 (เฉพาะโปรแกรม — bootloader/partition เดิมบนบอร์ด) · esptool --chip esp32 --baud 115200
- sha256: 2da3cff2696f05059219f761f5da68f582fe9fce08444ec0f6a3c818e10a3341 (502128 bytes)
- สถานะ: **ทดสอบบนโต๊ะแล้ว** (T1 ระบบ · T2 มุมไหล 0.00°/2 นาที · PID 4/2.0/0.1 autotune ล้อลอย)
- **ยังไม่ทดสอบวิ่งบนพื้น** (T3 หมุน 360° · T4/T5 สี่เหลี่ยม · ภารกิจ) — ใช้แข่งหลังทดสอบพื้นเท่านั้น
- ย้อนเวอร์ชัน: แฟลชไฟล์รุ่นก่อนใน ~/fw_backup หรือ FW_OLD=1 bash tools/flash.sh upload (เฟิร์มแวร์เดิมแบบ linorobot2)
