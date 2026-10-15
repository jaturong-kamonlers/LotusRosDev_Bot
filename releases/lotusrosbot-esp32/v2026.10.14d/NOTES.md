# LotusRosBot v2026.10.14d (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-13 · git db9bbe3 · ไฟล์ต้นทาง ~/fw_backup/firmware_lotusrosbot_v2026.10.14d_20261013_190944.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: e141de8bd06f1693a53d055c680afa28300a4db526fe42f3bb7b7eb5a558b27a (502128 bytes)
- ชนิด: LotusRosBot · node /lotusrosbot_base

## เปลี่ยนจากรุ่นก่อน

- โหมด coast (ปล่อยล้อฟรี) สำหรับหมุนล้อนับรอบด้วยมือ — เมนู 7 และหน้าเว็บ

## ข้อควรรู้

- PID เริ่มต้นตามค่าคอมไพล์ 0.35/0.40/0.10 ถ้ายังไม่เคยบันทึกค่าลง NVS
