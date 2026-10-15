# LotusRosBot v2026.10.14b (ESP32 + LotusRosDev Shield)

- สร้าง: 2026-10-13 · git 6946f64 · ไฟล์ต้นทาง ~/fw_backup/firmware_LotusRosBot_v2026.10.14b_20261013_165120.bin
- เขียนที่ 0x10000 (เฉพาะโปรแกรม) · esptool --chip esp32 --baud 115200
- sha256: d52665eba7183d4e3955f4404d1bc3837120afc0ee636417e50187fc9db56813 (464256 bytes)
- ชนิด: LotusRosBot · node /lotusrosbot_base

## เปลี่ยนจากรุ่นก่อน

- เปลี่ยนชื่อเป็น LotusRosBot — จัดโค้ดเป็นโมดูล ros_link / base / heading / sensors / actuators / display / netinfo / calib
- ชื่อ node ใหม่ /lotusrosbot_base (topic / type / QoS เท่าเดิม ตรวจด้วย snapshot แล้ว)
- ตัดไลบรารีที่ไม่ใช้ออก (464 KB)

## ข้อควรรู้

- PID ล้อตามค่าคอมไพล์เดิม 0.35/0.40/0.10 (ยังไม่จูน)
