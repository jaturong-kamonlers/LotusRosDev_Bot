#!/bin/bash
# efuse.sh — ตั้ง eFuse ให้แฟลช ESP32 ใช้ 3.3V ถาวร (ไม่สนขา GPIO12)
# เจ้าของพิมพ์ BURN ยืนยันก่อน แล้วสคริปต์ค่อยต่อชิปและเขียนทันที (ไม่ค้าง serial ระหว่างรอพิมพ์ — กันสายหลุด)
P=~/.platformio/penv/bin/python; T=~/.platformio/packages/tool-esptoolpy
LOG=~/efuse_log.txt
if pgrep -f "micro_ros_agent serial" >/dev/null; then echo "❌ agent เปิดอยู่ — ปิดก่อน: bash ~/stop.sh all"; exit 1; fi
echo "== ตรวจชิปและแฟลช =="
$P $T/esptool.py --chip esp32 --port /dev/esp32 --baud 115200 flash_id 2>&1 | grep -E "Chip is|Manufacturer|Device|flash size"
S=$($P $T/espefuse.py --chip esp32 --port /dev/esp32 summary 2>&1)
if echo "$S" | grep -q "set to 3.3V by efuse"; then echo "✅ ตั้งไว้แล้ว ไม่ต้องทำซ้ำ"; exit 0; fi
if ! echo "$S" | grep -q "determined by GPIO12"; then echo "❌ อ่านสถานะ eFuse ไม่ได้ — ลองรันใหม่"; echo "$S" | tail -3; exit 1; fi
echo
echo "จะตั้ง eFuse: ไฟแฟลช = 3.3V ถาวร (ย้อนกลับไม่ได้)"
read -r -p "พิมพ์ BURN (ตัวใหญ่) แล้วกด Enter เพื่อยืนยัน: " ans
[ "$ans" = "BURN" ] || { echo "ยกเลิก (ไม่ได้พิมพ์ BURN)"; exit 1; }
for i in 1 2 3; do
  echo "== เขียน eFuse ครั้งที่ $i =="
  $P $T/espefuse.py --chip esp32 --port /dev/esp32 --do-not-confirm set_flash_voltage 3.3V 2>&1 | tee -a $LOG | tail -4
  R=$($P $T/espefuse.py --chip esp32 --port /dev/esp32 summary 2>&1 | grep -i "Flash voltage")
  echo "$R" | tee -a $LOG
  echo "$R" | grep -q "set to 3.3V by efuse" && { echo "✅ สำเร็จ"; exit 0; }
  sleep 2
done
echo "❌ ยังไม่สำเร็จ — แจ้ง Claude ให้ตรวจ $LOG"
