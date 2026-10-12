#!/bin/bash
# flash.sh — build / upload เฟิร์มแวร์ ESP32 จาก Jetson อย่างปลอดภัย (ใช้จาก terminal หรือ Task ใน VS Code)
#   bash ~/LotusRosDev_Bot/tools/flash.sh build     # คอมไพล์อย่างเดียว (ไม่แตะบอร์ด)
#   bash ~/LotusRosDev_Bot/tools/flash.sh upload    # build + สำรอง .bin + ปิด agent + เขียน + ตรวจ
#   bash ~/LotusRosDev_Bot/tools/flash.sh verify    # ตรวจว่าบอร์ดมีไฟล์ล่าสุดที่ build ไว้
# ใช้ /dev/esp32 เสมอ (ไม่ใช้ ttyUSB0 ของ platformio.ini — กันเขียนผิดใส่ LiDAR) · baud 115200 (เสถียรบน hub นี้)
set -u
REPO=~/LotusRosDev_Bot; FW=$REPO/firmware; BIN=$FW/.pio/build/myrobot/firmware.bin
PIO=~/.platformio/penv/bin/pio; PY=~/.platformio/penv/bin/python; ESPT=~/.platformio/packages/tool-esptoolpy/esptool.py
PORT=/dev/esp32; MODE=${1:-build}
say(){ printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
die(){ printf '\n\033[1;31m❌ %s\033[0m\n' "$*"; exit 1; }

ver=$(grep -o 'LOTUS_FW_VERSION "[^"]*"' $FW/src/firmware.ino | cut -d'"' -f2)
say "เฟิร์มแวร์ $ver · git $(git -C $REPO rev-parse --short HEAD)$(git -C $REPO diff --quiet || echo ' (มีไฟล์แก้ยังไม่ commit)')"

if [ "$MODE" = build ] || [ "$MODE" = upload ]; then
  say "1) build (pio run -e myrobot)"
  OUT=$(cd $FW && $PIO run -e myrobot 2>&1); RC=$?
  echo "$OUT" | grep -E "RAM:|Flash:|error:|Error|SUCCESS|FAILED" || true
  [ $RC -eq 0 ] && [ -f $BIN ] || die "build ไม่สำเร็จ — ดู error ด้านบน (เต็ม ๆ: cd $FW && $PIO run -e myrobot)"
  [ "$MODE" = build ] && { say "✅ build เสร็จ: $BIN ($(stat -c %s $BIN) bytes) — ยังไม่ได้เขียนลงบอร์ด"; exit 0; }
fi

if [ "$MODE" = upload ]; then
  [ -e $PORT ] || die "ไม่พบ $PORT — เสียบสาย ESP32 / ตรวจ udev (บทที่ 11.4)"
  say "2) สำรองไฟล์ที่จะเขียน → ~/fw_backup/"
  mkdir -p ~/fw_backup; B=~/fw_backup/firmware_${ver}_$(date +%Y%m%d_%H%M%S).bin; cp $BIN "$B"; echo "$B"
  if pgrep -f "[m]icro_ros_agent" >/dev/null; then
    say "3) ปิด agent (ต้องว่างพอร์ต) — หุ่นต้องจอดนิ่ง"
    read -r -p "agent กำลังทำงาน ปิดแล้ว flash ต่อ? พิมพ์ y: " a; [ "$a" = y ] || die "ยกเลิก"
    bash ~/stop.sh all >/dev/null 2>&1; sleep 2
  fi
  say "4) เขียนลง ESP32 (0x10000 · 115200)"
  ok=0
  for i in 1 2 3; do
    $PY $ESPT --chip esp32 --port $PORT --baud 115200 write_flash -z 0x10000 $BIN 2>&1 | grep -E "Wrote|Hash|fatal|Failed" && \
    $PY $ESPT --chip esp32 --port $PORT --baud 115200 verify_flash 0x10000 $BIN 2>&1 | grep -q "digest matched" && { ok=1; break; }
    echo "ลองใหม่ครั้งที่ $((i+1))…"; sleep 2
  done
  [ $ok = 1 ] || die "เขียนไม่สำเร็จ — ตรวจสาย / ลองใหม่ / ย้อนเวอร์ชันด้วยไฟล์ใน ~/fw_backup (บทที่ 17.10)"
  say "✅ flash + verify ผ่าน · บอร์ดรีเซ็ตแล้ว จอต้องขึ้น FW $ver · ต่อไป: bash ~/agent.sh (หรือ start.sh)"
  exit 0
fi

if [ "$MODE" = verify ]; then
  $PY $ESPT --chip esp32 --port $PORT --baud 115200 verify_flash 0x10000 $BIN 2>&1 | grep -E "verify|fatal" ; exit 0
fi
die "ใช้: flash.sh build | upload | verify"
