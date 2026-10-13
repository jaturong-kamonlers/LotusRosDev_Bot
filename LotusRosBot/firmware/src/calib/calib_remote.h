// calib_remote.h — สั่งโหมดคาลิเบรตจากหน้าเว็บ (ดูรายการคำสั่งใน calib_remote.cpp)
#ifndef LRB_CALIB_REMOTE_H
#define LRB_CALIB_REMOTE_H

#include <Arduino.h>

bool calib_bootPoll();                    // ระหว่างนับถอยหลังตอนบูต: true = เว็บขอเข้าโหมดคาลิเบรต (@CALIB)
bool calib_remotePoll();                  // ในหน้าเมนู: มีคำสั่ง '@' → เข้าโหมดเว็บ (คืน true เมื่อกลับมา)
void calib_remoteLoop(const String &first);

#endif
