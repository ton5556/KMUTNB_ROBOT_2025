#!/usr/bin/env python3
"""
========================================
Raspberry Pi 5 - QR Code Scanner
สำหรับการแข่งขันหุ่นยนต์ขนส่งของอัตโนมัติ
========================================
ใช้งาน: python3 qr_scanner.py
หยุดโปรแกรม: กด 'q' หรือ Ctrl+C
"""

import cv2
from pyzbar import pyzbar
import serial
import serial.tools.list_ports
import time
import sys
import numpy as np

# ===== CONFIGURATION =====
BAUD_RATE = 115200              # ความเร็ว Serial (ต้องตรงกับ Arduino)
CAMERA_INDEX = 19               # ⭐ เปลี่ยนเป็น 19 สำหรับ /dev/video19
RESEND_INTERVAL = 0.1           # ส่งคำสั่งซ้ำทุกๆ 0.1 วินาที
FRAME_WIDTH = 640               # ความกว้างของภาพ
FRAME_HEIGHT = 480              # ความสูงของภาพ

# ... (โค้ดส่วนอื่นเหมือนเดิม)

def main():
    """
    โปรแกรมหลัก
    """
    global ser, last_sent_value, last_send_time
    
    print("=" * 50)
    print("  Raspberry Pi QR Code Scanner")
    print("  Robot Control System")
    print("=" * 50)
    
    # เชื่อมต่อ Arduino
    ser = connect_arduino()
    if ser is None:
        print("\n⚠️  Running without Arduino connection (Preview mode)")
        print("   Connect Arduino and restart to enable control")
    
    # เปิดกล้อง
    print(f"\n📷 Opening camera {CAMERA_INDEX} (/dev/video{CAMERA_INDEX})...")
    
    # ลองหลายวิธี
    cap = None
    
    # ลองแบบที่ 1: ใช้ index กับ V4L2
    try:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if cap.isOpened():
            print("✅ Camera opened with V4L2 backend")
    except:
        pass
    
    # ลองแบบที่ 2: ใช้ index ธรรมดา
    if cap is None or not cap.isOpened():
        try:
            cap = cv2.VideoCapture(CAMERA_INDEX)
            if cap.isOpened():
                print("✅ Camera opened with default backend")
        except:
            pass
    
    # ลองแบบที่ 3: ใช้ path โดยตรง
    if cap is None or not cap.isOpened():
        try:
            cap = cv2.VideoCapture(f'/dev/video{CAMERA_INDEX}')
            if cap.isOpened():
                print("✅ Camera opened with direct path")
        except:
            pass
    
    if cap is None or not cap.isOpened():
        print("❌ ERROR: Cannot open camera!")
        print("   Please check:")
        print("   1. Camera is connected properly")
        print("   2. Camera permissions are granted")
        print("   3. No other program is using the camera")
        print(f"   4. /dev/video{CAMERA_INDEX} exists")
        sys.exit(1)
    
    # ตั้งค่าความละเอียดของกล้อง
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # อ่านความละเอียดจริงที่ได้
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ Camera resolution: {actual_width}x{actual_height}")
    
    print("\n" + "=" * 50)
    print("  Press 'q' to quit")
    print("=" * 50 + "\n")
    
    # ... (โค้ดส่วนที่เหลือเหมือนเดิม)
