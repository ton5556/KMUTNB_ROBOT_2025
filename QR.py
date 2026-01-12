#!/usr/bin/env python3
"""
========================================
Raspberry Pi 5 - QR Code Scanner
Camera Module 3 Version
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
from picamera2 import Picamera2

# ===== CONFIGURATION =====
BAUD_RATE = 115200              # ความเร็ว Serial (ต้องตรงกับ Arduino)
RESEND_INTERVAL = 0.1           # ส่งคำสั่งซ้ำทุกๆ 0.1 วินาที
FRAME_WIDTH = 640               # ความกว้างของภาพ
FRAME_HEIGHT = 480              # ความสูงของภาพ

# ===== QR CODE MAPPING =====
QR_MAPPING = {
    'POS_A_COL1': 1,
    'POS_A_COL2': 2,
    'POS_B_COL1': 3,
    'POS_B_COL2': 4,
    'POS_C_COL1': 5,
    'POS_C_COL2': 6,
    'POS_D_COL1': 7,
    'POS_D_COL2': 8,
    'STOP': 0,
    'HOME': 0
}

# ===== GLOBAL VARIABLES =====
ser = None
last_sent_value = None
last_send_time = 0
picam2 = None

# ===== CAMERA FUNCTIONS =====
def init_picamera():
    """
    เริ่มต้น Pi Camera Module 3
    Returns: Picamera2 object หรือ None
    """
    global picam2
    
    try:
        print("📷 Initializing Camera Module 3...")
        picam2 = Picamera2()
        
        # แสดงข้อมูลกล้อง
        camera_properties = picam2.camera_properties
        print(f"   Model: {camera_properties.get('Model', 'Unknown')}")
        
        # ตั้งค่า configuration สำหรับ Camera Module 3
        # ใช้ main stream สำหรับ QR scanning
        config = picam2.create_preview_configuration(
            main={
                "size": (FRAME_WIDTH, FRAME_HEIGHT),
                "format": "RGB888"
            },
            controls={
                "FrameRate": 30,
                # ปิด autofocus สำหรับ QR scanning (ถ้ารองรับ)
                "AfMode": 0,  # Manual focus
            }
        )
        
        picam2.configure(config)
        
        # เริ่มกล้อง
        picam2.start()
        
        # รอให้กล้องเริ่มทำงานและ stabilize
        print("   Warming up camera...")
        time.sleep(2)
        
        # ทดสอบถ่ายภาพ
        test_frame = picam2.capture_array()
        if test_frame is not None:
            print(f"✅ Camera Module 3 initialized successfully!")
            print(f"   Resolution: {test_frame.shape[1]}x{test_frame.shape[0]}")
            return picam2
        else:
            print("❌ Failed to capture test frame")
            return None
            
    except Exception as e:
        print(f"❌ Error initializing Camera Module 3: {e}")
        print("\n   Troubleshooting:")
        print("   1. Check camera cable connection")
        print("   2. Enable camera: sudo raspi-config → Interface Options → Camera")
        print("   3. Update system: sudo apt update && sudo apt upgrade")
        print("   4. Test camera: libcamera-hello")
        return None

def read_frame():
    """
    อ่านภาพจาก Pi Camera
    Returns: frame (numpy array) หรือ None
    """
    global picam2
    
    if picam2 is None:
        return None
    
    try:
        # Capture array จะได้ภาพในรูปแบบ RGB888
        frame = picam2.capture_array()
        return frame
    except Exception as e:
        print(f"❌ Error capturing frame: {e}")
        return None

def close_camera():
    """ปิดกล้อง"""
    global picam2
    
    if picam2 is not None:
        try:
            picam2.stop()
            picam2.close()
            print("✅ Camera closed")
        except:
            pass

# ===== SERIAL PORT FUNCTIONS =====
def find_arduino_port():
    """ค้นหา Serial Port ที่ Arduino เชื่อมต่ออยู่"""
    print("🔍 Searching for Arduino...")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # ใน Raspberry Pi มักจะเป็น /dev/ttyACM0 หรือ /dev/ttyUSB0
        if 'Arduino' in port.description or 'USB' in port.description or 'ACM' in port.device:
            print(f"✅ Found Arduino at: {port.device}")
            print(f"   Description: {port.description}")
            return port.device
    
    return None

def connect_arduino(port_path=None, retry=3):
    """เชื่อมต่อกับ Arduino ผ่าน Serial"""
    global ser
    
    if port_path is None:
        port_path = find_arduino_port()
    
    if port_path is None:
        print("❌ ERROR: Arduino not found!")
        print("   Available ports:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"   - {port.device}: {port.description}")
        return None
    
    for attempt in range(retry):
        try:
            print(f"🔌 Connecting to {port_path}... (Attempt {attempt + 1}/{retry})")
            ser = serial.Serial(port_path, BAUD_RATE, timeout=1)
            time.sleep(2)  # รอให้ Arduino reset
            print("✅ Connected to Arduino successfully!")
            return ser
        except serial.SerialException as e:
            print(f"⚠️  Connection failed: {e}")
            if attempt < retry - 1:
                time.sleep(1)
            else:
                print("❌ Failed to connect after multiple attempts")
                return None
    
    return None

def send_to_arduino(value):
    """ส่งข้อมูลไปยัง Arduino"""
    global ser, last_sent_value, last_send_time
    
    if ser is None or not ser.is_open:
        return False
    
    try:
        command = str(value)
        ser.write(command.encode())
        
        last_sent_value = value
        last_send_time = time.time()
        
        print(f"📤 Sent to Arduino: {value}")
        return True
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        return False

# ===== QR CODE PROCESSING =====
def decode_qr_code(frame):
    """อ่านและ Decode QR Code จากภาพ"""
    decoded_objects = pyzbar.decode(frame)
    return decoded_objects

def map_qr_to_destination(qr_data):
    """แปลงข้อมูล QR Code เป็นตำแหน่งปลายทาง (0-8)"""
    qr_data = qr_data.strip().upper()
    
    if qr_data in QR_MAPPING:
        return QR_MAPPING[qr_data]
    
    try:
        value = int(qr_data)
        if 0 <= value <= 8:
            return value
    except ValueError:
        pass
    
    return None

def draw_qr_info(frame, decoded_objects):
    """วาดกรอบและข้อความบน QR Code ที่พบ"""
    destination = None
    
    for obj in decoded_objects:
        points = obj.polygon
        
        if len(points) > 4:
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            points = hull
        
        # วาดกรอบรอบ QR Code (สีเขียว)
        n = len(points)
        for i in range(n):
            cv2.line(frame, tuple(points[i]), tuple(points[(i+1) % n]), (0, 255, 0), 3)
        
        # ดึงข้อมูลจาก QR Code
        qr_data = obj.data.decode('utf-8')
        destination = map_qr_to_destination(qr_data)
        
        # แสดงข้อความบน QR Code
        x = points[0][0]
        y = points[0][1] - 10
        
        cv2.putText(frame, f"Data: {qr_data}", (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if destination is not None:
            cv2.putText(frame, f"Destination: {destination}", (x, y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Unknown QR", (x, y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    return frame, destination

# ===== MAIN PROGRAM =====
def main():
    """โปรแกรมหลัก"""
    global ser, last_sent_value, last_send_time, picam2
    
    print("=" * 60)
    print("  Raspberry Pi 5 - QR Code Scanner")
    print("  Camera Module 3")
    print("  Robot Control System")
    print("=" * 60)
    
    # เชื่อมต่อ Arduino
    ser = connect_arduino()
    if ser is None:
        print("\n⚠️  Running without Arduino connection (Preview mode)")
        print("   You can still test QR scanning")
    
    # เปิดกล้อง
    print()
    picam2 = init_picamera()
    
    if picam2 is None:
        print("\n❌ ERROR: Cannot initialize Camera Module 3!")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("  System Ready!")
    print("  Press 'q' in the preview window to quit")
    print("=" * 60 + "\n")
    
    # Main loop
    frame_count = 0
    fps_start_time = time.time()
    fps = 0
    
    try:
        while True:
            # อ่านภาพจากกล้อง
            frame = read_frame()
            
            if frame is None:
                print("❌ Failed to read frame from camera")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            
            # คำนวณ FPS ทุกๆ 30 frames
            if frame_count % 30 == 0:
                fps_end_time = time.time()
                fps = 30 / (fps_end_time - fps_start_time)
                fps_start_time = fps_end_time
            
            # Decode QR Code
            decoded_objects = decode_qr_code(frame)
            
            # วาดข้อมูลและดึง destination
            frame, destination = draw_qr_info(frame, decoded_objects)
            
            # ส่งข้อมูลไป Arduino
            current_time = time.time()
            
            if destination is not None:
                if (destination != last_sent_value or 
                    current_time - last_send_time >= RESEND_INTERVAL):
                    send_to_arduino(destination)
            
            # แสดงสถานะบนหน้าจอ
            # Background สำหรับข้อความ
            cv2.rectangle(frame, (0, 0), (400, 120), (0, 0, 0), -1)
            
            # แสดงค่าที่ส่งล่าสุด
            status_text = f"Last Sent: {last_sent_value if last_sent_value is not None else 'None'}"
            cv2.putText(frame, status_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # แสดงสถานะการเชื่อมต่อ Arduino
            connection_status = "Arduino: Connected" if (ser and ser.is_open) else "Arduino: Disconnected"
            connection_color = (0, 255, 0) if (ser and ser.is_open) else (0, 0, 255)
            cv2.putText(frame, connection_status, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, connection_color, 2)
            
            # แสดง FPS
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # แสดง QR Count
            qr_count = len(decoded_objects)
            if qr_count > 0:
                cv2.putText(frame, f"QR Codes: {qr_count}", (10, 115), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # แสดงภาพ
            cv2.imshow('Camera Module 3 - QR Scanner (Press Q to quit)', frame)
            
            # ตรวจสอบการกดปุ่ม
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                print("\n👋 Shutting down...")
                break
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Program interrupted by user (Ctrl+C)")
    
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🧹 Cleaning up...")
        
        # ส่งคำสั่งหยุดก่อนปิด
        if ser and ser.is_open:
            print("   Sending STOP command to Arduino...")
            send_to_arduino(0)
            time.sleep(0.5)
            ser.close()
            print("✅ Serial connection closed")
        
        # ปิดกล้อง
        close_camera()
        
        # ปิด OpenCV windows
        cv2.destroyAllWindows()
        
        print("✅ Program ended successfully")
        print("\n" + "=" * 60)

# ===== ENTRY POINT =====
if __name__ == "__main__":
    main()
