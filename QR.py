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
CAMERA_INDEX = 0                # หมายเลขกล้อง (0 = กล้องตัวแรก)
RESEND_INTERVAL = 0.1        # ส่งคำสั่งซ้ำทุกๆ 0.5 วินาที
FRAME_WIDTH = 640               # ความกว้างของภาพ
FRAME_HEIGHT = 480              # ความสูงของภาพ

# ===== QR CODE MAPPING =====
# Dictionary สำหรับแปลงข้อมูล QR Code เป็นตำแหน่ง 1-8
QR_MAPPING = {
    'POS_A_COL1': 1,
    'POS_A_COL2': 2,
    'POS_B_COL1': 3,
    'POS_B_COL2': 4,
    'POS_C_COL1': 5,
    'POS_C_COL2': 6,
    'POS_D_COL1': 7,
    'POS_D_COL2': 8,
    # เพิ่ม Mapping ตามกติกาการแข่งขัน
    'STOP': 0,
    'HOME': 0
}

# ===== GLOBAL VARIABLES =====
ser = None                      # Serial connection object
last_sent_value = None          # ค่าที่ส่งล่าสุด
last_send_time = 0              # เวลาที่ส่งล่าสุด

# ===== SERIAL PORT FUNCTIONS =====
def find_arduino_port():
    """
    ค้นหา Serial Port ที่ Arduino เชื่อมต่ออยู่
    Returns: port path หรือ None ถ้าหาไม่เจอ
    """
    print("🔍 Searching for Arduino...")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # ค้นหา Arduino โดยดูจาก description
        if 'Arduino' in port.description or 'USB' in port.description or 'ACM' in port.device:
            print(f"✅ Found Arduino at: {port.device}")
            print(f"   Description: {port.description}")
            return port.device
    
    return None

def connect_arduino(port_path=None, retry=3):
    """
    เชื่อมต่อกับ Arduino ผ่าน Serial
    Args:
        port_path: path ของ serial port (ถ้าไม่ระบุจะค้นหาอัตโนมัติ)
        retry: จำนวนครั้งที่ลองใหม่
    Returns: serial object หรือ None
    """
    global ser
    
    # ถ้าไม่ระบุ port ให้ค้นหาเอง
    if port_path is None:
        port_path = find_arduino_port()
    
    if port_path is None:
        print("❌ ERROR: Arduino not found!")
        print("   Please check:")
        print("   1. Arduino is connected via USB")
        print("   2. USB cable supports data transfer")
        print("   3. Arduino is powered on")
        return None
    
    # พยายามเชื่อมต่อ
    for attempt in range(retry):
        try:
            print(f"🔌 Connecting to {port_path}... (Attempt {attempt + 1}/{retry})")
            ser = serial.Serial(port_path, BAUD_RATE, timeout=1)
            time.sleep(2)  # รอให้ Arduino reset และพร้อมใช้งาน
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
    """
    ส่งข้อมูลไปยัง Arduino
    Args:
        value: ตัวเลข 0-8 ที่จะส่ง
    """
    global ser, last_sent_value, last_send_time
    
    if ser is None or not ser.is_open:
        print("❌ Serial not connected!")
        return False
    
    try:
        # แปลง int เป็น string แล้วส่ง
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
    """
    อ่านและ Decode QR Code จากภาพ
    Args:
        frame: ภาพจากกล้อง (numpy array)
    Returns: list ของ decoded QR codes
    """
    # ใช้ pyzbar ในการ decode
    decoded_objects = pyzbar.decode(frame)
    return decoded_objects

def map_qr_to_destination(qr_data):
    """
    แปลงข้อมูล QR Code เป็นตำแหน่งปลายทาง (1-8)
    Args:
        qr_data: ข้อมูลที่อ่านได้จาก QR Code (string)
    Returns: ตัวเลข 0-8 หรือ None ถ้าไม่พบใน mapping
    """
    # ลบช่องว่างและแปลงเป็นตัวพิมพ์ใหญ่
    qr_data = qr_data.strip().upper()
    
    # ค้นหาใน mapping dictionary
    if qr_data in QR_MAPPING:
        return QR_MAPPING[qr_data]
    
    # ถ้าข้อมูลเป็นตัวเลข 1-8 โดยตรง
    try:
        value = int(qr_data)
        if 0 <= value <= 8:
            return value
    except ValueError:
        pass
    
    return None

def draw_qr_info(frame, decoded_objects):
    """
    วาดกรอบและข้อความบน QR Code ที่พบ
    Args:
        frame: ภาพจากกล้อง
        decoded_objects: list ของ QR codes ที่ decode แล้ว
    Returns: frame ที่วาดข้อมูลแล้ว, destination value
    """
    destination = None
    
    for obj in decoded_objects:
        # ดึงข้อมูลตำแหน่งของ QR Code
        points = obj.polygon
        
        # ถ้ามี polygon ไม่ครบ ให้ใช้ rect แทน
        if len(points) > 4:
            hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
            points = hull
        
        # วาดกรอบรอบ QR Code (สีเขียว)
        n = len(points)
        for i in range(n):
            cv2.line(frame, tuple(points[i]), tuple(points[(i+1) % n]), (0, 255, 0), 3)
        
        # ดึงข้อมูลจาก QR Code
        qr_data = obj.data.decode('utf-8')
        qr_type = obj.type
        
        # แปลงเป็นตำแหน่งปลายทาง
        destination = map_qr_to_destination(qr_data)
        
        # แสดงข้อความบน QR Code
        x = points[0][0]
        y = points[0][1] - 10
        
        # แสดงข้อมูล QR
        cv2.putText(frame, f"Data: {qr_data}", (x, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # แสดงตำแหน่งที่แปลงแล้ว
        if destination is not None:
            cv2.putText(frame, f"Destination: {destination}", (x, y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Unknown QR", (x, y - 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    return frame, destination

# ===== MAIN PROGRAM =====
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
    print(f"\n📷 Opening camera {CAMERA_INDEX}...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print("❌ ERROR: Cannot open camera!")
        print("   Please check:")
        print("   1. Camera is connected properly")
        print("   2. Camera permissions are granted")
        print("   3. No other program is using the camera")
        sys.exit(1)
    
    # ตั้งค่าความละเอียดของกล้อง
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    print("✅ Camera opened successfully!")
    print("\n" + "=" * 50)
    print("  Press 'q' to quit")
    print("=" * 50 + "\n")
    
    # Main loop
    try:
        while True:
            # อ่านภาพจากกล้อง
            ret, frame = cap.read()
            
            if not ret:
                print("❌ Failed to read frame from camera")
                break
            
            # Decode QR Code
            decoded_objects = decode_qr_code(frame)
            
            # วาดข้อมูลและดึง destination
            frame, destination = draw_qr_info(frame, decoded_objects)
            
            # ส่งข้อมูลไป Arduino
            current_time = time.time()
            
            if destination is not None:
                # ส่งใหม่ถ้าค่าเปลี่ยน หรือเวลาผ่านไป RESEND_INTERVAL
                if (destination != last_sent_value or 
                    current_time - last_send_time >= RESEND_INTERVAL):
                    send_to_arduino(destination)
            
            # แสดงสถานะบนหน้าจอ
            status_text = f"Last Sent: {last_sent_value if last_sent_value is not None else 'None'}"
            cv2.putText(frame, status_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # แสดงสถานะการเชื่อมต่อ
            connection_status = "Arduino: Connected" if (ser and ser.is_open) else "Arduino: Disconnected"
            connection_color = (0, 255, 0) if (ser and ser.is_open) else (0, 0, 255)
            cv2.putText(frame, connection_status, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, connection_color, 2)
            
            # แสดงภาพ
            cv2.imshow('QR Code Scanner - Press Q to quit', frame)
            
            # ตรวจสอบการกดปุ่ม
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                print("\n👋 Shutting down...")
                break
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Program interrupted by user")
    
    finally:
        # ปิดการเชื่อมต่อ
        print("🧹 Cleaning up...")
        
        # ส่งคำสั่งหยุดก่อนปิด
        if ser and ser.is_open:
            send_to_arduino(0)
            time.sleep(0.5)
            ser.close()
            print("✅ Serial connection closed")
        
        cap.release()
        cv2.destroyAllWindows()
        print("✅ Camera released")
        print("\n👋 Program ended successfully")

# ===== ENTRY POINT =====
if __name__ == "__main__":
    main()
