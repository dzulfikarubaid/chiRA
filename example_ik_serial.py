import numpy as np
import serial
import time
from inverse_kinematics import calculate_angles

SERIAL_PORT = 'COM5'
BAUD_RATE = 115200
TIME_OUT = 2

EE_OPEN_ANGLE = 90.0

INITIAL_ANGLES = (0.0, 80.0, 160.0)

def send_serial_command(ser_connection, command_type, angles, ee_angle, move_time=1000):
    if not ser_connection or not ser_connection.is_open:
        print("Koneksi serial tidak aktif. Perintah tidak dikirim.")
        return False

    theta1, theta2, theta3 = angles
    theta1 = np.clip(theta1, 0, 180)
    theta2 = np.clip(theta2, 0, 180)
    theta3 = np.clip(theta3, 0, 180)
    ee_angle = np.clip(ee_angle, 0, 180)

    data_to_send = (
        f"<{command_type},"
        f"{ee_angle:.2f},"
        f"{theta1:.2f},"
        f"{theta2:.2f},"
        f"{theta3:.2f},"
        f"{move_time},"
        f"{move_time},"
        f"{move_time},"
        f"{move_time}>"
    )
    print(f"Mengirim: {data_to_send}")
    try:
        ser_connection.write(data_to_send.encode())
        time.sleep(0.1)
        return True
    except serial.SerialException as e:
        print(f"Error saat menulis ke port serial: {e}")
        return False

if __name__ == "__main__":
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIME_OUT)
        print(f"Terhubung ke port serial: {SERIAL_PORT}")
        time.sleep(2)
        print("Arduino terhubung. Mengirim robot ke posisi awal (gripper terbuka)...")
        send_serial_command(ser, "MOVE_ROBOT_ARM", INITIAL_ANGLES, EE_OPEN_ANGLE)
        time.sleep(3)

    except serial.SerialException as e:
        print(f"Gagal membuka port serial {SERIAL_PORT}: {e}")
        print("Pastikan Arduino terhubung dan port yang benar telah dipilih.")
        exit()

    print("\n--- Masukkan Koordinat Target (dalam cm) ---")
    print("Ketik 'exit' untuk keluar.")

    while True:
        x_str = input("Masukkan koordinat X (cm): ")
        if x_str.lower() == 'exit':
            break
        x = float(x_str)

        y_str = input("Masukkan koordinat Y (cm): ")
        if y_str.lower() == 'exit':
            break
        y = float(y_str)

        z_str = input("Masukkan koordinat Z (cm): ")
        if z_str.lower() == 'exit':
            break
        z = float(z_str)

        print(f"\nMenghitung sudut untuk X={x:.2f}, Y={y:.2f}, Z={z:.2f} cm...")

        theta_1_servo, theta_2_servo, theta_3_servo, is_out_of_reach = calculate_angles(x, y, z)
        
        calculated_angles = (theta_1_servo, theta_2_servo, theta_3_servo)

        if is_out_of_reach:
            print(f"The point is out of reach. Servo angles: Theta 1: {calculated_angles[0]:.2f}, Theta 2: {calculated_angles[1]:.2f}, Theta 3: {calculated_angles[2]:.2f}")
        else:
            print(f"Sudut Terhitung: Theta1={calculated_angles[0]:.2f}, "
            f"Theta2={calculated_angles[1]:.2f}, Theta3={calculated_angles[2]:.2f}")
            send_serial_command(ser, "MOVE_ROBOT_ARM", calculated_angles, EE_OPEN_ANGLE)
            time.sleep(2)

               
    if ser and ser.is_open:
        print("Mengembalikan robot ke posisi awal sebelum menutup port serial...")
        send_serial_command(ser, "MOVE_ROBOT_ARM", INITIAL_ANGLES, EE_OPEN_ANGLE)
        time.sleep(2)
        ser.close()
        print("Port serial ditutup.")