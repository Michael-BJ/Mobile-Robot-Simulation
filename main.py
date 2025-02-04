import serial
import time

# Konfigurasi port serial (sesuaikan dengan port Pico)
port = "/dev/ttyACM0"  # Untuk Linux/Mac
baud_rate = 9600
ser = serial.Serial(port, baud_rate, timeout=1)

try:
    # Buka koneksi serial
    with serial.Serial(port, baud_rate, timeout=1) as ser:
        print(f"Connected to {port}")
        print("Masukkan perintah ('maju', 'mundur', 'kiri', 'kanan') atau 'exit' untuk keluar:")

        while True:
            # Input perintah dari pengguna
            user_input = input("Input perintah: ").strip()
            
            if user_input.lower() == "exit":
                print("Program terminated.")
                break

            if user_input in ["maju", "mundur", "kiri", "kanan", "stop"]:
                # Kirim perintah ke serial
                ser.write((user_input + "\n").encode('utf-8'))
                print(f"Sent: {user_input}")

                # Baca feedback dari Pico
                line = ser.readline().decode().strip()
                if line:
                    data = line.split(",")
                    if len(data) == 3:
                        motor_index = int(data[0])
                        angular_speed = float(data[1])
                        linear_speed = float(data[2])
                        print(f"Roda {motor_index} : Kecepatan Sudut = {angular_speed : .2f} rad/s, Kecepatan Linear = {linear_speed:.2f} m/s")        

            else:
                print("Perintah tidak valid. Gunakan 'maju', 'mundur', 'kiri', atau 'kanan'.")
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Program terminated.")
