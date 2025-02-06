# 05-02-2025
# Michael Bryan Jahanto

# Program Raspberry Pi untuk mengirim perintah dan menerima feedback dari Pico

import serial
import time

# Konfigurasi port serial
port = "/dev/ttyACM0"  # Sesuaikan dengan port Pico
baud_rate = 9600

try:
    # Buka koneksi serial
    with serial.Serial(port, baud_rate, timeout=1) as ser:
        print(f"Connected to {port}")
        print("Masukkan perintah ('maju', 'mundur', 'kiri', 'kanan', 'stop') atau 'exit' untuk keluar:")

        while True:
            # Input perintah dari pengguna
            user_input = input("Input perintah: ").strip().lower()

            if user_input == "exit":
                print("Program terminated.")
                break

            if user_input in ["maju", "mundur", "kiri", "kanan", "stop"]:
                # Kirim perintah ke serial
                try:
                    ser.write((user_input + "\n").encode('utf-8'))
                    print(f"Sent: {user_input}")

                    start_time = time.time()
                    feedback_received = False

                    while time.time() - start_time < 2:  # Timeout 2 detik
                        feedback = ser.readline().decode('utf-8').strip()
                        if feedback:
                            print("Received feedback:")
                            roda_data = feedback.split(";")
                            for roda in roda_data:
                                if roda:  # Pastikan tidak ada string kosong
                                    print(roda)
                            feedback_received = True
                            break
                    
                    if not feedback_received:
                        print("No feedback received. Check the connection.")

                except serial.SerialException as e:
                    print(f"Error communicating with Pico: {e}")

            else:
                print("Perintah tidak valid. Gunakan 'maju', 'mundur', 'kiri', 'kanan', atau 'stop'.")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Program terminated.")
