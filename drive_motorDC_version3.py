from machine import Pin, Timer, PWM
import select
import sys
import time

machine.freq(200000000)

# Inisialisasi LED bawaan
led = Pin(25, Pin.OUT)

# Variabel global untuk menyimpan data yang diterima
input_data = None

# Define GPIO pins for each encoder channel
encoder_channels = [(8, 9), (27, 26), (10, 11), (21, 20)]  # Encoder untuk keempat roda

# Define driver pins for Mecanum wheels (FR, FL, BL, BR)
motor_pins = [(6, 7), (19, 18), (14, 15), (17, 16)]  # (RPWM, LPWM) untuk setiap motor

# PWM frequency for motor control
PWM_FREQ = 1000

# Jumlah pulsa per putaran encoder (sesuaikan dengan spesifikasi encoder Anda)
PULSES_PER_REVOLUTION = 884  # Contoh: 20 pulsa per putaran

# Jari-jari roda dalam meter (sesuaikan dengan ukuran roda Anda)
WHEEL_RADIUS = 0.03  # Contoh: 5 cm

# Initialize motor PWM controls
motor_pwms = []
for rpwm_pin, lpwm_pin in motor_pins:
    rpwm = PWM(Pin(rpwm_pin))
    lpwm = PWM(Pin(lpwm_pin))
    rpwm.freq(PWM_FREQ)
    lpwm.freq(PWM_FREQ)
    motor_pwms.append((rpwm, lpwm))

# Initialize encoder pins
encoders = []
for channel_a, channel_b in encoder_channels:
    encoders.append((Pin(channel_a, Pin.IN), Pin(channel_b, Pin.IN)))

# Variabel untuk menyimpan jumlah pulsa encoder
encoder_counts = [0, 0, 0, 0]

# Fungsi untuk menangani interrupt encoder
def encoder_isr(motor_index):
    def isr(pin):
        global encoder_counts
        encoder_counts[motor_index] += 1
    return isr

# Attach interrupt untuk setiap encoder
for i, (channel_a, channel_b) in enumerate(encoders):
    channel_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr(i))

# Fungsi untuk menangani input UART
def handle_uart_input(timer):
    global input_data
    try:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            data = sys.stdin.readline().strip()
            if data:
                input_data = data
    except Exception as e:
        print(f"Error reading input: {e}")

# Timer untuk memeriksa input secara berkala
uart_timer = Timer(-1)
uart_timer.init(period=100, mode=Timer.PERIODIC, callback=handle_uart_input)

speed_timer = Timer(-1)
speed_timer.init(period=1000, mode=Timer.PERIODIC, callback=lambda t: calculate_speeds())

# Fungsi untuk mengendalikan motor dengan umpan balik encoder
def set_motor_speed_with_feedback(motor_index, target_speed):
    global encoder_counts
    rpwm, lpwm = motor_pwms[motor_index]
    
    # Konversi target_speed ke duty cycle
    target_duty = int(abs(target_speed) * 65535 / 100)
    
    # Hitung kecepatan aktual berdasarkan encoder counts
    current_count = encoder_counts[motor_index]
    time.sleep(0.1)  # Tunggu 100ms untuk mengukur kecepatan
    delta_count = encoder_counts[motor_index] - current_count
    actual_speed = delta_count  # Kecepatan aktual dalam pulsa per 100ms
    
    # Sesuaikan PWM berdasarkan perbedaan antara target_speed dan actual_speed
    if target_speed > 0:
        if actual_speed < target_speed:
            lpwm.duty_u16(min(target_duty + 1000, 65535))  # Tambahkan sedikit kecepatan
        elif actual_speed > target_speed:
            lpwm.duty_u16(max(target_duty - 1000, 0))  # Kurangi sedikit kecepatan
        else:
            lpwm.duty_u16(target_duty)
        rpwm.duty_u16(0)
    elif target_speed < 0:
        if actual_speed < -target_speed:
            rpwm.duty_u16(min(target_duty + 1000, 65535))  # Tambahkan sedikit kecepatan
        elif actual_speed > -target_speed:
            rpwm.duty_u16(max(target_duty - 1000, 0))  # Kurangi sedikit kecepatan
        else:
            rpwm.duty_u16(target_duty)
        lpwm.duty_u16(0)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)

# Fungsi untuk mengatur kecepatan semua motor secara bersamaan
def set_all_motors_speed(target_speed):
    for i in range(4):
        set_motor_speed_with_feedback(i, target_speed)

def calculate_speeds():
    global encoder_counts, last_encoder_time
    for i in range(4):
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, last_encoder_time[i]) / 1000.0  # Dalam detik

        if delta_time > 0:
            # Kecepatan sudut dalam radian per detik
            angular_speed = (encoder_counts[i] / PULSES_PER_REVOLUTION) * (2 * 3.14159) / delta_time

            # Kecepatan linear dalam meter per detik
            linear_speed = angular_speed * WHEEL_RADIUS

            # Kirim data ke Raspberry Pi melalui serial USB
            sys.stdout.write(f"{i},{angular_speed:.2f},{linear_speed:.2f}\n")
            sys.stdout.flush()

            # Reset encoder counts dan waktu
            encoder_counts[i] = 0
            last_encoder_time[i] = current_time


# **Fungsi untuk mengontrol gerakan Mecanum Wheel**
def forward():
    led.value(1)
    set_all_motors_speed(40)  # Semua motor maju
    time.sleep(5)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai

def backward():
    led.value(1)
    set_all_motors_speed(-40)  # Semua motor mundur
    time.sleep(1)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai
    
def strafe_right():
    led.value(1)
    set_motor_speed_with_feedback(0, 40)  # FL maju
    set_motor_speed_with_feedback(1, -40)  # FR mundur
    set_motor_speed_with_feedback(2, -40)  # BL mundur
    set_motor_speed_with_feedback(3, 40)  # BR maju
    print("Strafe Kanan")
    time.sleep(1)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai
    
def strafe_left():
    led.value(1)
    set_motor_speed_with_feedback(0, -40)  # FR mundur
    set_motor_speed_with_feedback(1, 40)  # FL maju
    set_motor_speed_with_feedback(2, 40)  # BL maju
    set_motor_speed_with_feedback(3, -40)  # BR mundur
    print("Strafe Kiri")
    time.sleep(1)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai

def rotate_right():
    led.value(1)
    set_motor_speed_with_feedback(0, -40)  # FR maju
    set_motor_speed_with_feedback(1, 40)  # FL mundur
    set_motor_speed_with_feedback(2, 40)  # BL maju
    set_motor_speed_with_feedback(3, -40)  # BR mundur
    print("Rotasi Kanan")
    time.sleep(1)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai

def rotate_left():
    led.value(1)
    set_motor_speed_with_feedback(0, -40)  # FR mundur
    set_motor_speed_with_feedback(1, 40)  # FL maju
    set_motor_speed_with_feedback(2, -40)  # BL mundur
    set_motor_speed_with_feedback(3, 40)  # BR maju
    print("Rotasi Kiri")
    time.sleep(1)  # Jalankan motor selama 1 detik
    stop()  # Hentikan motor setelah selesai

def stop():
    led.value(0)
    set_all_motors_speed(0)  # Semua motor berhenti
    print("Berhenti")

# Fungsi untuk mengendalikan robot berdasarkan perintah
def drive_mobile_robot(command):
    if command == "maju":
        forward()
    elif command == "mundur":
        backward()
    elif command == "kanan":
        strafe_right()
    elif command == "kiri":
        strafe_left()
    elif command == "rotate_kanan":
        rotate_right()
    elif command == "rotate_kiri":
        rotate_left()
    elif command == "stop":
        stop()
    elif command.startswith("speed"):
        try:
            speed = int(command.split()[1])
            set_all_motors_speed(speed)  # Set kecepatan semua motor
            print(f"Kecepatan disetel ke {speed}%")
        except ValueError:
            print("Perintah tidak valid untuk speed")
    else:
        print(f"Perintah tidak dikenal: {command}")

# Fungsi untuk mengolah data
def process_data():
    global input_data
    if input_data is not None:
        drive_mobile_robot(input_data)
        input_data = None

# Loop utama untuk memproses data
while True:
    process_data()
