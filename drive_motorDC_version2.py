# program ini merupakan program dari Bryan
# merupakan program basic untuk debugging komunikasi rasp dengan pico untuk menjalankan motor DC
# akan tetapi pada program berikut motor DC masi bergerak berbeda satu sama lain


from machine import Pin, Timer, PWM
import select
import sys

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

# Fungsi untuk mengolah data
def process_data():
    global input_data
    if input_data is not None:
        drive_mobile_robot(input_data)
        input_data = None

# Timer untuk memeriksa input secara berkala
uart_timer = Timer(-1)
uart_timer.init(period=100, mode=Timer.PERIODIC, callback=handle_uart_input)

# Fungsi untuk mengendalikan motor, tapi kebalik plus negatif
# def set_motor_speed(motor_index, speed):
#     rpwm, lpwm = motor_pwms[motor_index]
#     if speed > 0:
#         rpwm.duty_u16(int(speed * 65535 / 100))
#         lpwm.duty_u16(0)
#     elif speed < 0:
#         lpwm.duty_u16(int(-speed * 65535 / 100))
#         rpwm.duty_u16(0)
#     else:
#         rpwm.duty_u16(0)
#         lpwm.duty_u16(0)
        
def set_motor_speed(motor_index, speed):
    rpwm, lpwm = motor_pwms[motor_index]
    
    # Positif = Maju, Negatif = Mundur
    if speed > 0:
        lpwm.duty_u16(int(speed * 65535 / 100))  # Balik arah PWM jika perlu
        rpwm.duty_u16(0)
    elif speed < 0:
        rpwm.duty_u16(int(-speed * 65535 / 100))  # Balik arah PWM jika perlu
        lpwm.duty_u16(0)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)


def calculate_speeds():
    for i in range(4):
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, last_encoder_time[i]) / 1000.0  # Dalam detik
        
        if delta_time > 0:
            # Kecepatan sudut dalam radian per detik
            angular_speed = (encoder_counts[i] / PULSES_PER_REVOLUTION) * (2 * 3.14159) / delta_time
            
            # Kecepatan linear dalam meter per detik
            linear_speed = angular_speed * WHEEL_RADIUS
            
            print(f"Roda {i}: Kecepatan Sudut = {angular_speed:.2f} rad/s, Kecepatan Linear = {linear_speed:.2f} m/s")
            
            # Reset encoder counts dan waktu
            encoder_counts[i] = 0
            last_encoder_time[i] = current_time


# **Fungsi untuk mengontrol gerakan Mecanum Wheel**
def forward():
    led.value(1)
    set_motor_speed(0, 20)  # FL
    set_motor_speed(1, 20)  # FR
    set_motor_speed(2, 20)  # BR
    set_motor_speed(3, 20)  # BL
    print("Maju")

def backward():
    led.value(1)
    set_motor_speed(0, -20)  # FR
    set_motor_speed(1, -20)  # FL
    set_motor_speed(2, -20)  # BL
    set_motor_speed(3, -20)  # BR
    print("Mundur")

def strafe_right():
    led.value(1)
    set_motor_speed(0, -20)  # FR maju
    set_motor_speed(1, 20)  # FL mundur
    set_motor_speed(2, -20)  # BL mundur
    set_motor_speed(3, 20)  # BR maju
    print("Strafe Kanan")

def strafe_left():
    led.value(1)
    set_motor_speed(0, 20)  # FR mundur
    set_motor_speed(1, -20)  # FL maju
    set_motor_speed(2, 20)  # BL maju
    set_motor_speed(3, -20)  # BR mundur
    print("Strafe Kiri")

def rotate_right():
    led.value(1)
    set_motor_speed(0, -20)  # FR maju
    set_motor_speed(1, 20)  # FL mundur
    set_motor_speed(2, 20)  # BL maju
    set_motor_speed(3, -20)  # BR mundur
    print("Rotasi Kanan")

def rotate_left():
    led.value(1)
    set_motor_speed(0, -20)  # FR mundur
    set_motor_speed(1, 20)  # FL maju
    set_motor_speed(2, -20)  # BL mundur
    set_motor_speed(3, 20)  # BR maju
    print("Rotasi Kiri")

def stop():
    led.value(0)
    for i in range(4):
        set_motor_speed(i, 0)
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
    else:
        print(f"Perintah tidak dikenal: {command}")

# Loop utama untuk memproses data
while True:
    process_data()

