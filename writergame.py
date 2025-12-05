import pygame
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200
WIDTH, HEIGHT = 600, 400

X_MIN, X_MAX = -55, 55
Y_MIN, Y_MAX = 80, 140

PEN_UP = 80
PEN_DOWN = 150


MIN_DISTANCE = 10
RATE_LIMIT = 0.1
last_command_time = 0

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print("Connected to ESP32")
except Exception as e:
    print(f"Error connecting to ESP32: {e}")
    exit(1)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Draw & Send to ESP32")
clock = pygame.time.Clock()

drawing = False
points = []
last_point = None


def map_value(value, in_min, in_max, out_min, out_max):
    return out_min + (float(value - in_min) * (out_max - out_min) / (in_max - in_min))


def calculate_distance(p1, p2):
    return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5


def send_to_esp32(pen, x, y):
    global last_command_time

    current_time = time.time()
    if current_time - last_command_time < RATE_LIMIT:
        return

    cmd = f"{pen} {x:.2f} {y:.2f}\n"
    print(f"Sending: {cmd.strip()}")
    ser.write(cmd.encode())

    time.sleep(0.02)
    last_command_time = time.time()


def lift_pen():
    x, y = pygame.mouse.get_pos()
    mapped_x = map_value(x, 0, WIDTH, X_MIN, X_MAX)
    mapped_y = map_value(y, HEIGHT, 0, Y_MIN, Y_MAX)
    send_to_esp32(PEN_UP, mapped_x, mapped_y)
    time.sleep(0.2)


screen.fill((0, 0, 0))
pygame.display.flip()

running = True
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    drawing = True
                    x, y = event.pos
                    last_point = (x, y)
                    points = [last_point]

                    mapped_x = map_value(x, 0, WIDTH, X_MIN, X_MAX)
                    mapped_y = map_value(y, HEIGHT, 0, Y_MIN, Y_MAX)
                    send_to_esp32(PEN_UP, mapped_x, mapped_y)
                    time.sleep(0.2)
                    send_to_esp32(PEN_DOWN, mapped_x, mapped_y)

            elif event.type == pygame.MOUSEMOTION:
                if drawing:
                    x, y = event.pos
                    current_point = (x, y)

                    if (
                        current_point != last_point
                        and calculate_distance(current_point, last_point)
                        >= MIN_DISTANCE
                    ):

                        points.append(current_point)

                        pygame.draw.line(
                            screen, (255, 255, 255), last_point, current_point, 2
                        )

                        mapped_x = map_value(x, 0, WIDTH, X_MIN, X_MAX)
                        mapped_y = map_value(y, HEIGHT, 0, Y_MIN, Y_MAX)
                        send_to_esp32(PEN_DOWN, mapped_x, mapped_y)

                        last_point = current_point

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    drawing = False
                    lift_pen()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_c]:
            screen.fill((0, 0, 0))
            lift_pen()
            time.sleep(0.5)

        pygame.display.flip()
        clock.tick(60)

except Exception as e:
    print(f"Error during execution: {e}")

finally:
    try:
        lift_pen()
    except:
        pass

    pygame.quit()
    ser.close()
    print("Disconnected")
