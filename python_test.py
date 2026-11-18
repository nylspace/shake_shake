
import pygame
import serial
import math
import sys

# CHANGE THIS TO YOUR ARDUINO PORT
arduino = serial.Serial('COM4', 9600, timeout=0.05)

pygame.init()

# Screen settings
WIDTH = 800
HEIGHT = 600

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("MPU6050 Motion Controlled Ball")

clock = pygame.time.Clock()

# Ball settings
ball_x = WIDTH // 2
ball_y = HEIGHT // 2
ball_radius = 20

# Movement settings
angle_x = 0
angle_y = 0

smooth_x = 0
smooth_y = 0

font = pygame.font.SysFont("Arial", 22)

running = True

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read Arduino data
    
    # Read the latest complete sensor reading
    if arduino.in_waiting:
        try:
            latest = None

            # Process available complete lines
            while arduino.in_waiting:
                line = arduino.readline().decode(
                    errors="ignore"
                ).strip()

                if line.count(",") == 1:
                    latest = line

            if latest:
                values = latest.split(",")

                angle_x = float(values[0])
                angle_y = float(values[1])

        except ValueError:
            pass


    # Convert tilt into screen movement
    target_x = (angle_x / 30) * (WIDTH / 2 - ball_radius)
    target_y = (angle_y / 30) * (HEIGHT / 2 - ball_radius)

    # Smooth movement
    smooth_x += (target_x - smooth_x) * 0.15
    smooth_y += (target_y - smooth_y) * 0.15

    # Calculate ball position
    ball_x = WIDTH // 2 + smooth_x
    ball_y = HEIGHT // 2 + smooth_y

    # Keep ball inside screen
    ball_x = max(ball_radius, min(WIDTH - ball_radius, ball_x))
    ball_y = max(ball_radius, min(HEIGHT - ball_radius, ball_y))

    # Draw screen
    screen.fill((20, 25, 40))

    # Center reference
    pygame.draw.circle(
        screen, (70, 80, 100),
        (WIDTH // 2, HEIGHT // 2), 5
    )

    # Draw ball
    pygame.draw.circle(
        screen, (50, 150, 255),
        (int(ball_x), int(ball_y)), ball_radius
    )

    # Display sensor readings
    text = font.render(
        f"Tilt X: {angle_x:.1f} | Tilt Y: {angle_y:.1f}",
        True, (255, 255, 255)
    )

    screen.blit(text, (20, 20))

    pygame.display.flip()
    clock.tick(60)

# Cleanup
arduino.close()
pygame.quit()
sys.exit()
