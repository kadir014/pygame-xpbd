from random import uniform

import pygame
from pygame import Vector2

import xpbd


WINDOW_SIZE = WINDOW_WIDTH, WINDOW_HEIGHT = 1280, 720
MAX_FPS = 60

# Scale of the physics world compared to pygame screen
# 30 units = 1 pixel
ZOOM = 30.0


pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
clock = pygame.Clock()
is_running = True


space = xpbd.XPBDSpace()
space.domain = pygame.FRect(0, 0, WINDOW_WIDTH / ZOOM, WINDOW_HEIGHT / ZOOM)

for y in range(5):
    p = xpbd.Particle(Vector2(10 + uniform(-0.5, 0.5), 2 + y * 4), mass=2.0)
    space.add_particle(p)

space.add_ngon_softbody(6, Vector2(20, 20), 2, 0.003, 0.4)

space.add_pressure_softbody(25, Vector2(35, 20), 3, 0.003, mass=0.35, pressure_value=30)
space.add_pressure_softbody(25, Vector2(30, 12), 3, 0.003, mass=0.35, pressure_value=30)
space.add_pressure_softbody(25, Vector2(30, 5), 3, 0.003, mass=0.35, pressure_value=30)
space.add_pressure_softbody(25, Vector2(5, 5), 3, 0.003, mass=0.3, pressure_value=30)


while is_running:
    clock.tick(MAX_FPS)

    events = pygame.event.get()
    mouse = Vector2(*pygame.mouse.get_pos())
    for event in events:
        if event.type == pygame.QUIT:
            is_running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                is_running = False

    space.step(1.0 / 60.0, iters=4, substeps=1)

    window.fill((25, 19, 48))

    for p in space.iter_particles():
        pygame.draw.aacircle(
            window, (248, 247, 252), p.position * ZOOM, p.mass * ZOOM, 1
        )

    for c in space.iter_constraints():
        pygame.draw.aaline(
            window, (108, 240, 200), c.a.position * ZOOM, c.b.position * ZOOM, 1
        )

    pygame.display.flip()
    pygame.display.set_caption(f"Pygame XPBD Experiments @ {round(clock.get_fps())} FPS")

pygame.quit()