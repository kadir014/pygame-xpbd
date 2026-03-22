from random import uniform, randint

import pygame
from pygame import Vector2
import miniprofiler

import xpbd
from spline import spline_loop, catmull_rom


WINDOW_SIZE = WINDOW_WIDTH, WINDOW_HEIGHT = 800, 720
MAX_FPS = 60

# Scale of the physics world compared to pygame screen
# 30 units = 1 pixel
ZOOM = 30.0


pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
clock = pygame.Clock()
is_running = True
font = pygame.Font("assets/Lilex-Regular.ttf", 14)
prof = miniprofiler.Profiler(MAX_FPS)


space = xpbd.XPBDSpace()
space.domain = pygame.FRect(0.1, 0.1, WINDOW_WIDTH / ZOOM - 0.2, WINDOW_HEIGHT / ZOOM - 0.2)

for y in range(0):
    p = xpbd.Particle(Vector2(10 + uniform(-0.5, 0.5), 2 + y * 4), mass=2.0)
    space.add_particle(p)

for y in range(4):
    for x in range(2):
        space.add_ngon_softbody(randint(3,5), Vector2(2 + x * 5, 2 + y * 5), 2, 0.002, 0.3)

space.gravity.y *= 2.0

#space.add_pressure_softbody(25, Vector2(10, 20), 3, 0.003, mass=0.35, rest_area=20)
#space.add_pressure_softbody(25, Vector2(17, 15), 3, 0.003, mass=0.45, rest_area=45)
#space.add_pressure_softbody(25, Vector2(5, 5), 3, 0.003, mass=0.35, rest_area=30)


mouse_p = xpbd.Particle(mass=0, collidable=False)
mouse_c: xpbd.DistanceConstraint | None = None

drawing_mode = 0

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

            elif event.key == pygame.K_F2:
                p = xpbd.Particle(mouse / ZOOM, mass=2.0)
                space.add_particle(p)

            elif event.key == pygame.K_SPACE:
                space.add_pressure_softbody(15, mouse / ZOOM, 3, 0.003, mass=0.45, rest_area=45)

            elif event.key == pygame.K_F1:
                drawing_mode = (drawing_mode + 1) % 2

        elif event.type == pygame.MOUSEBUTTONDOWN:
            for p in space.iter_particles():
                if p is mouse_p: continue

                if mouse.distance_to(p.position * ZOOM) < 30.0:
                    mouse_c = xpbd.DistanceConstraint(p, mouse_p, 0.0, 0.001)
                    space.add_constraint(mouse_c)
                    break

        elif event.type == pygame.MOUSEBUTTONUP:
            if mouse_c is not None:
                space.remove_constraint(mouse_c)
                mouse_c = None

    mouse_p.position = mouse / ZOOM

    with prof.profile("step"):
        space.step(1.0 / 60.0, iters=4, substeps=5)

    window.fill((25, 19, 48))

    if drawing_mode == 0:
        for p in space.iter_particles():
            pygame.draw.aacircle(
                window, (248, 247, 252), p.position * ZOOM, p.mass * ZOOM, 1
            )

        for c in space.iter_constraints():
            if c is mouse_c: continue
            edge = (c.a.position - c.b.position).normalize()
            l = edge.rotate(-90)
            r = edge.rotate(90)

            a = c.a.position * ZOOM
            b = c.b.position * ZOOM
            ra = c.a.mass * ZOOM
            rb = c.b.mass * ZOOM

            pygame.draw.aaline(window, (108, 240, 200), a + l * ra, b + l * rb, 1)
            pygame.draw.aaline(window, (108, 240, 200), a + r * ra, b + r * rb, 1)

    elif drawing_mode == 1:
        for p in space.iter_particles():
            ind = False
            for pb in space.iter_softbodies():
                if p in pb.particles:
                    ind = True
                    break
            if ind:
                continue
            pygame.draw.aacircle(
                window, (248, 247, 252), p.position * ZOOM, p.mass * ZOOM, 1
            )

        for pb in space.iter_softbodies():
            #points = [p.position * ZOOM for p in pb.particles]

            centroid = pygame.Vector2(0.0)
            for p in pb.particles:
                centroid += p.position
            centroid /= len(pb.particles)

            points = []
            for p in pb.particles:
                n = (p.position - centroid).normalize()

                point = p.position + n * p.mass * 0.75

                points.append(point * ZOOM)

            splined = spline_loop(catmull_rom, points)

            hue = id(pb) % 360
            color = pygame.Color.from_hsla(hue, 65.0, 17.0, 100.0)

            pygame.draw.polygon(window, color, splined, 0)
            color = pygame.Color.from_hsla(hue, 100.0, 65.0, 100.0)
            pygame.draw.polygon(window, color, splined, 1)

    lines = (
        f"Particles:   {len(space._particles)}",
        f"Constraints: {len(space._consts)}",
        f"Physics: avg={round(prof['step'].avg*1000.0, 2)}ms 99%={round(prof['step'].p99*1000.0, 2)}ms 1%={round(prof['step'].p01*1000.0, 2)}ms",
    )

    for i, line in enumerate(lines):
        window.blit(font.render(line, True, (255, 255, 255)), (5, 5 + i * 22))

    pygame.display.flip()
    pygame.display.set_caption(f"Pygame XPBD Experiments @ {round(clock.get_fps())} FPS")

pygame.quit()