from typing import Callable

from pygame import Vector2


SplineFunc = Callable[[Vector2, Vector2, Vector2, Vector2, float], Vector2]

def catmull_rom(
        p0: Vector2,
        p1: Vector2,
        p2: Vector2,
        p3: Vector2,
        t: float
        ) -> Vector2:
    t2 = t * t
    t3 = t2 * t

    x = 0.5 * ((2.0 * p1.x) +
              (-p0.x + p2.x) * t +
              (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
              (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3)

    y = 0.5 * ((2 * p1.y) +
              (-p0.y + p2.y) * t +
              (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
              (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3)

    return Vector2(x, y)

def cubic_bezier_decasteljau(
        p0: Vector2,
        p1: Vector2,
        p2: Vector2,
        p3: Vector2,
        t: float
        ) -> Vector2:
    v0 = Vector2(p0)
    v1 = Vector2(p1)
    v2 = Vector2(p2)
    v3 = Vector2(p3)

    a = v0.lerp(v1, t)
    b = v1.lerp(v2, t)
    c = v2.lerp(v3, t)
    d = a.lerp(b, t)
    e = b.lerp(c, t)
    p = d.lerp(e, t)

    return p

def spline_segment(
        func: SplineFunc,
        p0: Vector2,
        p1: Vector2,
        p2: Vector2,
        p3: Vector2,
        steps: int
        ) -> list[Vector2]:
    """ Generate points for a single spline segment. """

    points = []
    # +1 so lines are closed at nodes
    for step in range(steps + 1):
        t = step / steps

        points.append(func(p0, p1, p2, p3, t))

    return points[:-1]

def spline_loop(
        func: SplineFunc,
        points: list[Vector2],
        segment_steps: int = 25,
        ) -> list[Vector2]:
    """
    Return the generated points over a spline for given control points.

    Parameters
    ----------
    points
        Control points to generate the spline over
    segment_steps
        Number of steps in a segment (between two control points)
    """
    
    looped_points = []
    n = len(points)
    
    for i in range(n):
        looped_points += spline_segment(
            func,
            points[i],
            points[(i + 1) % n],
            points[(i + 2) % n],
            points[(i + 3) % n],
            segment_steps
        )

    return looped_points