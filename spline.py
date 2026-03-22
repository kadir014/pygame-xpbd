from pygame import Vector2


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

def spline_segment(
        p0: Vector2,
        p1: Vector2,
        p2: Vector2,
        p3: Vector2,
        steps: int
        ) -> list[Vector2]:
    """ Generate points for a single Catmull-Rom spline segment. """

    points = []
    # +1 so lines are closed at nodes
    for step in range(steps + 1):
        t = step / steps

        points.append(catmull_rom(p0, p1, p2, p3, t))

    return points[:-1]

def spline_loop(
        points: list[Vector2],
        segment_steps: int = 25
        ) -> list[Vector2]:
    """
    Return the generated points over a Catmull-Rom spline for given points.

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
            points[i],
            points[(i + 1) % n],
            points[(i + 2) % n],
            points[(i + 3) % n],
            segment_steps
        )

    return looped_points