from typing import Iterator
from dataclasses import dataclass, field
from math import pi

import pygame
from pygame import Vector2


EPSILON = 0.00001

def degenerate_vector() -> Vector2:
    # Used for complete overlaps and such
    return Vector2(1.0, 0.0)


@dataclass
class Particle:
    """
    A point with mass in space.

    Pass 0.0 to mass to make this particle static.

    NOTE: Radius is equal to particle's mass.

    Attributes
    ----------
    position
        Position of the particle
    velocity
        Linear velocity of the particle
    mass
        Mass (and radius) of the particle
    collidable
        Whether this particle collides with other particles or not
    """

    position: Vector2 = field(default_factory=lambda: Vector2(0.0))
    velocity: Vector2 = field(default_factory=lambda: Vector2(0.0))
    mass: float = 1.0
    collidable: bool = True

    def __post_init__(self) -> None:
        self.inv_mass = 1.0 / self.mass if self.mass > 0.0 else 0.0
        self.position_prev = Vector2(0.0)
        self.force = Vector2(0.0)

    @property
    def is_static(self) -> bool:
        """ Is this particle static (infinite mass) or not? """
        return self.inv_mass == 0.0


@dataclass
class DistanceConstraint:
    """
    Distance constraint between two particles.

    Attributes
    ----------
    a
        First particle
    b
        Second particle
    rest
        Rest length of the distance constraint
    alpha
        Compliance of the constraint

        0.0 means the constraint is rigid
    """
    a: Particle
    b: Particle
    rest: float = 10.0
    alpha: float = 0.01

    def __post_init__(self) -> None:
        self.lambda_ = 0.0

    def solve(self, dt: float) -> None:
        delta = self.a.position - self.b.position
        dist = delta.length()

        if dist <= EPSILON:
            return
        
        w1 = self.a.inv_mass
        w2 = self.b.inv_mass

        n = delta / dist

        # Constraint error
        C = dist - self.rest

        # Compliance term (inverse stiffness)
        alpha_term = self.alpha / (dt * dt)

        # Incremental lambda
        d_lambda = (-C - alpha_term * self.lambda_) / (w1 + w2 + alpha_term)

        correction = d_lambda * n

        self.a.position += w1 * correction
        self.b.position -= w2 * correction

        self.lambda_ += d_lambda


@dataclass
class _PressureSoftBody:
    """ Proxy for a pressure-based soft-body. """
    particles: list[Particle]
    normals: list[pygame.Vector2]
    area: float
    rest_area: float
    k: float


def solve_particle_x_particle(a: Particle, b: Particle, dt: float) -> None:
    """
    Solve collision between two particles, modeled as a rigid constraint.
    """

    delta = a.position - b.position
    dist = delta.length()

    min_dist = a.mass + b.mass
    C = dist - min_dist

    if C >= 0:
        return

    if dist == 0:
        n = degenerate_vector()
    else:
        n = delta / dist

    w1 = a.inv_mass
    w2 = b.inv_mass

    alpha = 0.0 # hard constraint
    alpha_term = alpha / (dt * dt)

    # Lambda is solved one-shot, not incrementally
    # This would increase jitteriness and such?
    # Maybe implemenmt contact caching to store lambda
    lambda_ = (-C) / (w1 + w2 + alpha_term)

    correction = lambda_ * n

    a.position += w1 * correction
    b.position -= w2 * correction


def solve_particle_x_edge(a: Particle, b1: Particle, b2: Particle, dt: float) -> None:
    thickness = max(b1.mass, b2.mass)
    ... # TODO


# TODO: GENERALIZE CONSTRAINTS INSTEAD OF 30 DIFFERENT FUNCTIONS

def _min_bound(min_: float, p: Particle, axis: int, alpha: float, dt: float) -> None:
    C = min_ - (p.position[axis] - p.mass)

    if C > 0:
        w = p.inv_mass

        alpha_term = alpha / (dt * dt)
        lambda_ = -C / (w + alpha_term)

        p.position[axis] -= w * lambda_

def _max_bound(max_: float, p: Particle, axis: int, alpha: float, dt: float) -> None:
    C = p.position[axis] + p.mass - max_

    if C > 0:
        w = p.inv_mass

        alpha_term = alpha / (dt * dt)
        lambda_ = -C / (w + alpha_term)

        p.position[axis] += w * lambda_


class XPBDSpace:
    """
    Simulation space.

    Attributes
    ----------
    gravity
        Uniform gravity for all space
    domain
        A pygame rect determining simulation bounds
    """

    def __init__(self) -> None:
        self._consts: list[DistanceConstraint] = []
        self._particles: list[Particle] = []
        self._pressures: list[_PressureSoftBody] = []

        self.gravity = Vector2(0.0, 9.81)
        self.domain = pygame.FRect(0.0, 0.0, 50.0, 50.0)

    def iter_particles(self) -> Iterator[Particle]:
        """ Iterate over a shallow copy of particles. """
        for p in self._particles.copy():
            yield p

    def iter_constraints(self) -> Iterator[DistanceConstraint]:
        """ Iterate over a shallow copy of constraints. """
        for c in self._consts.copy():
            yield c

    def add_particle(self, p: Particle) -> None:
        """ Add a new particle. """
        self._particles.append(p)

    def add_constraint(self, c: DistanceConstraint) -> None:
        """ Add a new constraint. """
        self._consts.append(c)

    def remove_particle(self, p: Particle) -> None:
        """ Remove a particle. """
        if p in self._particles:
            self._particles.remove(p)

    def remove_constraint(self, c: DistanceConstraint) -> None:
        """ Remove a constraint. """
        if c in self._consts:
            self._consts.remove(c)

    def add_ngon_softbody(self,
            n: int,
            center: Vector2,
            radius: float,
            alpha: float,
            mass: float = 1.0,
            diagonals: bool = True
            ) -> list[Particle]:
        """
        Add a simple regular-polygon soft-body with N vertices.

        Parameters
        ----------
        n
            Number of vertices
        center
            Center of the polygon in physics space
        radius
            Radius of the polygon in physics space
        alpha
            Compliance of constraints
        mass
            Mass of vertex particles
        diagonals
            Whether to add diagonal structural constraints
        """
        local_particles: list[Particle] = []

        # Vertices
        arm = Vector2(radius, 0.0)
        for i in range(n):
            angle = (i / n) * 2 * pi

            x = center + arm.rotate_rad(angle)

            p = Particle(x, mass=mass)
            self.add_particle(p)
            local_particles.append(p)

        # Edges
        for i in range(n):
            p1 = local_particles[i]
            p2 = local_particles[(i + 1) % n]

            rest_length = (p1.position - p2.position).length()

            self.add_constraint(DistanceConstraint(p1, p2, rest_length, alpha))

        # Diagonal supports
        if diagonals:
            for i in range(n):
                for j in range(i + 2, n):
                    # Skip immediate neighbors
                    if j == (i - 1) % n:
                        continue

                    p1 = local_particles[i]
                    p2 = local_particles[j]

                    rest_length = (p1.position - p2.position).length()

                    # Inside edges should be weaker than outline faces
                    self.add_constraint(DistanceConstraint(p1, p2, rest_length, alpha * 5.0))

        return local_particles
    
    def add_pressure_softbody(self,
            n: int,
            center: Vector2,
            radius: float,
            alpha: float,
            rest_area: float = 30.0,
            pressure_value: float = 3.0,
            mass: float = 1.0,
            ) -> _PressureSoftBody:
        """
        Add a pressure-force-based soft-body.
        
        Parameters
        ----------
        n
            Number of vertices
        center
            Center of the body in physics space
        radius
            Radius of the body in physics space
        alpha
            Compliance of edge constraints
        rest_area
            Rest area for pressure calculation
        pressure_value
            Pressure value
        mass
            Mass of vertex particles
        """
        
        # Pressure force formulation comes from
        # https://www.researchgate.net/publication/228574502_How_to_implement_a_pressure_soft_body_model
        
        particles = self.add_ngon_softbody(
            n, center, radius, alpha, mass, diagonals=False
        )

        b = _PressureSoftBody(particles, [], 0.0, rest_area, pressure_value)

        self._pressures.append(b)

        return b

    def step(self, dt: float, iters: int = 1, substeps: int = 1) -> None:
        """
        Advance simulation.
        
        Parameters
        ----------
        dt
            Time step in seconds
        iters
            Constraint solving iteration count
        substeps
            Substep count 
        """
        # See Matthias Müller's video on XPBD
        # https://www.youtube.com/watch?v=jrociOAYqxA

        sdt = dt / substeps

        for _ in range(substeps):
            for c in self._consts:
                c.lambda_ = 0.0

            # Calculate valumes
            for pb in self._pressures:
                pb.area = 0.0
                n = len(pb.particles)

                # shoelace
                for i in range(n):
                    a = pb.particles[i]
                    b = pb.particles[(i + 1) % n]
                    xa = a.position
                    xb = b.position

                    delta = xa - xb
                    pb.normals.append(Vector2(-delta.y, delta.x).normalize())
                    
                    pb.area += xa.x * xb.y - xb.x * xa.y

                pb.area *= 0.5

            # External forces
            for p in self._particles:
                if p.is_static: continue

                p.force += self.gravity * p.mass

            # Pressure forces
            for pb in self._pressures:
                n = len(pb.particles)
                for i in range(n):
                    a = pb.particles[i]
                    b = pb.particles[(i + 1) % n]

                    delta = a.position - b.position
                    dist = delta.length()

                    pressure = dist * pb.k * ((pb.rest_area - pb.area) / pb.area)

                    a.force += pressure * pb.normals[i]
                    b.force += pressure * pb.normals[i]

            # PBD integration
            for p in self._particles:
                if p.is_static: continue

                p.velocity += p.force * p.inv_mass * sdt

                # TODO: better linear damping
                p.velocity *= 0.995

                p.position_prev = p.position.copy()

                p.position = p.position + p.velocity * sdt

            # Solve constraints
            for _ in range(iters):
                # Boundaries
                for p in self._particles:
                    if p.is_static: continue

                    alpha = 0.0 # affects restitution, too high makes it tunnel
                    _min_bound(self.domain.left, p, 0, alpha, sdt)
                    _min_bound(self.domain.top, p, 1, alpha, sdt)
                    _max_bound(self.domain.right, p, 0, alpha, sdt)
                    _max_bound(self.domain.bottom, p, 1, alpha, sdt)

                # Particle x Particle
                for a in self._particles:
                    for b in self._particles:
                        if id(a) >= id(b): continue
                        if not a.collidable or not b.collidable: continue

                        solve_particle_x_particle(a, b, sdt)

                # Distance constraints
                for c in self._consts:
                    c.solve(sdt)
            
            # Update velocities with constrained positions
            for p in self._particles:
                if p.is_static: continue

                p.velocity = (p.position - p.position_prev) / sdt
                p.force = Vector2(0.0)