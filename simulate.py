import random
from Box2D import *
from Box2D.examples.framework import Framework



class Simulation(Framework):
    def __init__(self):
        super(Simulation, self).__init__()
        # Disable default gravity
        self.world.gravity = (0.0, 0.0)
        # Newtonian Gravity constant
        self.G = 6.6743e-11 
        # Screen parameters
        self.X_min = -30
        self.X_max = 30
        self.Y_min = 0
        self.Y_max = 40
        
        # all bodies move to each other to minimize simulations with fly away
        # point should be close to screen center
        X_dest_tmp = (self.X_min + self.X_max) / 2
        Y_dest_tmp = (self.Y_min + self.Y_max) / 4
        X_dest = random.randint(0, 5) + X_dest_tmp
        Y_dest = random.randint(0, 5) + Y_dest_tmp
        print(X_dest,Y_dest)

        for i in range(0,3):
            radius = random.uniform(0.1, 3)
            density = random.randint(1.0e+11,1.0e+12)
            pos_x = random.randint(self.X_min ,self.X_max)
            pos_y = random.randint(self.Y_min ,self.Y_max)
            linear_velocity_x = (X_dest - pos_x + random.randint(-2,2)) / 4
            linear_velocity_y = (Y_dest - pos_y + random.randint(-2,2)) / 4
            body = b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, friction=10, restitution=0.2)
            self.world.CreateBody(type=b2_dynamicBody, position=b2Vec2(pos_x, pos_y), fixtures=body, linearVelocity=(linear_velocity_x, linear_velocity_y))            


    def Step(self, settings):
        super(Simulation, self).Step(settings)
        bodies_on_screen = 0

        # Simulate the Newton's gravity
        for bi in self.world.bodies:
            if bi.position.x > self.X_min and bi.position.x < self.X_max:
                if bi.position.y > self.Y_min and bi.position.y < self.Y_max:
                    bodies_on_screen += 1
            for bk in self.world.bodies:
                if bi == bk:
                    continue

                pi, pk = bi.worldCenter, bk.worldCenter
                mi, mk = bi.mass, bk.mass
                delta = pk - pi
                r = delta.length
                if abs(r) < 1.0:
                    r = 1.0

                force = self.G * mi * mk / (r * r)
                delta.Normalize()
                bi.ApplyForce(force * delta, pi, True)
        
        if bodies_on_screen == 0:
        	#print('All bodies left screen') 
        	pass

if __name__ == "__main__":
    while True:
            Simulation().run()
