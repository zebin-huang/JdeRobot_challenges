import random
import numpy as np
import time


class Brownian_Bot:
    def __init__(self, screen_width: int, screen_height: int, radius: int = 10, color: str = "blue") -> None:
        """constructor for Brownian_Bot class

        Args:
            screen_width (int): screen width
            screen_height (int): screen height
            radius (int): radius of robot
            color (string): the color of robot and its direction line
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.position = np.array([screen_width/2, screen_height/2], dtype=float)
        self.velocity = np.array([3, 0], dtype=float)
        self.velocity_scalar = np.sqrt(np.sum(self.velocity**2))
        self.radius = radius
        self.color = color
        self.last_collision = 0

    def check_collision(self):
        # when the bot hits the edge of the screen, randomly change direction
        if time.time() - self.last_collision < 0.06:
            # wait for 0.06 seconds to avoid getting bounced back and forth
            return
        if self.position[0] - self.radius <= 0 or self.position[0] + self.radius >= self.screen_width:
            # check if the bot is hitting the left or right edge of the screen
            new_velocity = self.random_rotation()
            new_velocity[0] = -new_velocity[0]
            new_velocity[1] = random.choice([-1, 1]) * new_velocity[1]  # randomly revert the direction of y
            self.velocity = new_velocity
            self.last_collision = time.time()
        if self.position[1] - self.radius <= 0 or self.position[1] + self.radius >= self.screen_height:
            # check if the bot is hitting the top or bottom edge of the screen
            new_velocity = self.random_rotation()
            new_velocity[1] = -new_velocity[1]
            new_velocity[0] = random.choice([-1, 1]) * new_velocity[0]  # same as above
            self.velocity = new_velocity
            self.last_collision = time.time()
            # print(self.velocity)

    def random_rotation(self):
        dir_x = np.sign(self.velocity[0]) or 1
        dir_y = np.sign(self.velocity[1]) or 1
        # calculate random velocity in x direction within a specified range
        vel_x = dir_x * random.uniform(self.velocity_scalar/3, 2 * self.velocity_scalar/3)
        # calculate velocity in y direction to keep the overall velocity magnitude constant
        vel_y = dir_y * np.sqrt(self.velocity_scalar**2 - vel_x**2)
        new_velocity = np.array([vel_x, vel_y])
        # current timestamp and the new velocity
        print(time.time(), new_velocity)
        return new_velocity

    def update(self):
        # when update, check if there is a collide and then update the position by velocity
        self.check_collision()
        self.position += self.velocity

    def render(self):
        # for render bot posision
        return (int(self.position[0]), int(self.position[1]))
