#!/usr/bin/env python

import os
import pygame
import numpy as np
from math import sin, radinas, degrees, copysign

class Car:
    # class constructor funtion
    def __init__(self, x, y, yaw=0.0, max_steering=30, max_velocity = 1000, max_acceleration=1000.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.max_steering = max_steering
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        
        # deceleration when push the space key (brake)
        self.brake_deceleration = 300
        # deceleration without pushing keyboard
        self.free_deceleration = 50
        self.linear_acceleration = 10.0
        
        self.linear_velocity = 0.0
        self.steering_angle = 0.0
        self.wheel_base = 84

        self.car_img_x = 0
        self.car_img_y = 0
        # left-top(-64, -32), left-bot(-64, 32), right-bot(64, -32), right-top(64, 32)
        self.car_x_ori = [-64, -64, 64, 64]
        self.car_y_ori = [-32, 32, -32, 32]

    def update(self, dt):
        # calculate the current linear velocity (vel = acc * dt)
        self.linear_velocity += (self.linear_acceleration * dt)
        self.linear_velocity = min(max(-self.max_velocity, self.linear_velocity), self.max_velocity)

        # calculate the current angular velocity (ang_vel = vel/R = (vel/L)*tan(theta)) <= ackerman model
        self.angular_velocity = 0.0

        if self.steering_angle != 0.0:
            self.angular_velocity = (self.linear_velocity/self.wheel_base) * np.tan(np.radians(self.steering_angle))

        # calculate the yaw angle
        self.yaw += (np.degrees(self.angular_velocity) * dt)
        # calculate the distance
        self.spatium = self.linear_velocity * dt

        # car position (x, y) in local coordinate
        self.x += (self.spatium * np.cos(np.radians(-self.yaw)))
        self.y += (self.spatium * np.sin(np.radians(-self.yaw)))
        # car position (x, y) in global coordinate
        car_x = [0,0,0,0]
        car_y = [0,0,0,0]

        # rotational transformation matrix
        for i in range(4):
            car_x[i] = self.car_x_ori[i] * np.cos(-np.radians(self.yaw)) - self.car_y_ori[i] * np.sin(-np.radians(self.yaw)) + self.x

            car_y[i] = self.car_y_ori[i] * np.sin(-np.radians(self.yaw)) + self.car_y_ori[i] * np.cos(-np.radians(self.yaw)) + self.y

        # simplify car position
        self.car_img_x = int(round(min(car_x)))
        self.car_img_x = int(round(min(car_x)))

# initialize pygame
pygame.init()
# pygame window title
pygame.display.set_caption("Pygame Car Simulator #1")
# pygame sindow size
width, height = 1280, 720
screen = pygame.display.set_mode((width, height))
# clock object for while loop
clock = pygame.time.Clock()
# create car object
current_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(current_dir, "car.png")
car_image = pygame.image.load(image_path)

# create car class object
car = Car(100,100)
exit_flags = False

# do while loop until exit_flags become True
while not exit_flags:
    # fps = 60
    clock.tick(60)
    # unit time
    dt = clock.get_time() / 1000

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exit_flags = True

    # keyboard -> "pressed" variable
    pressed = pygame.key.get_pressed()

    # if I push the upward keyboard
    if pressed[pygame.K_UP]:
        # if linear velocity is negative
        if car.linear_velocity < 0:
            car.linear_acceleration = car.brake_deceleration

        # if linear velocity is positive
        else:
            car.linear_acceleration += 10 * dt
    
    # if I push the downward keyboard
    elif pressed[pygame.K_DOWN]:
        # if linear velocity is positive
        if car.linear_velocity > 0:
            car.linear_acceleration = -car.brake_deceleration

        # if linear velocity is negative
        else:
            car.linear_acceleration -= 10 * dt

    # if I push the space keyboard
    elif pressed[pygame.K_SPACE]:
        # if linear velocity is bigger than (brake deceleration * dt)
        if abs(car.linear_velocity) > dt * car.brake_deceleration:
            # copysign(x,y): y sign = x sign
            car.linear_acceleration = -copysign(car.brake_deceleration, car.linear_velocity)
        # if linear velocity is smaller than (brake deceleration * dt)
        else:
            car.linear_acceleration = -car.linear_velocity / dt

    # if no keyboard is pressed
    else:
        if abs(car.linear_velocity) > dt * car.free_deceleration:
            car.linear_acceleration = -copysign(car.free_deceleration, car.linear_velocity)
        else:
            if dt != 0:
                car.linear_acceleration = -car.linear_velocity / dt

    # -1000 < linear_acceleration < 1000
    car.linear_acceleration = max(-car.max_acceleration, min(car.linear_acceleration, car.max_acceleration))

    # if I push the rightward keyboard
    if pressed[pygame.K-RIGHT]:
        car.steering_angle -= 30 * dt

    # if I push the leftward keyboard
    elif pressed[pygame.K-LEFT]:
        car.steering_angle += 30 * dt

    # if no side keyboard is pressed
    else:
        car.steering_angle = 0

    # -30 < steering_angle < 30
    car.steering_angle = max(-car.max_steering, min(car.steering_angle, car.max_steering))

    # update the state of car over time
    car.update(dt)

    # screen background color is black
    screen.fill((0,0,0))
    # rotate car image (car.png)
    rotated = pygame.transform.rotate(car_image, car.yaw)
    # put the rotated car image at the caculated postion
    screen.blit(rotated, [car.car_img_x, car.car_img_y])
    # renew the screen
    pygame.display.flip()

# quit the program
pygame.quit()