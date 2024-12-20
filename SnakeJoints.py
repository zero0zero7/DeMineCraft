
import math
import numpy as np
import time
from collections import deque

class SnakeJoints():
    def __init__(self, a, b, c, number_of_segments = 12, length_of_snake = 1500, target_speed = 50):
        self.number_of_segments = number_of_segments
        self.length_of_snake = length_of_snake
        self.length_per_segment = self.length_of_snake/self.number_of_segments
        self.target_speed = target_speed

        self.distance_moved_by_each_segment = np.array([0 - i*self.length_per_segment for i in range(self.number_of_segments + 1)])
        self.a = np.array([a for _ in range(self.number_of_segments + 1)])
        self.b = np.array([b for _ in range(self.number_of_segments + 1)])
        self.c = np.array([c for _ in range(self.number_of_segments + 1)])
        self.B = b * self.distance_moved_by_each_segment
        self.C = c * self.distance_moved_by_each_segment
        self.queues_for_parameters = [deque() for _ in range(self.number_of_segments + 1)]

    def set_parameters(self, a, b, c):
        for q in self.queues_for_parameters:
            q.append({
                "a": a, 
                "b": b, 
                "c": c, 
                "distance": self.distance_moved_by_each_segment[0]
            }) 

    def start_movement(self):
        self.start_time = time.time()
        self.prev_time = self.start_time

    def next_step(self):
        assert self.start_time
        cur_time = time.time()
        interval = cur_time - self.prev_time
        delta_distance = self.target_speed * (interval)
        self.distance_moved_by_each_segment += delta_distance
        self.check_update_params()
        self.increment_B_and_C(delta_distance)
        self.prev_time = cur_time
        return interval
    
    def check_update_params(self):
        for i, q in enumerate(self.queues_for_parameters):
            if q:
                if q[0]['distance'] <= self.distance_moved_by_each_segment[i]:
                    params = q.popleft()
                    self.a[i] = params['a']
                    self.b[i] = params['b']
                    self.c[i] = params['c']

    def increment_B_and_C(self, delta_distance):
        self.B += self.b * delta_distance
        self.C += self.c * delta_distance

    def get_phis(self):
        self.next_step()
        thetas = self.get_thetas()
        ones = self.get_ones()
        inverse = self.get_inverse(thetas)
        sc = self.get_scs(thetas)
        head = self.get_head_dxy()
        return ones @ (inverse @ (sc @ head))

    def get_thetas(self):
        return self.a[1:] * np.cos(self.B[1:]) + self.C[1:]
        

    def get_ones(self):
        ones = np.eye(self.number_of_segments)
        ones[:-1,1:] -= np.eye(self.number_of_segments-1)
        return ones[:-1, :] # since we only need joint 1 to join n-1
    
    def get_Cij(self, i , j, thetas):
        return math.cos(thetas[i] - thetas[j])

    def get_inverse(self, thetas):
        to_be_inversed = np.eye(self.number_of_segments) * .5
        lower_indices = np.tril_indices(self.number_of_segments, -1)
        for i, j in zip(lower_indices[0], lower_indices[1]):
            to_be_inversed[i,j] = self.get_Cij(i, j, thetas)
        to_be_inversed = to_be_inversed * self.length_per_segment
        inverse = np.linalg.inv(to_be_inversed)
        return inverse

    def get_scs(self, thetas):
        return np.array([-np.sin(thetas),np.cos(thetas)]).transpose()

    def get_head_dxy(self):
        theta0 = self.a[0] * np.cos(self.B[0]) + self.C[0]
        dx = math.cos(theta0) #* self.target_speed
        dy = math.sin(theta0) #* self.target_speed
        return np.array([[dx],[dy]])