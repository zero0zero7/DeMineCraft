
import math
import numpy as np
import time
from collections import deque

class SnakeJoints():
    def __init__(self, a, b, c, number_of_segments = 12, length_of_snake = 1500, target_speed = 50):
        '''
        params: 
        a int: initial undulation
        b int: initial period
        c int: initial angular velocity
        '''
        self.number_of_segments = number_of_segments
        self.length_of_snake = length_of_snake
        self.length_per_segment = self.length_of_snake/self.number_of_segments
        self.target_speed = target_speed

        # this part is not from the paper. the paper inputs a,b,c as a function of distance travelled by the joint. in our case, we want a,b,c to be modified on the go. might have a more elegant solution 
        # disp of each segment; ref point = starting pos of head (node 0th)
        self.distance_moved_by_each_segment = np.array([0 - i*self.length_per_segment for i in range(self.number_of_segments + 1)])
        self.a = np.array([a for _ in range(self.number_of_segments + 1)])
        self.b = np.array([b for _ in range(self.number_of_segments + 1)])
        self.c = np.array([c for _ in range(self.number_of_segments + 1)])
        self.B = b * self.distance_moved_by_each_segment
        self.C = c * self.distance_moved_by_each_segment
        self.queues_for_parameters = [deque() for _ in range(self.number_of_segments + 1)]

    def start_movement(self):
        self.start_time = time.time()
        self.prev_time = self.start_time

    def get_phis(self):
        # equation (22 and 23)
        self.next_step()
        thetas = self.get_thetas()
        ones = self.get_ones()
        inverse = self.get_inverse(thetas)
        sc = self.get_scs(thetas)
        head = self.get_head_dxy()
        return ones @ (inverse @ (sc @ head)) #@ =numpy matrix multiply

    def set_parameters(self, a, b, c):
        for q in self.queues_for_parameters:
            q.append({
                "a": a, 
                "b": b, 
                "c": c, 
                "distance": self.distance_moved_by_each_segment[0]
            }) 

    def set_target_speed(self, target_speed):
        self.target_speed = target_speed

    ## util fns below
    def next_step(self):
        assert self.start_time
        cur_time = time.time()
        interval = cur_time - self.prev_time
        delta_distance = self.target_speed * (interval) #dist to be travelled (?)
        self.distance_moved_by_each_segment += delta_distance
        self.check_update_params()
        self.increment_B_and_C(delta_distance)
        self.prev_time = cur_time
        return interval
    
    def check_update_params(self):
        #i=index ; q=deque
        for i, q in enumerate(self.queues_for_parameters):
            if q: #q not empty
                #distance moved by 0th segment <= dist by this particular segment
                if q[0]['distance'] <= self.distance_moved_by_each_segment[i]:
                    params = q.popleft() #dequeue of 0th segment
                    # reset this particular segment's params to that of the 0th segment
                    self.a[i] = params['a']
                    self.b[i] = params['b']
                    self.c[i] = params['c']

    def increment_B_and_C(self, delta_distance):
        self.B += self.b * delta_distance
        self.C += self.c * delta_distance

    def get_thetas(self):
        # equation (12) 
        return self.a[1:] * np.cos(self.B[1:]) + self.C[1:]
        
    def get_ones(self):
        # first matrix of equation (23)
        ones = np.eye(self.number_of_segments)
        ones[:-1,1:] -= np.eye(self.number_of_segments-1)
        return ones[:-1, :] # since we only need joint 1 to join n-1
    
    def get_Cij(self, i , j, thetas):
        # equation (20)
        return math.cos(thetas[i] - thetas[j])

    def get_inverse(self, thetas):
        # second matrix of equation (23)
        to_be_inversed = np.eye(self.number_of_segments) * .5
        lower_indices = np.tril_indices(self.number_of_segments, -1)
        for i, j in zip(lower_indices[0], lower_indices[1]):
            to_be_inversed[i,j] = self.get_Cij(i, j, thetas)
        to_be_inversed = to_be_inversed * self.length_per_segment
        inverse = np.linalg.inv(to_be_inversed)
        return inverse

    def get_scs(self, thetas):
        # third matrix of equation (23)
        return np.array([-np.sin(thetas),np.cos(thetas)]).transpose()

    def get_head_dxy(self):
        # input of equation (22)
        theta0 = self.a[0] * np.cos(self.B[0]) + self.C[0]
        dx = math.cos(theta0) #* self.target_speed
        dy = math.sin(theta0) #* self.target_speed
        return np.array([[dx],[dy]])

###TESTING###
a = math.pi/3
b = math.pi/2
c = 0
target_speed = 50
length_of_snake = 1500 # in mm
number_of_segments = 12

snake_joints = SnakeJoints(a, b, c, number_of_segments, length_of_snake, target_speed)
# print(snake_joints.distance_moved_by_each_segment)
# print(snake_joints.B, snake_joints.C)
# print(snake_joints.queues_for_parameters)
snake_joints.start_movement()
print(snake_joints.get_phis())