#!/usr/bin/env python
# vim:fileencoding=utf-8

import rospy
import math
import operator
from numpy.random import *
from scipy.stats import norm
from geometry_msgs.msg import Twist
from std_msgs.msg import *
from std_srvs.srv import *
from turtlesim.srv import *
from turtlesim.msg import *

class Particle:
    def __init__(self, pose, weight, name):
        self.x = pose
        self.x_hat = pose
        self.weight = weight
        self.name = name

    def call_position(self):
        telep_abs_client = rospy.ServiceProxy('/'+self.name + '/teleport_absolute', TeleportAbsolute)
        telep_req = TeleportAbsoluteRequest(self.x.x, self.x.y, self.x.theta)
        telep_res = telep_abs_client(telep_req)
        return telep_res

    def set_x_hat(self, x, y, theta):
        self.x_hat = Pose(x,y,theta,0,0)

    def set_x(self):
        self.x = self.x_hat
        self.call_position()


class TurtleMCL:
    def __init__(self):
        rospy.init_node('turtlesim_mcl')
        rospy.Subscriber('/turtle1/gps', Pose, self.measurement_update,  queue_size=10)
        rospy.Subscriber('/turtle1/cmd_vel', Twist, self.odom_update, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.init_pos_cb)

        self.is_init = False
        self.num = 10
        self.init_weight = 1.0 / self.num
        self.particles=[]
        self.init_cov_trans = 0.5
        self.init_cov_rot = math.pi/12 * math.pi/12

        self.is_init_pos = False
        self.init_theta = 0.0
        self.init_x = 0.0
        self.init_y = 0.0

        self.is_first_calc_odom = True
        self.on_update = False

        self.dt = 1.0

    def init_pos_cb(self, data):
        if self.is_init_pos:
            return
        self.init_theta = data.theta
        self.init_x = data.x
        self.init_y = data.y
        self.is_init_pos = True

    def init_mcl(self):
        if self.is_init or not self.is_init_pos:
            return False

        rospy.loginfo('init')
        for i, var in enumerate(range(0, self.num)):
            x_init = Pose(
                normal(self.init_x, self.init_cov_trans),
                normal(self.init_y, self.init_cov_trans),
                normal(self.init_theta, self.init_cov_rot),
                0,0
            )
            self.particles.append(
                Particle(x_init, self.init_weight, 'particle_'+str(i))
            )
            spawn_req = SpawnRequest(x_init.x, x_init.y, x_init.theta, 'particle_'+str(i))
            spawn_client = rospy.ServiceProxy('/spawn', Spawn)
            spawn_resp = spawn_client(spawn_req)

        self.is_init = True
        return True

    def calc_odom_diff(self, velocity):

        theta = math.fmod(velocity.angular.z * self.dt, 2*math.pi)
        x = math.sin(theta + math.pi/2.0) * velocity.linear.x * self.dt
        y = math.cos(theta + math.pi/2.0) * velocity.linear.x * self.dt
        rospy.loginfo(str(x) + ',' + str(theta))
        return theta, x, y

    def odom_update(self, data):
        if not self.is_init:
            return

        rospy.loginfo(data)
        diff_theta, diff_x, diff_y = self.calc_odom_diff(data)
        for particle in self.particles:
            particle.set_x_hat(
                particle.x.x +
                math.sin(particle.x.theta + math.pi/2.0) *
                diff_x *
                normal(1.0, 0.01),
                particle.x.y +
                -math.cos(particle.x.theta + math.pi/2.0) *
                diff_x *
                normal(1.0, 0.01),
                particle.x.theta + diff_theta * normal(1.0, 0.05)
            )
        rospy.loginfo('odometry updated')

    def resampling(self):
        self.particles.sort(key=operator.attrgetter('weight'))
        new_particles = self.particles
        i = 1
        j = 0
        while (len(self.particles) - i) > 0:
            if abs(self.particles[i].weight) < 0.0001:
                resample_size = 0
            else:
                resample_size = int(1 / self.particles[i].weight)
            if len(self.particles) - i < resample_size:
                resample_size = i
            for j in range(i, resample_size):
                self.particles[j] = self.particles[i]
            if resample_size < 1:
                i = i+1
            else:
                i = i+resample_size
        weight_list = []
        for particle in self.particles:
            weight_list.append(particle.weight)
        # rospy.loginfo(str(weight_list))
        rospy.loginfo('resampled')
        return

    def measurement_update(self, data):
        if not self.is_init:
            return

        weight_sum = float()
        for particle in self.particles:
            r = math.sqrt(
                pow(data.x - particle.x_hat.x, 2) +
                pow(data.y - particle.x_hat.y, 2)
            )
            phai = abs(data.theta - particle.x_hat.theta)
            q1 = norm.pdf(r)
            q2 = norm.pdf(phai, 0, math.pi/12 * math.pi/12)
            particle.weight = q1 + q2
            weight_sum += particle.weight

        normalize_coe = 1.0 / weight_sum
        for particle in self.particles:
            particle.weight = particle.weight * normalize_coe

        rospy.loginfo('measurement updated')
        self.resampling()

        for particle in self.particles:
            particle.set_x()

if __name__ == '__main__':
    mcl = TurtleMCL()
    while not mcl.init_mcl():
        pass
    while not rospy.is_shutdown():
        pass
