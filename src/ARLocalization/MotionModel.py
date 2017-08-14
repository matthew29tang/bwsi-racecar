import numpy as np
import rospy

class MotionModel:

    def __init__(self):
        """
        TODO
        Do initialization you need here.
        """
        self.a1 = 1
        self.a2 = 1
        self.a3 = 1
        self.a4 = 1
        self.trans_error = rospy.get_param('/ar_localization/motion_trans_error', [1., 1.])
        self.angle_error = rospy.get_param('/ar_localization/motion_angle_error', [1., 1.])

    def update(self, prev_pose, curr_pose, particles):
        """
        TODO
        Use the prev_pose and curr_pose to update the
        particles.

        prev_pose and curr_pose are tuples (x, y, theta)

        particles is a 3xN numpy array where each row represents an (x, y, theta) state

        the output should be a new 3xN numpy array.
        """
        num_particles = particles.shape[0]

        (prev_x, prev_y, prev_theta) = prev_pose
        (curr_x, curr_y, curr_theta) = curr_pose

        ########################
        # Your work here

        d_rot1 = np.arctan2(curr_y-prev_y, curr_x-prev_x) - prev_theta
        d_trans = np.sqrt(np.square(prev_x-curr_x) + np.square(prev_y - curr_y))
        d_rot2 = curr_theta - prev_theta - d_rot1
        
        # robot doens't move
        if abs(d_trans) <= 0.03:
            print ("Car not moving")
            return particles

	d_rot1_ar = np.full((num_particles), d_rot1)
	d_trans_ar = np.full((num_particles), d_trans)
	d_rot2_ar = np.full((num_particles), d_rot2)

	i = abs(self.a1*d_rot1 + self.a2*d_trans)
	j = abs(self.a3*d_trans + self.a4*(d_rot1+d_rot2))
	k = abs(self.a1*d_rot2 + self.a2*d_trans)

	a = np.random.normal(0, i, num_particles)
	b = np.random.normal(0, j, num_particles)
	c = np.random.normal(0, k, num_particles)

	#a,b,c = 0, 0, 0
	
        d2_rot1 = d_rot1_ar - a
        d2_trans = d_trans_ar - b
        d2_rot2 = d_rot2_ar - c
        
        particles[:,0] += d2_trans*np.cos(particles[:,2]+d2_rot1)
        particles[:,1] += d2_trans*np.sin(particles[:,2]+d2_rot1)
        particles[:,2] += d2_rot1 + d2_rot2
	print "working!"

        return particles

