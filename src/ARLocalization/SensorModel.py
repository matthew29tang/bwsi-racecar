import numpy as np
from scipy.stats import norm
import rospy 

class SensorModel:

    def __init__(self):
        self.dist_dev = rospy.get_param('/ar_localization/sensor_dist_dev', 0.1)
        self.orbit_dev = rospy.get_param('/ar_localization/sensor_orbit_dev', 0.1)
        self.rot_dev = rospy.get_param('/ar_localization/sensor_rot_dev', 0.1)
        """
        TODO
        Do any initialization you need here
        """

    def update(self, observations, particles):

        # If there are no observations,
        # the particles do not change
        if not observations:
            return particles

        # re-weight the samples
        # based on the observations
        weights = self.reweight_particles(observations, particles)

        weights = self.normalize(weights)

        # re-sample the particles
        # based on their weights
        particles = self.resample_particles(particles, weights)

        return particles

    def reweight_particles(self, observations, particles):

        num_particles = particles.shape[0]
        weights = np.ones(num_particles)

        for observation in observations:

            obs_pose, ar_pose = observation

            # These are the quantities we are testing in
            # the particles:
            ##############################################

            # The distance from observation to the ar tag
            obs_dist = np.linalg.norm(obs_pose[:2])

            # The radial position of the observation 
            # around the ar tag
            obs_orbit_ang = ar_pose[2] - obs_pose[2]

            # The orientation of the observation
            # relative to the ar tag position
            obs_rot_ang = np.arctan(obs_pose[1]/obs_pose[0])

            ##############################################

            particle_pos = ar_pose[:2] - particles[:, :2]
            particle_ang = particles[:, 2]

            # The distance from the particles to the tag
            particle_dist = np.linalg.norm(particle_pos, axis=1)

            # The radial position of the particles
            # around the ar tag
            particle_orbit_vec = (particle_pos.T / particle_dist).T
            particle_orbit_ang = \
                    np.sign(particle_orbit_vec[:, 1]) * \
                    abs(np.arccos(particle_orbit_vec[:, 0]))

            # The orientation of the particles
            # relative to the ar tag position
            particle_rot_ang = particle_orbit_ang - particle_ang

            # Wrap the angles between -pi and pi,
            # centered around the desired values
            particle_orbit_ang = np.remainder(particle_orbit_ang + obs_orbit_ang + np.pi, 2*np.pi) - np.pi
            particle_rot_ang = np.remainder(particle_rot_ang + obs_rot_ang + np.pi, 2*np.pi) - np.pi

            # A normal distribution around radius and theta
            p_dist = norm.pdf(particle_dist, scale=self.dist_dev, loc=obs_dist)
            p_orbit_ang = norm.pdf(particle_orbit_ang, scale=self.orbit_dev, loc=0.)
            p_rot_ang = norm.pdf(particle_rot_ang, scale=self.rot_dev, loc=0.)

            # Multiply the weights together
            weights = np.multiply(weights, p_dist)
            weights = np.multiply(weights, p_orbit_ang)
            weights = np.multiply(weights, p_rot_ang)

        return weights

    def normalize(self, weights):
        """
        TODO
        normalize the probabilities to sum to 1
        """

        ##################
        # Your code here
	# weights = weights / np.sum(weights)

        return weights

    def resample_particles(self, particles, weights):
        """
        TODO
        resample the particles based on their weights
        Check out np.random.choice
        """
        num_particles = particles.shape[0]

        #################
        # your code here
	# particles = np.random.choice(particles, num_particles, weights)

	return particles
