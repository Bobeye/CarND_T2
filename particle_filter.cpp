/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	default_random_engine gen;

	// number of particles
	num_particles = 10;

	// Resize wights and particle of the filter regarding the number of particles
	weights.resize(num_particles);
    particles.resize(num_particles);

	// Standard deviation for x, y and psi 
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2]; 

	// This line creates a normal (Gaussian) distribution for x
	normal_distribution<double> dist_x(x, std_x);
	
	// TODO: Create normal distributions for y and psi
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for(int i = 0; i < num_particles; ++i){
		particles[i].id = i; 
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen); 
		particles[i].weight = 1;
		weights[i] = particles[i].weight; 
	}

	is_initialized = true; 
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	
	// Standard deviation for x, y and psi 
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	
	for(int i = 0; i < num_particles ; ++i){
		// update  position
		double predict_x = velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
		double predict_y = velocity / yaw_rate * (-cos(particles[i].theta + yaw_rate * delta_t) + cos(particles[i].theta));
		double predict_theta = yaw_rate * delta_t;

		// Add Gaussian noise
		normal_distribution<double> dist_x(predict_x, std_x);
        normal_distribution<double> dist_y(predict_y, std_y);
        normal_distribution<double> dist_theta(predict_theta, std_theta);

		// Update the current particle
		particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double var_x = pow(std_x, 2);
	double var_y = pow(std_y, 2);
	// for normalization
	double sum_weights = 0;

	for(int i = 0; i < num_particles ; i++){
		double weight = 1.0;
		for(int j = 0; j < observations.size(); j++){
			// convert observations from Vehicle to Map space
			LandmarkObs map_observation;
			LandmarkObs veh_observation = observations[j];
			map_observation.x = (veh_observation.x * cos(particles[i].theta)) - (veh_observation.y * sin(particles[i].theta)) + particles[i].x;
			map_observation.y = (veh_observation.x * sin(particles[i].theta)) + (veh_observation.y * cos(particles[i].theta)) + particles[i].y;
			map_observation.id = veh_observation.id;

			// find the nearest landmark
			Map::single_landmark_s nearest_landmark;
			double nearest_dist = 1000000.0;
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
				double distance = dist(map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f, map_observation.x, map_observation.y);
				if (distance < nearest_dist) {
					nearest_dist = distance;
					nearest_landmark = map_landmarks.landmark_list[k];
				}
			}

			// Update weights
			double new_weight = exp(-0.5 * (pow((map_observation.x - nearest_landmark.x_f), 2) / var_x + pow((map_observation.y - nearest_landmark.y_f), 2) / var_y)) / (2 * M_PI * std_x * std_y);
			weight *= new_weight; 
		} 
		sum_weights += weight;
		particles[i].weight = weight; 
	}

	// Normalize weights
	for (int i = 0; i < num_particles; i++){
		particles[i].weight /= sum_weights;
		weights[i] = particles[i].weight;
	}	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	discrete_distribution<int> dist_particles(weights.begin(), weights.end());
	vector<Particle> resampled_particles;
	resampled_particles.resize(num_particles);
	for (int i = 0; i < num_particles; i++) {
		resampled_particles[i] = particles[dist_particles(gen)];
	}
	particles = resampled_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
