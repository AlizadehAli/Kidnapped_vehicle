/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  std::default_random_engine gen;

  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  double std_x std_y std_theta
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

  // create Gaussian distributions with mean value of x, y and theta from GPS data
  // add Gaussian noise
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  weight.resize(num_particles); // resize the wight vector to new particle size
  particles = vector<Particle>(num_particles)
  for (int i=0; i<num_particles; ++i){
      particle.x = dist_x(gen);
      particle.y = dist_y(gen);
      particle.theta = dist_theta(gen);
      particle.weight = 1.0; // set all weights to 1.
      particles[i] = particle;
  }
  is_initialized = true
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  std::default_random_engine gen;
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  double std_x std_y std_theta
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];

  double x_ y_ theta_

  for (int i=0; i<num_particles; ++i){
      if (fabs(yaw_rate)<0.0001){ // equations when yaw rate is zero
          x_ = particles[i].x + velocity * delta_t * cos(particles[i].theta);
          y_ = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      }
      else { // equations when yaw rate is not zero
          x_ = particles[i].x + (velocity/yaw_rate) * ( sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta) );
          y_ = particles[i].y + (velocity/yaw_rate) * ( -cos(particles[i].theta + yaw_rate*delta_t) + cos(particles[i].theta) );
      }
      theta_ = particles[i].theta + yaw_rate * delta_t;

      // add Gaussian noise
      std::normal_distribution<double> dist_x(x_, std_x);
      std::normal_distribution<double> dist_y(y_, std_y);
      std::normal_distribution<double> dist_theta(theta_, std_theta);

      particles[i].x = dist_x(gen);
      particles[i].y = dist_y(gen);
      particles[i].theta = dist_theta(gen);
    }
  }

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for(int i=0; i<observations.size(); i++){
      double obs_x = observations[i].x;
      double obs_y = observations[i].y;
      double min_dist_value = 1000000.0;  // An arbitrary big number
      int min_dist_index = 0;
      for(int j=0; j<predicted.size(); j++){
          double distance = dist(obs_x, obs_y, predicted[j].x, predicted[j].y);
          if(distance < min_dist_value){
              min_dist_value = distance;
              min_dist_index = j;
        }
    }
    observations[i].id = min_dist_index;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a multi-variate Gaussian
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  /*
      Steps followed in this function:

        1. Transform each observation marker from the vehicle's coordinates to
              the map's coordinates with respect to the particle.

        2. Data assocociation; associate each transformed observation with a land mark identifier.

        3. Particle weights: product of Multivariate-Gaussian probability measurements.
   */

  for(int i=0; i<num_particles; i++){
      vector<LandmarkObs> lm_obsMap;

      for(int j=0; j<map_landmarks.landmark_list.size(); j++){
          LandmarkObs lm;

          lm.x = map_landmarks.landmark_list[j].x_f;
          lm.y = map_landmarks.landmark_list[j].y_f;
          lm.id= map_landmarks.landmark_list[j].id_i;

          if( dist(lm.x, lm.y, particles[i].x, particles[i].y) < sensor_range){
              lm_obsMap.push_back(lm);
          }
      }

      vector<LandmarkObs> vec_obs;
      for(int j=0; j<observations.size(); j++){
          LandmarkObs tObs;

          tObs.x  = particles[i].x + observations[j].x*cos(particles[i].theta) -
                                     observations[j].y*sin(particles[j].theta);
          tObs.y  = particles[i].y + observations[j].x*sin(particles[i].theta) +
                                     observations[j].y*cos(particles[j].theta);
          vec_obs.push_back(tObs);
      }

      dataAssociation(lm_obsMap, vec_obs);

      particles[i].weight = 1;
      vector<double> vec_senseX;
      vector<double> vec_senseY;
      vector<int> vec_associations;

      for(int j=0; j<vec_obs.size(); j++){
          double ox = vec_obs[j].x;
          double oy = vec_obs[j].y;
          int    associateId = vec_obs[j].id;

          double ux = lm_obsMap[associateId].x;
          double uy = lm_obsMap[associateId].y;

          double up = (ox - ux)*(ox - ux)/(2*std_landmark[0]*std_landmark[0]) +
                      (oy - uy)*(oy - uy)/(2*std_landmark[1]*std_landmark[1]);

          double Pxy = 1.0 / (2*M_PI*std_landmark[0]*std_landmark[1]) * pow(M_E, -1*up);
          vec_senseX.push_back(ox);
          vec_senseY.push_back(oy);
          vec_associations.push_back(associateId);
          particles[i].weight *= Pxy;
      }
      SetAssociations(particles[i], vec_associations, vec_senseX, vec_senseY);
    }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}