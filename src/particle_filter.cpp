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

  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  std::default_random_engine gen;
  num_particles = 100;  // TODO: Set the number of particles

  double std_x;
  double std_y;
  double std_theta;
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

  // create Gaussian distributions with mean value of x, y and theta from GPS data
  // add Gaussian noise
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

//  weight.resize(num_particles); // resize the wight vector to new particle size
  particles = vector<Particle>(num_particles);
  Particle particle;
  for (int i=0; i<num_particles; ++i){
      particle.x = dist_x(gen);
      particle.y = dist_y(gen);
      particle.theta = dist_theta(gen);
      particle.weight = 1.0; // set all weights to 1.
      particles[i] = particle;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {

  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;
  double std_x;
  double std_y;
  double std_theta;
  std_x = std_pos[0];
  std_y = std_pos[1];
  std_theta = std_pos[2];

  double x_;
  double y_;
  double theta_;

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

  for(auto& obs: observations){
    double min_dist_value = 1000000.0;

    for(const auto& pred: predicted){
      double distance = dist(obs.x, obs.y, pred.x, pred.y);
      if( min_dist_value > distance){
        min_dist_value = distance;
        obs.id = pred.id;
      }
    }
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

        2. Data association; associate each transformed observation with a land mark identifier.

        3. Particle weights: product of Multivariate-Gaussian probability measurements.
   */

  for(auto& p: particles){
    p.weight = 1.0;

    // collect the valid landmarks
    vector<LandmarkObs> predictions;
    for(const auto& lmrk: map_landmarks.landmark_list){
      double distance = dist(p.x, p.y, lmrk.x_f, lmrk.y_f);
      if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
        predictions.push_back(LandmarkObs{lmrk.id_i, lmrk.x_f, lmrk.y_f});
      }
    }

    // convert observations from vehicle to map coordinates
    vector<LandmarkObs> observations_map;
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);

    for(const auto& obs: observations){
      LandmarkObs tmp;
      tmp.x = obs.x * cos_theta - obs.y * sin_theta + p.x;
      tmp.y = obs.x * sin_theta + obs.y * cos_theta + p.y;
      observations_map.push_back(tmp);
    }

    // find landmark index for each observation
    dataAssociation(predictions, observations_map);

    // compute the particle's weight:
    for(const auto& obs_m: observations_map){

      Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id-1);
      double x_term = pow(obs_m.x - landmark.x_f, 2) / (2 * pow(std_landmark[0], 2));
      double y_term = pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
      double w = exp(-(x_term + y_term)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
      p.weight *=  w;
    }

    weights.push_back(p.weight);
  }
}


void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  /*
  */
  std::default_random_engine gen;
  vector<Particle> new_particles;

  vector<double> weights;
  for(int i=0; i<num_particles; i++){
    weights.push_back(particles[i].weight);
  }
  std::uniform_real_distribution<double> unirealdist(0.0, 1.0);
  int index   = int(unirealdist(gen) * num_particles);
  double beta = 0.0;
  double mw   = *max_element(weights.begin(), weights.end());
  for(int i=0; i<num_particles; i++){
    beta += unirealdist(gen) * 2.0 * mw;
    while(beta > weights[index]){
      beta -= weights[index];
      index = (index+1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
  weights.clear();
}


/* particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
 * associations: The landmark id that goes along with each listed association
 * sense_x: the associations x mapping already converted to world coordinates
 * sense_y: the associations y mapping already converted to world coordinates
 */

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

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