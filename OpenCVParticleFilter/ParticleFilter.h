#pragma once

#include "common.h"

class ParticleFilter
{
	// Particles
	cv::Mat Xn;

	// Weights
	cv::Mat Wn;

	// Process noise
	cv::Mat R;

	// state transition model
	cv::Mat A;

	// measurement model
	cv::Mat H;

	// dimensions of the state vector
	unsigned int Ds;

	// number of particles
	unsigned int N;

	// redistributions threshold
	unsigned int N_threshold;
	
public:
	ParticleFilter(void);
	ParticleFilter(const cv::Mat &inZ, const cv::Mat &inR, unsigned int inN=1000);
	void resampleParticles();
	void predict();
	void update(const cv::Mat &inZ);
	cv::Mat currentPrediction();
	cv::Mat showParticles(const cv::Mat &inImage);
	cv::Mat showPredictedLocation(const cv::Mat &inImage);
	~ParticleFilter(void);
	
private:   // helper function
	void initParticles(cv::Mat &lR, cv::Mat &uR);
	void selectDynamicModel(unsigned int D);
	void normalizeWeights();
	void weightingParticles(const cv::Mat &inZ);
	double distanceGaussian(const cv::Mat &inZ,  const cv::Mat &pZt);
	cv::Mat resampler(const cv::Mat &inWn);
};

