#include "ParticleFilter.h"
#include <fstream>

cv::Mat loadImage(int imageNum);
cv::Mat readMatlabFile(std::string filename);

int main()
{
	cv::Mat ballLocation;
	ballLocation = readMatlabFile("ballSequence\\ballLocation.dat");

	std::cout << ballLocation << std::endl;
 	cv::Mat inR;
	inR = cv::Mat::eye(6, 6, CV_32FC1);
	inR = inR/2;
	
	int startFrame = 13;

	ParticleFilter pf(ballLocation.col(startFrame), inR);
	
	for(int i = startFrame; i < ballLocation.cols; i++)
	{
		cv::Mat inImage = loadImage(i+1);
		if(ballLocation.at<float>(0, i)+ballLocation.at<float>(1, i) != 0)
		{
			// update particle weights to the input location
			pf.update(ballLocation.col(i));
		}

		// resample particles using importance sampling
		pf.resampleParticles();
		cv::Mat pImage;
		pImage = pf.showParticles(inImage);
		pImage = pf.showPredictedLocation(pImage);
		cv::imshow("pf Out", pImage);
		cv::waitKey(0);

		// To save output
		//char buffer[256];
		//sprintf(buffer, "output\\saved%.5d.png", i);
		//cv::imwrite(buffer, pImage);
		
		// predict next state
		pf.predict();
	}
	return 1;
}

cv::Mat loadImage(int imageNum)
{
	cv::Mat inImage;
	char buffer[256];
	sprintf(buffer, "ballSequence\\Color_%d.png", imageNum);

	inImage = cv::imread(buffer);

	return inImage;
}

cv::Mat readMatlabFile(std::string filename)
{
	std::fstream file;
	cv::Mat retMat;
	file.open(filename.c_str(), std::ios::in | std::ios::binary);
	if(!file.is_open())
	{
		std::cout << "Error opening matlab binary file" << std::endl;
		std::cout << filename << std::endl;
		//isProbLoaded = false;
		//return false;		
	}

	// read the size of the Mat (including the number of channels)
	double colsMat, rowsMat, channelsMat;

	file.read((char*)&rowsMat, sizeof(rowsMat));
	file.read((char*)&colsMat, sizeof(colsMat));
	//file.read((char*)&channelsMat, sizeof(channelsMat));


	retMat = cv::Mat::zeros((int)rowsMat, (int)colsMat, CV_32FC1);

	//for(int k = 0; k < retMat.channels(); k++)
	//{
	for(int i = 0; i < retMat.cols; i++)
	{

		for(int j = 0; j < retMat.rows; j++)
		{

			double buff;
			file.read((char*)&buff, sizeof(buff));
			//file.read((char*)&m_ratioProb.at<double>(j, i), sizeof(m_ratioProb.at<double>(j, i)));
			retMat.at<float>(j,i) = (float)buff;
		}
	}
	//}
	//isProbLoaded = true;
	file.close();
	return retMat;
}