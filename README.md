# OpenCVParticleFilter: Particle Filter Framework Implementation using OpenCV C++

OpenCVParticleFilter
============

Code for Particle Filter-based object tracking from color images. The framework utilizes importance sampling and dynamical system model for object tracking. It is capable of tracking in the presence of short-lived occlusion. The current implementation takes ground truth object location as input and infers object location when it is occluded. 

Pre-requisites
==============

- OpenCV 2.x (Included in repo)

Usage
=====

For usage and demo please see main.cpp inside the project

TODO: Write Update, Predict, Resample function description and help for calling here

Further work
============
Further work can be done to write color histogram-based particles from the following paper:

- Nummiaro, Katja, Esther Koller-Meier, and Luc Van Gool. "An adaptive color-based particle filter." Image and vision computing 21.1 (2003): 99-110.

Examples
========

TODO: Add examples
<img src="https://github.com/devkicks/OpenCVKinect/blob/master/CollectedSampleData/color/colorAGif.gif" alt="Color Image" width="400"/>
<img src="https://github.com/devkicks/OpenCVKinect/blob/master/CollectedSampleData/colorizedDepth/colorizedDepthAGif.gif" alt="Colorized Depth" width="400"/>
