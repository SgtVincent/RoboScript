# GLIP 

## Installation 

To install GLIP, you need to build libraries locally, you need C/C++ and CUDA nvcc compilers in your system. 

If you have any problem installing GLIP in a conda environment with ROS installed due to compiling issues (depending on your system), please install GLIP in a new conda environment and install ROS in that environment later, since ROS installation will change C/C++ environment. 

```bash 
# 1. Install torch ACCORDING TO YOUR CUDA VERSION
# WARNING: since your need nvcc to build libraries, please make sure the cudatoolkit version is the same as your nvcc version locally.
# For example, if you have nvcc 11.6, please install cudatoolkit=11.6
# mamba install pytorch==1.12.1 torchvision==0.13.1 cudatoolkit=11.6 -c pytorch -c conda-forge

# 2. Install requirements
pip install -r requirements.txt

# 3. Install glip
pip install -e .
```
