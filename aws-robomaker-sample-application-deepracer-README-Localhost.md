
**This file describes steps to perform build/launch aws-robomaker-sample-application-deepracer simulation.**

**Prerequisite:**

1. ROS2 Dashing installation [Ubuntu (18.04) Bionic] 
https://index.ros.org//doc/ros2/Installation/Dashing/Linux-Install-Debians/

2. Development tools and ros tools.

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev

**Python pkg depencencies:**

sudo apt-get update 
sudo apt-get install python3.6

sudo -H pip3 install boto3==1.9.23
sudo -H pip3 install futures==3.1.1
sudo -H pip3 install gym==0.10.5
sudo -H pip3 install kubernetes==7.0.0
sudo -H pip3 install minio==4.0.5
sudo -H pip3 install numpy==1.13.3
sudo -H pip3 install pandas==0.20.2
sudo -H pip3 install Pillow==4.3.0
sudo -H pip3 install pygame==1.9.3
sudo -H pip3 install PyYAML==4.2b1
sudo -H pip3 install redis==2.10.6
sudo -H pip3 install rospkg==1.1.7
sudo -H pip3 install scipy==0.19.0
sudo -H pip3 install tensorflow==1.12.2
sudo -H pip3 install rl-coach-slim==0.11.1
sudo -H pip3 install annoy==1.8.3
sudo -H pip3 install matplotlib==2.0.2
sudo -H pip3 install netifaces==0.10.7
sudo -H pip3 install scikit-image==0.13.0 

**Gazebo models:**
  mkdir -p ~/.gazebo
  hg clone https://bitbucket.org/osrf/gazebo_models ~/.gazebo/models

**Building simulation:**
Get source code of Deepracer application and build as below commands.

  cd aws-robomaker-sample-application-deepracer/simulation_ws
  source /opt/ros/dashing/setup.bash
  rosws update
  colcon build
  source install/setup.sh

**Running simulation:**
Need to set below Environment variables before Launch application:

# Note: Use your own S3 Bucket and its region, AWS keys, model S3 prefix for traind model to avoid overriding.
1. export MARKOV_PRESET_FILE="deepracer.py" MODEL_S3_BUCKET=<name_of_the_S3_bucket> MODEL_S3_PREFIX=<s3_path_where_you_want_to_store> ROS_AWS_REGION=<aws_region> AWS_ACCESS_KEY_ID=<access_key_id> AWS_SECRET_ACCESS_KEY=<secret_access_key>

2. export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins/

3. export WORLD_NAME=easy_track (easy_track/medium_track/hard_track)

4. export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~<Local path of example>/aws-robomaker-sample-application-deepracer/simulation_ws/src/deepracer_simulation/models

**Launching the scripts:**

1. For Local Training,
ros2 launch deepracer_simulation local_training.launch.py

2. For Evalution,
ros2 launch deepracer_simulation evaluation.launch.py

