FROM ros:melodic

# Make workspace dir
RUN mkdir -p /home/catkin_ws/src
# Change WORKDIR
WORKDIR /home/catkin_ws/
# Copy roboteq_control folder
ADD . /home/catkin_ws/src/roboteq_control
# Install all dependencies
RUN apt-get update && apt-get install -y \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/
# Compile package
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'