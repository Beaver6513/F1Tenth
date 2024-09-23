# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:foxy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-tk \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d
RUN apt install ros-foxy-slam-toolbox

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .
RUN pip3 install numpy matplotlib tk

# ros2 gym bridge
RUN mkdir -p sim_ws/src/f1tenth_gym_ros
COPY . /sim_ws/src/f1tenth_gym_ros
RUN source /opt/ros/foxy/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

RUN printf '#!/bin/bash\n\
tmux new-session -d -s mysession\n\
tmux rename-window -t mysession:0 "Main"\n\
tmux send-keys -t mysession:0 "colcon build" C-m\n\
tmux send-keys -t mysession:0 "source /opt/ros/foxy/setup.sh" C-m\n\
tmux send-keys -t mysession:0 "source install/setup.sh" C-m\n\
tmux send-keys -t mysession:0 "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" C-m\n\
tmux split-window -h -t mysession:0\n\
tmux send-keys -t mysession:0.1 "colcon build" C-m\n\
tmux send-keys -t mysession:0.1 "source /opt/ros/foxy/setup.sh" C-m\n\
tmux send-keys -t mysession:0.1 "source install/setup.sh" C-m\n\
tmux send-keys -t mysession:0.1 "ros2 run f1tenth_gym_ros DEGF" C-m\n\
tmux select-pane -t mysession:0.0\n\
tmux attach-session -t mysession' > /sim_ws/start-tmux.sh

RUN chmod +x /sim_ws/start-tmux.sh

WORKDIR '/sim_ws'
CMD ["start-tmux.sh"]
ENTRYPOINT ["/bin/bash"]
