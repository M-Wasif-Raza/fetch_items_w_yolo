# Base Image - ROS Noetic (OSRF Official)
FROM osrf/ros:noetic-desktop-full

# Install basic dependencies and required packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    nano \
    gedit \
    build-essential \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-smach \
    ros-noetic-smach-ros \
    ros-noetic-sound-play \
    ros-noetic-turtlebot3* \
    ros-noetic-dwa-local-planner \
    espeak \
    espeak-data \
    libportaudio2 \
    libportaudiocpp0 \
    portaudio19-dev \
    python3-tk \
    libjansson-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install yolov4 beautifulsoup4 vosk pyttsx3 sounddevice 
RUN pip install yolo-v4

WORKDIR /opt
RUN git clone https://github.com/AlexeyAB/darknet.git && \
    cd darknet && \
    make LIBSO=1 OPENCV=1 AVX=1 OPENMP=1 -j $(nproc) && \
    chmod +x darknet

# Create non-root user
ARG USERNAME=wasif
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo for the new user
RUN apt-get update \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Fix potential permission issues
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh 
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

# Set working directory
WORKDIR /home/$USERNAME/