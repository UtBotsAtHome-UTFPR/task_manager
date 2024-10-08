# utbots_tasks
- ROS repository designed to orchestrate tasks for RoboCup@Home.

### Building

```bash
cd ~/catkin_ws/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_tasks.git
cd ..
catkin_make
```

### Dependencies

Some tasks require additional packages. You need to go to the relevant repository and build it. 
To start this, you can install utbots_dependencies:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
cd ..
catkin_make
```
To install Python dependencies for this repository:

```bash
cd /usr/bin
sudo python3 -m venv venv_utbots_tasks
source /usr/bin/venv_utbots_tasks/bin/activate
cd $HOME/catkin_ws/src/utbots_tasks
sudo pip install -r requirements.txt
```

## Running

To test it, you can launch a specific file, such as:

```bash
roslaunch task_manager answer_questions.launch 
```
