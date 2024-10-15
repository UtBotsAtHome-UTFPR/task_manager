#include "recognition.hpp"

Recognize::Recognize() : recognition_action("recognize", true){

    ROS_INFO("[BT::RECOGNITION] Waiting for recognition action server to start");
    // wait for the action server to start
    // There are functions such as recognition_action.isServerConnected() which should be able to be put on a while loop, it works but system is still unreactive to ctrl + c. Someone please fix all calls of this function later.
    recognition_action.waitForServer();

    ROS_INFO("[BT::RECOGNITION] Recognition server has started");
}

BT::NodeStatus Recognize::Recognition(BT::TreeNode& tree){

    utbots_actions::recognitionGoal goal;

    goal.ExpectedFaces.data = 0;

    ROS_INFO("[BT::RECOGNITION] Sending recognition action");
    recognition_action.sendGoalAndWait(goal, ros::Duration(3));
    
    // Change to both get and send image to parameter server (action needs to have sensor_msgs.msg Image as definition, goal is already done)
    utbots_actions::recognitionResultConstPtr result = recognition_action.getResult();

    if(recognition_action.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("[BT::RECOGNITION] Recognition system failed or image had no people: ");
        return BT::NodeStatus::FAILURE;
    }

    // Put this into the vatiable server
    ROS_INFO("[BT::RECOGNITION] Recognized people are: ");
    // Trocar array por bbox
    //for_each(result.get()->People.array.begin(), result.get()->People.array.end(), [](const vision_msgs::Object elem) { std::cout << elem.id.data << " "; });

    return BT::NodeStatus::SUCCESS;
    
}

Train::Train(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config), train_action("train", true) {
    
    ROS_INFO("[BT::TRAIN] Waiting for train action server to start");
    // wait for the action server to start
    train_action.waitForServer();
    
    //ros::Subscriber sub = nh.subscribe<utbots_actions::new_faceFeedback>("new_face/feedback", 0, &NewFace::NewFaceFeedbackCb);

    ROS_INFO("[BT::TRAIN] Train server has started");

}

BT::PortsList Train::providedPorts(){
    // This is a placeholder and not in use
    return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus Train::onStart(){
    utbots_actions::trainGoal goal;

    // Change this to take the name from the Behavior tree database

    train_action.sendGoal(goal);

    ROS_INFO("[BT::NEW_FACE] Running train");

    return BT::NodeStatus::RUNNING;

}

BT::NodeStatus Train::onRunning(){
    
    //float p_complete = (n_pics / (float)total_n_pictures) * 100;

    // Change so it publishes the percentage complete
    if(train_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("[BT::TRAIN] Succeeded");
        return BT::NodeStatus::SUCCESS;
    }
    else if(train_action.getState() == actionlib::SimpleClientGoalState::PREEMPTED || train_action.getState() == actionlib::SimpleClientGoalState::REJECTED || train_action.getState() == actionlib::SimpleClientGoalState::RECALLED || train_action.getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("[BT::TRAIN] Fell into a failed state");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;

}

void Train::onHalted(){
    ROS_INFO("[BT::TRAIN] Action halted");
    train_action.cancelAllGoals();
}

NewFace::NewFace(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config), new_face_action("new_face", true) {

    ROS_INFO("[BT::NEW_FACE] Waiting for new face action server to start");
    // wait for the action server to start
    new_face_action.waitForServer();
    
    //ros::Subscriber sub = nh.subscribe<utbots_actions::new_faceFeedback>("new_face/feedback", 0, &NewFace::NewFaceFeedbackCb);

    ROS_INFO("[BT::NEW_FACE] New face server has started");
    
}

BT::PortsList NewFace::providedPorts()
  {
    // This is a placeholder and not in use
    return { BT::InputPort<std::string>("message") };
  }

void NewFace::NewFaceFeedbackCb(utbots_actions::new_faceFeedback msg){
    this->n_pics = msg.pics_taken.data;
}

BT::NodeStatus NewFace::onStart(){
    // Chama a ação
    // Retorna running

    utbots_actions::new_faceGoal goal;

    // Change this to take the name from the Behavior tree database
    goal.n_pictures.data = 10;
    goal.name.data = "Operator";

    new_face_action.sendGoal(goal);

    ROS_INFO("[BT::NEW_FACE] Running new faces");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NewFace::onRunning(){
    
    //float p_complete = (n_pics / (float)total_n_pictures) * 100;

    // Change so it publishes the percentage complete

    if(new_face_action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("[BT::NEW_FACE] Succeeded");
        return BT::NodeStatus::SUCCESS;
    }
    else if(new_face_action.getState() == actionlib::SimpleClientGoalState::PREEMPTED || new_face_action.getState() == actionlib::SimpleClientGoalState::REJECTED || new_face_action.getState() == actionlib::SimpleClientGoalState::RECALLED || new_face_action.getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("[BT::NEW_FACE] Fell into a failed state");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;

}

void NewFace::onHalted(){
    ROS_INFO("[BT::NEW_FACE] Action halted");
    new_face_action.cancelAllGoals();
}