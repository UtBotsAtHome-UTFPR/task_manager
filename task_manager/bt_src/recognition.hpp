// Basic includes for using a behavior tree
// This file will contain action nodes
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

// Including the actions that will be called. See actions tutorial to learn how to create them and make them acessible for outside packages
// When actions get moved to a vision_actions, voice_actions and so on package this will need to be changed
#include <utbots_face_recognition/RecognizeAction.h>
#include <utbots_actions/new_faceAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

class Recognize
{
public:
    Recognize();

    ~Recognize(){}

    BT::NodeStatus Recognition(BT::TreeNode& tree);

private:
    actionlib::SimpleActionClient<utbots_face_recognition::RecognizeAction> recognition_action;

};

class NewFace : public BT::StatefulActionNode
{
public:
    NewFace(const std::string& name, const BT::NodeConfiguration& config);

    // This function must be declared even if no ports are used
    static BT::PortsList providedPorts();

    void NewFaceFeedbackCb(utbots_actions::new_faceFeedback msg);

    BT::NodeStatus onStart();
    
    BT::NodeStatus onRunning();

    void onHalted();

private:
    
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<utbots_actions::new_faceAction> new_face_action;
    int n_pics;

};