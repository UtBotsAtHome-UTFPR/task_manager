// Basic includes for using a behavior tree
// This file will contain action nodes
#include <behaviortree_cpp_v3/action_node.h>
#include <ros/ros.h>

// Including the actions that will be called. See actions tutorial to learn how to create them and make them acessible for outside packages
// When actions get moved to a vision_actions, voice_actions and so on package this will need to be changed
#include <utbots_face_recognition/RecognizeAction.h>

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