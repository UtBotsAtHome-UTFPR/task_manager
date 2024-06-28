#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

// file that contains the custom nodes definitions
#include "recognition.hpp"

using namespace BT;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bt");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    Recognize recognition_leaf;

    
    factory.registerNodeType<NewFace>(("new_face"));
    
    // Not cursed, just fucked up
    factory.registerSimpleAction("recognition", [&recognition_leaf](TreeNode& node){ return recognition_leaf.Recognition(node); });

    auto tree = factory.createTreeFromFile("src/utbots_tasks/task_manager/bt_src/recognition.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRootWhileRunning();

    return 0;
}