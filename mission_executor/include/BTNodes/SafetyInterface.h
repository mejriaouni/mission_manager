
namespace done
{

class SafetyInterface
{

public:

    SafetyInterface()
    {}

    // Behavior tree mock node
    BT::NodeStatus isPersonDetected()
    {
        return BT::NodeStatus::FAILURE;
    }

};
}//namespace done
