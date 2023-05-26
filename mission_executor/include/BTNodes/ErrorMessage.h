namespace done
{

class ErrorMessage : public BT::SyncActionNode
{

public:
  ErrorMessage(const std::string& name);

  // Behavior tree mock action node for error message
  BT::NodeStatus tick() override;

};

}// namespace done


