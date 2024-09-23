
#include "assignment3_context.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

class ASBRPlannerManager : public planning_interface::PlannerManager{

public:

  ASBRPlannerManager() : planning_interface::PlannerManager() {}

  virtual 
  bool 
  initialize
  ( const moveit::core::RobotModelConstPtr& model,
    const rclcpp::Node::SharedPtr&,
    const std::string&){
    context.reset( new ASBRContext( model, 
				    std::string( "ASBR" ),
				    std::string( "manipulator" ) ) );
    
    return true;
  }

  virtual
  bool 
  canServiceRequest
  ( const moveit_msgs::msg::MotionPlanRequest& ) const 
  { return true; }

  virtual std::string getDescription() const
  { return std::string( "ASBRPlanner" ); }

  virtual
  void 
  getPlanningAlgorithms
  ( std::vector<std::string> &algs ) const{
    algs.resize(1);
    algs[0] = "ASBRPlanner";
  }

  virtual 
  planning_interface::PlanningContextPtr 
  getPlanningContext
  ( const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::msg::MoveItErrorCodes&) const{
    context->setPlanningScene( planning_scene );
    context->setMotionPlanRequest( req );
    return planning_interface::PlanningContextPtr( context );
  }

  virtual 
  void 
  setPlannerConfigurations
  (const planning_interface::PlannerConfigurationMap&){}

private:
  
  std::shared_ptr<ASBRContext> context;

};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( ASBRPlannerManager, planning_interface::PlannerManager );
