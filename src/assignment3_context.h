#include <moveit/planning_interface/planning_interface.h>

#include <random>

MOVEIT_CLASS_FORWARD( ASBRContext );

class ASBRContext : public planning_interface::PlanningContext {

public:

    typedef double weight;
    typedef std::size_t index;
    typedef std::vector<double> vertex;
    typedef std::vector<vertex> path;
    double euclidean_norm(const ASBRContext::vertex &q1, const ASBRContext::vertex &q2);
    bool random_bool();
    static std::random_device randomDevice;
    static std::mt19937 generator;

    ASBRContext( const moveit::core::RobotModelConstPtr& model,
                 const std::string &name,
                 const std::string& group );

    virtual ~ASBRContext();

    virtual bool solve( planning_interface::MotionPlanResponse &res );
    virtual bool solve( planning_interface::MotionPlanDetailedResponse &res );

    virtual void clear();
    virtual bool terminate();

    /**
       Feel free to change this method if you can do better.
       Test if a state collides with the scene.
       Call this method if you need to find if the robot collides with the
       environment in the given robot's state.
       \param[in] q The robot state
       \return      true if the robot state collides. false otherwise
    */
    bool state_collides( const vertex& q ) const;

    /**
       Utility method
       Linear interpolation between two configurations.
       \input qA The first configuration
       \input qB The second configuration
       \input t The interpolation parameter t=[0,1]
       \return The interpolated configuration
    */
    vertex interpolate( const vertex& qA, const vertex& qB, double t );


    /**
       TODO
       Sample a configuration according to a probability density function provided by the
       configuration weights.
       \input weights A vector of weights. w[i] is the weight of ith configuration. The higher the weight,
                      the higher the probability.
       \return The index of the randomly chosen configuration
    */
    index select_config_from_tree( const std::vector<weight>& w );

    /**
       TODO
       Create a random sample. The returned vertex represents a collision-free
       configuration (i.e. 6 joints) of the robot.
       \return  A collision-free configuration
    */
    vertex sample_nearby( const vertex& q, const vertex& q_goal );

    /**
       TODO
       Find the nearest configuration in the tree to a random sample
       \input q_rand The random configuration
       \return The nearest vertex in the tree
    */
    ASBRContext::index find_nearest_configuration( const std::vector<vertex>& V, const vertex& q );

    /**
       TODO
       Determine if a straight line path is collision-free between two configurations.
       \input q_near The first configuration
       \input q_rand The second configuration
       \return True if the path is collision-free. False otherwise.
    */
    bool is_local_path_collision_free( const vertex& q, const vertex& q_rand );

//    // Declaration of the extend function
    bool extend(std::vector<vertex>& V_from, std::vector<index>& parent_from,
                std::vector<weight>& w_from, const std::vector<vertex>& V_to,
                long& bridgePointFromIdx, long& bridgePointToIdx, double nearThreshold,
                const vertex& q_target);

    /**
       TODO
       Once the goal configuration has been added to the tree. Search the tree to find and return the
       path between the root (q_init) and the goal (q_goal).
       \input q_init The root configuration
       \input q_goal The goal configuration
       \return The path (a sequence of configurations) between q_init and q_goal.
    */
//  path search_path( const std::vector<vertex>& V,
//		    const std::vector<index> parent,
//		    const index& idx_init,
//		    const index& idx_goal );
    path search_path(const std::vector<vertex>& V, const std::vector<index>& parent, const index& idx_init, const index& idx_goal);

    /**
       TODO
       This is the main EST algorithm that adds vertices/edges to a tree and searches for a path.
       \input q_init The initial configuration
       \input q_goal The goal configuration
       \return The path between q_init and q_goal
    */
    path est( const vertex& q_init, const vertex& q_goal );

protected:

    moveit::core::RobotModelConstPtr robotmodel;

};
