#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h" // to retreive the current position of the robot on the map -- central database and processing engine for co-ordinate transforms in ROS 2 Nodes
#include "tf2_ros/transform_listener.h" // to retreive and store coordinate frame transformations in ROS2 applications


namespace bumperbot_planning {

    // ----------- SUPPORT CLASS ---------
    // used to respresent an abstruct nodes in the graph, their co-ordinates and properties
    struct GraphNode {
        // Properties
        int x;
        int y;
        int cost; // cost of travel to the next node.
        std::shared_ptr<GraphNode>prev; // pointer to their parent node.

        // constructor
        GraphNode (int in_x, int in_y): x(in_x), y(in_y), cost(0) { // x and y co-ordinates and cost

        } 

        // base constructor. Use main one to set co-ordinates of new node made as (0,0)
        GraphNode() :GraphNode(0,0) {}


        // overloading the operators.
        bool operator>(const GraphNode &other) const { // "other" is the other node we are comparing to.
            return cost > other.cost;
        }
        bool operator==(const GraphNode &other) const {
            return x == other.x && y == other.y;
        }

        GraphNode operator+(std::pair<int, int> const &other) {
            GraphNode res(x + other.first, y + other.second); // creating a new graph node, who's co-ordinates are the addition of the node we were on and the other node we received as input.
            return res; // return result.
        }
    };
    class DijkstraPlanner : public rclcpp::Node
{
    public:
        DijkstraPlanner();

    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_; // will receieve the map of the env that the planner is going to use for planning.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_; // will receive the destination of the planner.

        // going to use these publishers to visualise the results of the DJ planner in RViz.
        // and also to share them with other nodes that wanna use the path planned by the Dijkstra algo.
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; //publish the path
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; // will publish the map (occupany grid) containing the cells that the algo has visited so we can see which cells have been explored in map

        nav_msgs::msg::OccupancyGrid::SharedPtr map_; //shared pointer to the map currently in use to plan a path from the starting point to the destination.
        nav_msgs::msg::OccupancyGrid visited_map_; // we will use to contain the visited map

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // ---------- FUNCTIONS, Declaration ----------
        void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr map); // will be called when the map topic receives a message -- when it receives the map of the env
        void goalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr pose); // will be called whenever a message is recieved by the positiion subscriber. (??)

        geometry_msgs::msg::Pose gridToWorld(const GraphNode &node);

        GraphNode worldtoGrid(const geometry_msgs::msg::Pose &pose); // returns a graph node object whose co-ordinates respond to a position in the occupancy grid.
        bool poseOnMap (const GraphNode &node); // is position on Map? This function will be used to test that.
        unsigned int poseToCell(const GraphNode &node); // converts a 2D co-ordinate (e.g. Row 5, Col 10) into a single array (510)

        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal); //used to plan a path. Will take the starting position and the goal position.
};

}

