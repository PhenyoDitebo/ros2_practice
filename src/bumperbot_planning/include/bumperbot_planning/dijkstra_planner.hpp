#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h" // to retreive the current position of the robot on the map -- central database and processing engine for co-ordinate transforms in ROS 2 Nodes
#include "tf2_ros/transform_listener.h" // to retreive and store coordinate frame transformations in ROS2 applications


namespace bumperbot_planning {
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

        // ---------- FUNCTIONS ----------
        void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr map); // will be called when the map topic receives a message -- when it receives the map of the env
        void goalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr pose); // will be called whenever a message is recieved by the positiion subscriber. (??)

        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal); //used to plan a path. Will take the starting position and the goal position.
};

}

