#include "bumperbot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"

namespace bumperbot_planning {
    DijkstraPlanner:: DijkstraPlanner(): Node("dijkstra_node") { // constructor
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());// will allow us to retrieve the current position of robot on the map
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Quality of service object that defines how messages should be handled between nodes.
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // initialise the ROS2 Interface of Publishers and subscribers

        // Subscribers
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&DijkstraPlanner::mapCallBack, this, std::placeholders::_1 ));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(& DijkstraPlanner::goalCallBack, std::placeholders::_1));

        //Publishers
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/dijkstra/path", 10);
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map", 10);
    }

    void DijkstraPlanner::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr map) { // used whenever we receive a new occupancy grid on the map topic
        map_ = map;
        visited_map_.header.frame_id = map->header.frame_id;
        visited_map_.info = map->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);
    }
    void DijkstraPlanner::goalCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr pose) { // excecuted whenever we recieve a new goal.
        if (!map_) {
            RCLCPP_ERROR(get_logger(), "No map received!");
            return;
        }

    } 

        // this is the CORE of our planner. It's what will implement the Dijkstra algorithm to calc the plan using the map using start pos and goal pos
        nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal) {

        }
}