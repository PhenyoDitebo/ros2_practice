#include <queue>

#include "bumperbot_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace bumperbot_planning {
    DijkstraPlanner:: DijkstraPlanner(): Node("dijkstra_node") { // constructor
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());// will allow us to retrieve the current position of robot on the map
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Quality of service object that defines how messages should be handled between nodes.
        rclcpp::QoS map_qos(10);
        map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        // initialise the ROS2 Interface of Publishers and subscribers

        // Subscribers
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&DijkstraPlanner::mapCallBack, this, std::placeholders::_1));
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

        visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1); // clean up. So DJ doesn't look at nodes its already looked at to avoid looping forever -- wipe visited list from last run
        geometry_msgs::msg::TransformStamped map_to_base_tf;
        try {
            map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero); // gives starting point of the DJ search, and maybe footprint of bot relative to that (??)
        }
        catch(const tf2::TransformException &ex) { // unable to retreive the starting pos, or current position of the planner
            RCLCPP_ERROR(get_logger(), "Could not transform frrom map to base_footprint.");
            return;
        } 

        // ------------- TYPE CONVERSION -------------
        // Takes the spatial data from the transform object and packs it into the Pose (position) object.
        // We do this because the transform and position objects use different data structures. (??)
        geometry_msgs::msg::Pose map_to_base_pose;
        map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
        map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
        map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

        auto path = plan(map_to_base_pose, pose->pose);

        if (!path.poses.empty()) {
            RCLCPP_INFO(get_logger(), "Shortest path found.");
            path_pub_ -> publish(path);
        }
        
        else {
            RCLCPP_WARN(get_logger(), "No proper path found, or path is impossible to map out. Either way it's not happening.");
        }

    } 

        // this is the CORE of our planner. It's what will implement the Dijkstra algorithm to calc the plan using the map using start pos and goal position
        nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal) {
            // will use to indicate what it's node neighbors are to be added to priority queue. Each cell should have only 4 neighbours (+)
            std::vector<std::pair<int, int>> explore_directions = {
                {-1,0}, {1,0}, {0,-1}, {0,1} // down, up, left, right.
            };

            // remember, the dijkstra algo is basically a priority queue that uses weight/costs to see which node to visit next
            std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes; // will keep track of the nodes we are yet to visit. Will store in a vector of graph nodes. The greater part is for comparison.
            std::vector<GraphNode> visited_nodes; // used to store the list of the graph nodes that have already been visited by the algo to avoid double visits.  

            // turning the start co-ordinate into a graph node and pushing it into the queue of nodes we need to visit.
            pending_nodes.push(worldtoGrid(start));

            // going to use this object to recursively keep track of the currently active node when we are exploring.
            GraphNode active_node;

            while(!pending_nodes.empty() && rclcpp::ok()) {
                active_node = pending_nodes.top(); // .top() is used for priroity queues over .front(): .top() always points to the node with the lowest weight/cost.
                pending_nodes.pop();

                if (worldtoGrid(goal) == active_node) { // if the goal node is the active_node, we reached the goal.
                    break;
                }
                // else if that hasn't happened, explore directions (4, +) 
                // "have we been here before?"
                for (const auto &dir : explore_directions) {
                    GraphNode new_node = active_node + dir; // we are adding the co-ordinates
                    if (std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && poseOnMap(new_node) && map_->data.at(poseToCell(new_node)) == 0) { // checking if the new nodes had already been visited and in the vector 
                       // [beginning, end, node we are looking for]. Also, the std::find returns the last node (.end) if the node we looked for hasn't been found.
                       // need to check if the location of the node is now also in-scope. We will use a support function.
                       // when all conditions are met, we can add the new node.

                    } 
                }
            }
        }

        // before starting exploration, we need to make a small conversion.
        // start and end(goal) position are expressed with reference/respect to the map frame.
        // before using the start and end (goal) nodes, we need to convert them to co-ordinates of the grid. Co-ordinates that rep a certain point in the occupancy grid that we are using the planning.
        GraphNode DijkstraPlanner::worldtoGrid(const geometry_msgs::msg::Pose &pose) { // pose is a reference to the robot's position in the real world (meters??)

            // create a new graph node whose co-ordinates are corresponding to a certain position in the map.
            int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x)/ map_->info.resolution);
            int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y)/ map_->info.resolution);

            return GraphNode(grid_x, grid_y);

        };

         bool DijkstraPlanner::poseOnMap (const GraphNode &node) {
            return node.x >= 0 && node.x < static_cast<int>(map_->info.width) // is the node's x co-ordinate bigger than 0 and smaller than the width of the map?
                && node.y >= 0 && node.y < static_cast<int>(map_->info.height); // is the node's y co-ordinate bigger than 0 and smaller than the height of the map?

         }

         unsigned int DijkstraPlanner::poseToCell(const GraphNode &node) { // will convert the y and x co-ordinate into a simple index  to access a specific cell of the data vector
            return node.y * map_->info.width + node.x; // this will get us to the index where the co-ordinate is actually stored.
         }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

