#include <rclcpp/rclcpp.hpp> // core ROS2 Library
#include <std_msgs/msg/string.hpp> // includes the string message type definition

// rclcpp is the namespace for the ROS2 C++ Client Library, it is the family name for all C++ ROS2 classes and functions
// std_msgs is the namespace for standard message definitions, msg is the sub-namespace for message types
// rclcpp::Node is the class for creating a ROS2 node, which we inherit to create our own node class
// rclcpp::Publisher is the class for creating a publisher object, and is the tool for sending messages
// rclcpp::init is the function to initialize the ROS2 system (or start the ROS2 client library)

#include <chrono>
using namespace std::chrono_literals;

class simpleqospublisher : public rclcpp::Node { // create a class derived from rclcpp::Node, inherting its properties
    public:
        simpleqospublisher() : Node("simple_qos_publisher"), qos_profile_pub_(10), counter_(0) // constructor

        // Remember QoS Policies. We apply them here for node communication.
        // I think nodes use all 8 QoS policies during communication.
        {
            declare_parameter<std::string>("reliability", "system_default");
            declare_parameter<std::string>("durability", "system_default");

            const auto reliability = get_parameter("reliability").as_string();
            const auto durability = get_parameter("durability").as_string();

            // Reliability, Quality of Connection, Transit.
            if (reliability == "best_effort") {
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                RCLCPP_INFO(get_logger(), "[Reliability]: Best Effort."); // Best effort: attempt to deliver samples, but may lose them if the network is not robust.
            }
            else if (reliability == "reliable") {
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                RCLCPP_INFO(get_logger(), "[Reliability]: Reliable."); // Reliable: guarantee that samples are delivered, may retry multiple times.
            }
            else if (reliability == "system_default") {
                qos_profile_pub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Reliability]: System Default."); // defaults back to system default.
            }
            else {
                RCLCPP_ERROR_STREAM(get_logger(), "Selected reliability QoS: " << reliability << " doesn't exist.");
                return;
            }

            // Durability, Memory of the Publisher, History.
            if (durability == "volatile") { // Volatile: no attempt is made to persist samples.
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                RCLCPP_INFO(get_logger(), "[durability]: volatile");
            } 
            else if (durability == "transient_local") { // Transient local, the "latching" policy: the publisher becomes responsible for persisting samples for “late-joining” subscriptions.
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                RCLCPP_INFO(get_logger(), "[durability: transient_local]");
            }
            else if (durability == "system_default") {
                qos_profile_pub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Durability]: System Default."); // defaults back to system default.
            }
            else {
                RCLCPP_ERROR_STREAM(get_logger(), "Selected durability QoS: " << durability << " doesn't exist.");
                return;
            }

            pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10); // create publisher object, defining topic name and queue size
            timer_ = create_wall_timer(1s, std::bind(&simpleqospublisher::timerCallback, this)); // create a timer to trigger periodic callbacks every second

            RCLCPP_INFO(get_logger(), "Publishing at 1Hz"); // log info message to indicate publishing rate
        }
    private:
        unsigned int counter_; // counter variable to keep track of number of messages published
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; // publisher object, with its own type definition, smart pointer --> cleans up memeory automatically
        rclcpp::TimerBase::SharedPtr timer_; // timer object to trigger periodic callbacks
        rclcpp::QoS qos_profile_pub_;

        void timerCallback() { // callback function triggered by the timer
            auto message = std_msgs::msg::String(); // create a new string message object, auto keyword deduces the type automatically by compiler
            message.data = "Hello ROS2 - Counter " + std::to_string(counter_++); // Converts the number to text so it can be added to the string.
            pub_->publish(message); // publish the message using the publisher object
        }
};

int main(int argc, char* argv[]) { // standard C++ main function, it accepts command line arguments, int argc is the number of arguments, char* argv[] is an array of argument strings

    rclcpp::init(argc, argv); // master power switch, initializes the ROS2 system
    auto node = std::make_shared<simpleqospublisher>(); // create an instance of the simpleqospublisher node using a smart pointer, which means memory is managed automatically and lifecyle is handled
    rclcpp::spin(node); // spin function keeps the node alive and processing callbacks (like timerCallback) until shutdown
    rclcpp::shutdown(); // cleanly shutdown the ROS2 system, releasing resources

    return 0;
}
