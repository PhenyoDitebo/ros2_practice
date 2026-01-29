#include <rclcpp/rclcpp.hpp> // core ROS2 Library
#include <std_msgs/msg/string.hpp> // includes the string message type definition

using std::placeholders::_1; // for binding the callback function

class SimpleQoSSubscriber : public rclcpp::Node 
{
    public:
        SimpleQoSSubscriber() : Node("simple_subscriber"), qos_profile_sub_(10) // constructor 
        {
            declare_parameter<std::string>("reliability", "system_default");
            declare_parameter<std::string>("durability", "system_default");

            const auto reliability = get_parameter("reliability").as_string();
            const auto durability = get_parameter("durability").as_string();

            // Reliability, Quality of Connection, Transit.
            if (reliability == "best_effort") {
                qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
                RCLCPP_INFO(get_logger(), "[Reliability]: Best Effort."); // Best effort: attempt to deliver samples, but may lose them if the network is not robust.
            }
            else if (reliability == "reliable") {
                qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                RCLCPP_INFO(get_logger(), "[Reliability]: Reliable."); // Reliable: guarantee that samples are delivered, may retry multiple times.
            }
            else if (reliability == "system_default") {
                qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Reliability]: System Default."); // defaults back to system default.
            }
            else {
                RCLCPP_ERROR_STREAM(get_logger(), "Selected reliability QoS: " << reliability << " doesn't exist.");
                return;
            }

            // Durability, Memory of the Publisher, History.
            if (durability == "volatile") { // Volatile: no attempt is made to persist samples.
                qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
                RCLCPP_INFO(get_logger(), "[durability]: volatile");
            } 
            else if (durability == "transient_local") { // Transient local, the "latching" policy: the publisher becomes responsible for persisting samples for “late-joining” subscriptions.
                qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                RCLCPP_INFO(get_logger(), "[durability: transient_local]");
            }
            else if (durability == "system_default") {
                qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
                RCLCPP_INFO(get_logger(), "[Durability]: System Default."); // defaults back to system default.
            }
            else {
                RCLCPP_ERROR_STREAM(get_logger(), "Selected durability QoS: " << durability << " doesn't exist.");
                return;
            }

            sub_ = create_subscription<std_msgs::msg::String>("chatter", qos_profile_sub_, std::bind(&SimpleQoSSubscriber::msgCallback,this, _1)); // create subscription object, defining topic name, queue size, and binding the callback function
        }

    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; // subscription object with its own type definition, smart pointer --> cleans up memory automatically
        rclcpp::QoS qos_profile_sub_;

        void msgCallback(const std_msgs::msg::String &msg) const
        {
            RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str()); // log the received message to the console
        }
};


int main(int argc, char* argv[]) { // standard C++ main function, it accepts command line arguments, int argc is the number of arguments, char* argv[] is an array of argument strings
    rclcpp::init(argc, argv); // master power switch, initializes the ROS2 system

    auto node = std::make_shared<SimpleQoSSubscriber>(); // create an instance of the SimpleSubscriber node using a smart pointer
    rclcpp::spin(node); // spin function keeps the node alive and processing callbacks (like msgCallback) until shutdown
    rclcpp::shutdown(); // cleanly shutdown the ROS2 system, releasing resources

    return 0;
}
