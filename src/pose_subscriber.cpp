#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include <ncurses.h>
#include <thread>
#include <chrono>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/timer.hpp"

// Signal handler function
void signal_handler(int signal) {
    // Handle Ctrl+C signal
    if (signal == SIGINT) {
        // End ncurses mode
        endwin();
        // Shutdown the ROS 2 node
        rclcpp::shutdown();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ctrl+C pressed (SIGINT). Shutting down...");
        std::raise(SIGKILL);
    }
}

class PoseSubscriber : public rclcpp::Node {
public:
  using ToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleToPose = rclcpp_action::ClientGoalHandle<ToPose>;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  PoseSubscriber(): Node("pose_subscriber") {
    RCLCPP_INFO(get_logger(), "Started...");

    // Create a subscription to the /odom topic
    subscription_ = create_subscription<PoseStamped>(
        "/odom", 10, [this](const PoseStamped::SharedPtr msg) {
          this->listenerCallback(*msg);
        });

    // Create an action client for the /navigate_to_pose action
    this->action_client_ = rclcpp_action::create_client<ToPose>(this, "/navigate_to_pose");

    // Initialize ncurses for keyboard input
    initscr();
    timeout(0); // set non-blocking mode
    noecho();   // don't echo input to the screen

    // Create a timer that expires every 1.0 second to check for keyboard input
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      // Handle timer expiration (check for input)
      int ch = getch();
      if (ch != ERR) {
        handleKeyboardInput(ch);
      }
    });
  }

  // Send the goal to the action server
  void send_goal() {
    using namespace std::placeholders;
    RCLCPP_INFO(get_logger(), "Started to go to Pose #%zu", head_pose_index_);

    // Wait for the action server to become available
    while (!this->action_client_->wait_for_action_server(std::chrono::duration<double>(2.0))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    }
    
    RCLCPP_INFO(this->get_logger(), "Action server available.");

    // Create the goal message
    auto goal_msg = ToPose::Goal();
    // frame_id
    goal_msg.pose.header.frame_id = "map";
    
    goal_msg.pose.pose.position.x = saved_pose_list_[head_pose_index_].x;
    goal_msg.pose.pose.position.y = saved_pose_list_[head_pose_index_].y;
    goal_msg.pose.pose.position.z = saved_pose_list_[head_pose_index_].z;
    goal_msg.pose.pose.orientation.x = saved_orientation_list_[head_pose_index_].x;
    goal_msg.pose.pose.orientation.y = saved_orientation_list_[head_pose_index_].y;
    goal_msg.pose.pose.orientation.z = saved_orientation_list_[head_pose_index_].z;
    goal_msg.pose.pose.orientation.w = saved_orientation_list_[head_pose_index_].w;
    
    RCLCPP_INFO(this->get_logger(), "Sending goal to (%f, %f, %f)", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_msg.pose.pose.position.z);
    
    // Set up the goal options
    auto send_goal_options = rclcpp_action::Client<ToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&PoseSubscriber::result_callback, this, _1);
    
    // Send the goal to the action server
    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void handleKeyboardInput(int ch) {
    if (ch == 's') {
      // Save the current pose to the list
      savePose();
    }
    else if (ch == 'r') {
      if (saved_pose_list_.size() > 0) {
        // End ncurses mode
        endwin();
        
        // Unsubscribe from the topic and reset the subscription
        subscription_.reset();

        // Print the saved poses
        for (auto pose : saved_pose_list_) {
          RCLCPP_INFO(get_logger(), "Saved Pose: (%f, %f, %f)", pose.x, pose.y, pose.z);
        }
        
        // Start the first goal
        send_goal();
      }
      else {
        RCLCPP_INFO(get_logger(), "No saved poses to send.");
      }
    }
    else if (ch == 'q') {
        // End ncurses mode
        endwin();
        // Shut down the ROS 2 system
        rclcpp::shutdown();
    }
         
  }



private:
  void listenerCallback(const PoseStamped &msg) {
    // Save the current pose to the class member variable current_pose_

    current_pose_.x = msg.pose.position.x;
    current_pose_.y = msg.pose.position.y;
    current_pose_.z = msg.pose.position.z;
    current_orientation_.x = msg.pose.orientation.x;
    current_orientation_.y = msg.pose.orientation.y;
    current_orientation_.z = msg.pose.orientation.z;
    current_orientation_.w = msg.pose.orientation.w;
  }
 
  
  void savePose() {
    saved_pose_list_.push_back(current_pose_);
    saved_orientation_list_.push_back(current_orientation_);
    RCLCPP_INFO(get_logger(), "Pose saved: (%f, %f, %f)", saved_pose_list_.back().x, saved_pose_list_.back().y, saved_pose_list_.back().z);
  }

  // Callback function for the result of the action server
  void result_callback(const GoalHandleToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was successful");
        if(head_pose_index_ == 6){
          sleep(5.0);
          RCLCPP_INFO(this->get_logger(), "Goal was successful");
          head_pose_index_++;
          send_goal();
        }
        else if(head_pose_index_ == 8 && first_pose8_ == false){
          first_pose8_ = true;
          sleep(2.0);
          RCLCPP_INFO(this->get_logger(), "Goal was successful");
          head_pose_index_++;
          send_goal();
        }
        // If there is only one goal, the robot is now idle
        else if(saved_pose_list_.size() == 1) {
          RCLCPP_INFO(this->get_logger(), "Only one goal was saved. Robot is now idle.");
        }
        // If there are more goals, send the next one
        else if (head_pose_index_ < saved_pose_list_.size() - 1) {
          head_pose_index_++;
          send_goal();
        }
        // If there are no more goals and if size is more than 9, reset the index to 8   ~~~~ this is for just show
        else if(saved_pose_list_.size() > 9){
          RCLCPP_INFO(this->get_logger(), "All goals completed. Resetting head_pose_index_ to 8");
          head_pose_index_ = 8;
          send_goal();
        }
        // If there are no more goals, reset the index to 0
        else {
          RCLCPP_INFO(this->get_logger(), "All goals completed. Resetting head_pose_index_ to 0.");
          head_pose_index_ = 0;
          send_goal();
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", static_cast<int>(result.code));
        return;
    }
  }

private:
  // Struct to store the pose data (x, y, z)
  struct Pose {
    float x;
    float y;
    float z;
  };
  // Struct to store the orientation data (x, y, z, w)
  struct Orientation {
    float x;
    float y;
    float z;
    float w;
  };

  // Class member variables
  rclcpp::Subscription<PoseStamped>::SharedPtr subscription_;
  rclcpp_action::Client<ToPose>::SharedPtr action_client_;
  std::vector<Pose> saved_pose_list_;
  std::vector<Orientation> saved_orientation_list_;
  Pose current_pose_;
  Orientation current_orientation_;
  size_t head_pose_index_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_pose8_ = false;
};


int main(int argc, char *argv[]) {
  // Set up the signal handler for Ctrl+C
  std::signal(SIGINT, signal_handler);

  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  
  // Create a node 
  auto node = std::make_shared<PoseSubscriber>();

  
  rclcpp::spin(node);


  // End ncurses mode
  endwin();
  // Shut down the ROS 2 system
  rclcpp::shutdown();

  return 0;
}