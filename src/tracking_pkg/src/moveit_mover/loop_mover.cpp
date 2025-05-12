#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <thread> 
#include <std_msgs/msg/bool.hpp>


// gripper_mover true = gripper open
// gripper_mover false = gripper close
// gripper_zeroer true = sensing aktiv
// gripper_zeroer false = sensing inaktiv

class HandPositionFollower : public rclcpp::Node {
public:
    HandPositionFollower()
    : Node("moveit_mover") {
        // OMPL Parameter deklarieren
        this->declare_parameter("robot_description_planning.default_planner_config", "RRTConnect");
        this->declare_parameter("robot_description_planning.default_planner_config.settings", "RRTConnectkConfigDefault");
        // Optional: Timeout und Planungszeit konfigurieren
        this->declare_parameter("robot_description_planning.ompl_planning.timeout", 10.0);
        this->declare_parameter("robot_description_planning.ompl_planning.max_planning_attempts", 10);
        // Kinematik-Plugin deklarieren
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_timeout", 0.05);

        // Abonnieren des /hand_position Topics
        hand_position_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/hand_pose", 10,
        std::bind(&HandPositionFollower::handPositionCallback, this, std::placeholders::_1));
        // Gripper-Status abbonnieren
        gripper_done_sub_ = this->create_subscription<std_msgs::msg::Bool>("/gripper_done", 10,
        std::bind(&HandPositionFollower::gripperDoneCallback, this, std::placeholders::_1));
        // tool_selection abbonieren
        tool_selection_sub_ = this->create_subscription<std_msgs::msg::String>("/tool_selection", 10, 
        std::bind(&HandPositionFollower::toolSelectionCallback, this, std::placeholders::_1));

        // Gripper mover publisher initialisieren
        gripper_mover_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);
        // Gripper zeroer publisher initialisieren
        gripper_zeroer_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_zeroer", 10);

        RCLCPP_INFO(this->get_logger(), "Moveit Mover Node initialized.");
    }

    // Initialisiere die MoveGroupInterface
    void initializeMoveGroupInterface() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");
        move_group_->setEndEffectorLink("gripper_tip_link");
    
        // Konfiguriere MoveIt Parameter
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
        

        // Gripper öffnen (false)
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        publishGripperMover(true);
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Kurze Wartezeit für den Gripper
    }

private:
    geometry_msgs::msg::Point tool_position_;
    geometry_msgs::msg::Quaternion tool_orientation_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    geometry_msgs::msg::Pose hand_pose_;
    geometry_msgs::msg::Pose hand_pose_with_offset;
    bool hand_pose_received_ = false;
    bool waiting_for_hand_pose_ = false;
    bool waiting_for_gripper_done_ = false;

    void handPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!waiting_for_hand_pose_) {
            RCLCPP_INFO(this->get_logger(), "Ignoring hand pose – no pending command.");
            return;
        }
    
        hand_pose_ = *msg;
        waiting_for_hand_pose_ = false;
        hand_pose_received_ = true;
    
        RCLCPP_INFO(this->get_logger(), "Hand pose received, proceeding with handover.");
        performHandoverToHandPose();
    }
    

    void toolSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
    
        if (cmd == "1") { // Pinzette lang
            tool_position_.x = -0.2285;
            tool_position_.y = -0.19;
            tool_position_.z = 0.03;
            tool_orientation_.x = 1;
            tool_orientation_.y = 0;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_pose_with_offset.position.x = hand_pose_.position.x;
            hand_pose_with_offset.position.y = hand_pose_.position.y;
            hand_pose_with_offset.position.z = hand_pose_.position.z;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
        } else if (cmd == "2") { // Hammer
            tool_position_.x = -0.2035;
            tool_position_.y = -0.37;
            tool_position_.z = 0.03;
            tool_orientation_.x = 0;
            tool_orientation_.y = 1;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_pose_with_offset.position.x = hand_pose_.position.x;
            hand_pose_with_offset.position.y = hand_pose_.position.y;
            hand_pose_with_offset.position.z = hand_pose_.position.z;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
        } else if (cmd == "3") { // Schere lang
            tool_position_.x = -0.1785;
            tool_position_.y = -0.25;
            tool_position_.z = 0.032;
            tool_orientation_.x = 0;
            tool_orientation_.y = 1;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_pose_with_offset.position.x = hand_pose_.position.x;
            hand_pose_with_offset.position.y = hand_pose_.position.y;
            hand_pose_with_offset.position.z = hand_pose_.position.z;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
        } else if (cmd == "4") { // Schere kurz
            tool_position_.x = -0.1535;
            tool_position_.y = -0.36;
            tool_position_.z = 0.035;
            hand_pose_with_offset.position.x = hand_pose_.position.x;
            hand_pose_with_offset.position.y = hand_pose_.position.y;
            hand_pose_with_offset.position.z = hand_pose_.position.z;
            handover_orientation_.x = 0.0;
            handover_orientation_.y = 0.891;
            handover_orientation_.z = 0.0;
            handover_orientation_.w = 0.454;
        } else if (cmd == "5") { // Retraktor klein
            tool_position_.x = -0.14;
            tool_position_.y = -0.25;
            tool_position_.z = 0.026;
            tool_orientation_.x = 1;
            tool_orientation_.y = 0;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_pose_with_offset.position.x = hand_pose_.position.x;
            hand_pose_with_offset.position.y = hand_pose_.position.y;
            hand_pose_with_offset.position.z = hand_pose_.position.z;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", cmd.c_str());
            return;
        }
    
        // Nach Tastendruck Objekt aufnehmen
        moveToObjectPosition(
            tool_position_.x,
            tool_position_.y,
            tool_position_.z
        );
    
        // Jetzt auf Handposition warten
        waiting_for_hand_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Waiting for hand pose...");
    }


    void performHandoverToHandPose() {
        // Fahre über die Hand
        geometry_msgs::msg::Pose target_pose = hand_pose_with_offset;
        target_pose.orientation = handover_orientation_;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Moving above hand...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to above hand failed.");
            return;
        }
        
        // Öffne den Greifer über /gripper_zeroer
        RCLCPP_INFO(this->get_logger(), "Activating gripper sensing...");
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        publishGripperZeroer(true);
        waiting_for_gripper_done_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tool_selection_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_; 
    void gripperDoneCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Gripper opened, returning to home.");
            moveToHomePositionUsingJoints();
        }
    }   
    


    // Bewegung zur Home-Position über Joint-Winkel
    void moveToHomePositionUsingJoints() {
        // Definiere Joint-Winkel für die Home-Position
        std::vector<double> home_joint_positions = {
            0,    // Joint 1: 0°
            -2.486, // Joint 2: 90°
            1.227,    // Joint 3: 0°
            -1.294,    // Joint 4: 0°
            -M_PI_2,    // Joint 5: 0°
            0.0     // Joint 6: 0°
        };

        // Setze Joint-Winkel als Ziel
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(home_joint_positions);

        // Bewegung planen und ausführen
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to Home-Position (Joints) successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to Home-Position (Joints) failed.");
        }
    }


    void moveToObjectPosition(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
    
        // Fahre über Werkzeug
        publishGripperMover(true);
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;
        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += 0.1;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(lift_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }

        // Senke auf Werkzeug ab
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(object_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object failed.");
            return;
        }
    
        // Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    
        // Über Werkzeug fahren
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(lift_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Lifting object...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Lift motion failed.");
        }
        moveToHomePositionUsingJoints();
    }

    void publishGripperMover(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_mover_->publish(msg);
    }

    void publishGripperZeroer(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_zeroer_->publish(msg);
    }

    // Subscriber für /hand_position Topic
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_position_sub_;

    // MoveGroupInterface für Robotersteuerung
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Publisher für Gripper-Befehle
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_;
};

int main(int argc, char** argv) {
    // ROS 2 initialisieren
    rclcpp::init(argc, argv);

    // Node erstellen und MoveGroupInterface initialisieren
    auto node = std::make_shared<HandPositionFollower>();
    node->initializeMoveGroupInterface();

    // Node ausführen
    rclcpp::spin(node);

    // ROS 2 beenden
    rclcpp::shutdown();
    return 0;
}
