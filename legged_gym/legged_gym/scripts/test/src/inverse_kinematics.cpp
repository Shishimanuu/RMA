#include <eigen3/Eigen/Dense>
#include "shvan_msgs/Trajectory.h"
#include "shvan_msgs/JointState.h"
#include <ros/ros.h>


class IKControlNode {
public:
    IKControlNode() {

        ros::NodeHandle nh;
        pose_sub_ = nh.subscribe<shvan_msgs::Trajectory>("/svan/ee_ref", 10, &IKControlNode::poseCallback, this);
        joint_pub_ = nh.advertise<shvan_msgs::JointState>("/shvan/motor_command", 10);
        m_joint_command.position.clear();
        m_joint_command.velocity.clear();
        m_joint_command.effort.clear();

        m_ee_state_ref = Eigen::VectorXd::Zero(18);
        m_ee_vel_ref = Eigen::VectorXd::Zero(18);
        m_ee_acc_ref = Eigen::VectorXd::Zero(18);
        m_joint_state_ref = Eigen::VectorXd::Zero(18);
        m_joint_vel_ref = Eigen::VectorXd::Zero(18);;


        for (int i = 0; i < 12; i++)
        {
            m_joint_command.position.push_back(0);
            m_joint_command.velocity.push_back(0);
            m_joint_command.effort.push_back(0);
        }
    }

    void poseCallback(const shvan_msgs::Trajectory::ConstPtr& trajectory_msg) {
        
        this->get_ee_ref_cb(trajectory_msg);
        int m_gait_id = 1;
        m_joint_state_ref = IKControlNode::InverseKinematics(m_ee_state_ref);
        std::cout << "m joit state ref: " << m_joint_state_ref.transpose() << std::endl;
        for (int i = 0; i < 12; i++)
        {
            m_joint_command.gait_id = m_gait_id;
            m_joint_command.position[i] = m_joint_state_ref(6 + i);
        }
        joint_pub_.publish(m_joint_command);
    }

private:
    
    ros::Subscriber pose_sub_;
    ros::Publisher joint_pub_;
    Eigen::VectorXd m_ee_state_ref;
    Eigen::VectorXd m_ee_vel_ref;
    Eigen::VectorXd m_ee_acc_ref;
    Eigen::VectorXd m_joint_state_ref;
    Eigen::VectorXd m_joint_vel_ref;
    shvan_msgs::JointState m_joint_command;
    float l2 = 1;
    float l3 = 1;
    float l4 = 1;
    float b = 1;
    float h = 1;

    void get_ee_ref_cb(const shvan_msgs::Trajectory::ConstPtr &msg)
    {   
        int m_gait_id = msg->gait_id;
        for (int i = 0; i < 18; ++i)
            {
                m_ee_state_ref(i) = msg->position[i];
                m_ee_vel_ref(i) = msg->velocity[i];
                m_ee_acc_ref(i) = msg->acceleration[i];
            }
    }

    Eigen::VectorXd InverseKinematics(Eigen::VectorXd &ee_state)
    {
        Eigen::VectorXd js = Eigen::VectorXd::Zero(18);

        js.block<6, 1>(0, 0) = ee_state.block<6, 1>(0, 0);

        float bx = ee_state(0);
        float by = ee_state(1);
        float bz = ee_state(2);

        float ex = ee_state(3);
        float ey = ee_state(4);
        float ez = ee_state(5);

        float r00 = cos(ey) * cos(ez);
        float r01 = -cos(ey) * sin(ez);
        float r02 = sin(ey);
        float r10 = cos(ez) * sin(ex) * sin(ey) + cos(ex) * sin(ez);
        float r11 = cos(ex) * cos(ez) - sin(ex) * sin(ey) * sin(ez);
        float r12 = -cos(ey) * sin(ex);
        float r20 = -cos(ex) * cos(ez) * sin(ey) + sin(ex) * sin(ez);
        float r21 = cos(ez) * sin(ex) + cos(ex) * sin(ey) * sin(ez);
        float r22 = cos(ey) * cos(ex);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R << r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22;

        float x, y, z, x_, y_, z_, et, k1, k2, r, m1, m2, s;
        float c3, s3;
        float theta1, theta2, theta3;

        for (int i = 0; i < 4; i++)
        {
            x_ = ee_state(6 + 3 * i) - bx;
            y_ = ee_state(6 + 3 * i + 1) - by;
            z_ = ee_state(6 + 3 * i + 2) - bz;

            if (i == 0)
            {
                x = x_ - (r00 * h / 2 + r01 * (-b / 2));
                y = y_ - (r10 * h / 2 + r11 * (-b / 2));
                z = z_ - (r20 * h / 2 + r21 * (-b / 2));
            }
            else if (i == 1)
            {
                x = x_ - (r00 * h / 2 + r01 * (b / 2));
                y = y_ - (r10 * h / 2 + r11 * (b / 2));
                z = z_ - (r20 * h / 2 + r21 * (b / 2));
            }
            else if (i == 2)
            {
                x = x_ - (r00 * (-h / 2) + r01 * (b / 2));
                y = y_ - (r10 * (-h / 2) + r11 * (b / 2));
                z = z_ - (r20 * (-h / 2) + r21 * (b / 2));
            }
            else if (i == 3)
            {
                x = x_ - (r00 * (-h / 2) + r01 * (-b / 2));
                y = y_ - (r10 * (-h / 2) + r11 * (-b / 2));
                z = z_ - (r20 * (-h / 2) + r21 * (-b / 2));
            }

            std::cout << "Feet coordinates wrt hip: " << x << " " << y << " " << z << " " << std::endl;

            Eigen::Vector3d r_ = R.transpose() * Eigen::Vector3d(x, y, z);

            x = r_(0);
            y = r_(1);
            z = r_(2);

            et = y * y + z * z - l2 * l2;

            if (i == 0)
            {
                c3 = (x * x + et - l3 * l3 - l4 * l4) / (2 * l3 * l4);
                s3 = std::sqrt(1 - c3 * c3);
                theta3 = std::atan2(s3, c3);
                k1 = l3 + l4 * c3;
                k2 = l4 * s3;
                r = std::sqrt(k1 * k1 + k2 * k2);
                theta2 = atan2(x / r, std::sqrt(et) / r) - atan2(k2, k1);
                m1 = -1 * l2;
                m2 = l3 * std::cos(theta2) + l4 * std::cos(theta2 + theta3);
                s = std::sqrt(m1 * m1 + m2 * m2);
                theta1 = atan2(-z / s, y / s) - atan2(m2, m1);
                theta1 *= -1;
            }
            else if (i == 1)
            {
                c3 = (x * x + et - l3 * l3 - l4 * l4) / (2 * l3 * l4);
                s3 = -1 * std::sqrt(1 - c3 * c3);
                theta3 = std::atan2(s3, c3);
                k1 = l3 + l4 * c3;
                k2 = l4 * s3;
                r = std::sqrt(k1 * k1 + k2 * k2);
                theta2 = atan2(-x / r, std::sqrt(et) / r) - atan2(k2, k1);
                m1 = l2;
                m2 = -1 * (l3 * std::cos(theta2) + l4 * std::cos(theta2 + theta3));
                s = std::sqrt(m1 * m1 + m2 * m2);
                theta1 = atan2(z / s, y / s) - atan2(m2, m1);
            }
            else if (i == 2)
            {
                c3 = (x * x + et - l3 * l3 - l4 * l4) / (2 * l3 * l4);
                s3 = -1 * std::sqrt(1 - c3 * c3);
                theta3 = std::atan2(s3, c3);
                k1 = l3 + l4 * c3;
                k2 = l4 * s3;
                r = std::sqrt(k1 * k1 + k2 * k2);
                theta2 = atan2(-x / r, std::sqrt(et) / r) - atan2(k2, k1);
                m1 = l2;
                m2 = -1 * (l3 * std::cos(theta2) + l4 * std::cos(theta2 + theta3));
                s = std::sqrt(m1 * m1 + m2 * m2);
                theta1 = atan2(z / s, y / s) - atan2(m2, m1);
                theta1 *= -1;
            }
            else if (i == 3)
            {
                c3 = (x * x + et - l3 * l3 - l4 * l4) / (2 * l3 * l4);
                std::cout << "c3 "<< c3<<std::endl;
                s3 = std::sqrt(1 - c3 * c3);
                std::cout << "s3 "<< s3<<std::endl;
                theta3 = std::atan2(s3, c3);
                std::cout<<"Theta 3: "<<theta3<<std::endl;
                k1 = l3 + l4 * c3;
                k2 = l4 * s3;
                r = std::sqrt(k1 * k1 + k2 * k2);
                std::cout<<"r: "<<r<<std::endl;
                theta2 = atan2(x / r, std::sqrt(et) / r) - atan2(k2, k1);
                std::cout<<"Theta 2: "<<theta2<<std::endl;
                m1 = -l2;
                m2 = l3 * std::cos(theta2) + l4 * std::cos(theta2 + theta3);
                s = std::sqrt(m1 * m1 + m2 * m2);
                theta1 = atan2(-z / s, y / s) - atan2(m2, m1);
                std::cout<<"Theta 1: "<<theta1<<std::endl;
                
            }
            std::cout << "Theta 1 2 3: "<<theta1<<" "<<theta2<<" "<<theta3<<std::endl;
            js(6 + 3 * i) = theta1;
            js(6 + 3 * i + 1) = theta2;
            js(6 + 3 * i + 2) = theta3;
        }

        return js;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ik_control_node");
    IKControlNode ik_control_node;
    ros::spin();
    return 0;
}