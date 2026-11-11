#ifndef __TF_EIGEN_MANAGER_H__
#define __TF_EIGEN_MANAGER_H__
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class Pose2D {
    public:
        Pose2D(): x(0.0), y(0.0), theta(0.0) {}
        Pose2D(double x, double y, double theta): x(x), y(y), theta(theta) {}
        double x, y, theta;
        friend std::ostream& operator<<(std::ostream& o, Pose2D const& pose) {
            o << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")";
            return o;
        }
        bool operator==(const Pose2D& other) {
            if(x == other.x && y == other.y && theta == other.theta) return true;
            else return false;
        }
};

class Pose3D {
    public:
        Pose3D(): x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
        Pose3D(double x, double y, double z, double roll, double pitch, double yaw)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
        double x, y, z, roll, pitch, yaw;
        friend std::ostream& operator<<(std::ostream& o, Pose3D const& pose) {
            o << "(" << pose.x << ", " << pose.y << ", " << pose.z << ", " << pose.roll << ", " << pose.pitch << ", " << pose.yaw << ")";
            return o;
        }
        bool operator==(const Pose3D& other) {
            if(x == other.x && y == other.y && z == other.z &&
               roll == other.roll && pitch == other.pitch && yaw == other.yaw) return true;
            else return false;
        }
};

class TFEigenManager {
    public:
        TFEigenManager() {}

        // Initialize by parameter Initialize by parameter 2D Pose(x,y,theta)
        TFEigenManager(double x, double y, double yaw, const std::string& frame_id, const std::string& child_frame_id)
        : t_(x, y, 0.0), roll_(0), pitch_(0), yaw_(yaw) {
            Eigen::Quaterniond q = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
        }

        // Initialize by parameter Pose2D class
        TFEigenManager(Pose2D pose, const std::string& frame_id, const std::string& child_frame_id)
        : t_(pose.x, pose.y, 0.0), roll_(0), pitch_(0), yaw_(pose.theta) {
            Eigen::Quaterniond q = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
        }

        // Initialize by parameter Pose3D class
        TFEigenManager(Pose3D pose, const std::string& frame_id, const std::string& child_frame_id)
        : t_(pose.x, pose.y, pose.z), roll_(pose.roll), pitch_(pose.pitch), yaw_(pose.yaw) {
            Eigen::Quaterniond q = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
        }

        // Initialize by parameter PoseStamped msg type
        TFEigenManager(geometry_msgs::PoseStamped pose, const std::string& child_frame_id)
        : q_(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z),
          t_(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) {
            tf_.header = pose.header;
            tf_.child_frame_id = child_frame_id;
            // tf_.transform.translation.x = pose.pose.position.x;
            // tf_.transform.translation.y = pose.pose.position.y;
            // tf_.transform.translation.z = pose.pose.position.z;
            // tf_.transform.rotation.x = pose.pose.orientation.x;
            // tf_.transform.rotation.y = pose.pose.orientation.y;
            // tf_.transform.rotation.z = pose.pose.orientation.z;
            // tf_.transform.rotation.w = pose.pose.orientation.w;
            tf2::Matrix3x3(tf2::Quaternion(q_.x(), q_.y(), q_.z(), q_.w())).getRPY(roll_, pitch_, yaw_);
        }

        // Initialize by parameter TransformStamped msg type
        TFEigenManager(geometry_msgs::TransformStamped tf)
        : q_(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z),
          t_(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z), tf_(tf) {
            tf2::Matrix3x3(tf2::Quaternion(q_.x(), q_.y(), q_.z(), q_.w())).getRPY(roll_, pitch_, yaw_);
        }

        // Initialize by parameter Quaterniond, Vector3d type in Eigen Library
        TFEigenManager(Eigen::Quaterniond q, Eigen::Vector3d t, const std::string& frame_id, const std::string& child_frame_id)
        : q_(q), t_(t) {
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
            tf2::Matrix3x3(tf2::Quaternion(q_.x(), q_.y(), q_.z(), q_.w())).getRPY(roll_, pitch_, yaw_);
        }

        // Initialize by parameter 4x4 transform matrix
        TFEigenManager(Eigen::Matrix4d rot_mat, const std::string& frame_id, const std::string& child_frame_id) {
            q_ = Eigen::Quaterniond(rot_mat.block<3,3>(0,0));
            t_ = rot_mat.block<3,1>(0,3);
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
            tf2::Matrix3x3(tf2::Quaternion(q_.x(), q_.y(), q_.z(), q_.w())).getRPY(roll_, pitch_, yaw_);
        }
        
        // Initialize by parameter euler angle and translation
        TFEigenManager(Eigen::Vector3d euler, Eigen::Vector3d t, const std::string& frame_id, const std::string& child_frame_id)
        : t_(t) {
            roll_ = euler(0); pitch_ = euler(1); yaw_ = euler(2);
            Eigen::Quaterniond q = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
            tf_.header.frame_id = frame_id;
            tf_.child_frame_id = child_frame_id;
        }
        
        TFEigenManager operator*(const TFEigenManager& other) const {
            Eigen::Quaterniond q_tmp = q_ * other.q_;
            Eigen::Vector3d t_tmp = q_ * other.t_ + t_;
            
            return TFEigenManager(q_tmp, t_tmp, std::string(tf_.header.frame_id), std::string(other.tf_.child_frame_id));
        }

        TFEigenManager inverse() {
            Eigen::Matrix4d rot_mat = toRotationMatrix().inverse();
            return TFEigenManager(rot_mat, tf_.child_frame_id, tf_.header.frame_id);
        }

        Eigen::Matrix4d toRotationMatrix() {
            Eigen::Matrix4d rot_mat;
            rot_mat << q_.toRotationMatrix(), t_,
                       Eigen::MatrixXd::Zero(1, 3), 1;
            return rot_mat;
        }

        void getEigen(Eigen::Quaterniond& q, Eigen::Vector3d& t) { q = q_; t = t_; }
        geometry_msgs::TransformStamped getTransformStamped(ros::Time time_stamp) {
            tf_.header.stamp = time_stamp;
            tf_.transform.translation.x = t_(0);
            tf_.transform.translation.y = t_(1);
            tf_.transform.translation.z = t_(2);
            q_.normalize();
            tf_.transform.rotation.x = q_.x();
            tf_.transform.rotation.y = q_.y();
            tf_.transform.rotation.z = q_.z();
            tf_.transform.rotation.w = q_.w();
            return tf_;
        }

        void setYaw(double yaw) {
            roll_ = 0.0; pitch_ = 0.0; yaw_ = yaw;
            Eigen::Quaterniond q = Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX())
                                   * Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
            q_ = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
        }

        double tx() { return t_(0); }
        double ty() { return t_(1); }
        double tz() { return t_(2); }
        double qx() { return q_.x(); }
        double qy() { return q_.y(); }
        double qz() { return q_.z(); }
        double qw() { return q_.w(); }
        std::string getFrameID() { return tf_.header.frame_id; }
        std::string getChildFrameID() { return tf_.child_frame_id; }
        double getRoll() { return roll_; }
        double getPitch() { return pitch_; }
        double getYaw() { return yaw_; }
        void getRPY(double& roll, double& pitch, double& yaw) { roll = roll_; pitch = pitch_; yaw = yaw_; }
        Pose2D getPose2D() { return Pose2D(t_(0), t_(1), yaw_); }

        void setFrameID(const std::string& frame_id) { tf_.header.frame_id = frame_id; }
        void setChildFrameID(const std::string& child_frame_id) { tf_.child_frame_id = child_frame_id; }

        friend std::ostream& operator<<(std::ostream& o, TFEigenManager const& tfem) {
            o << tfem.tf_.header.frame_id << "2" << tfem.tf_.child_frame_id << ": " << tfem.t_(0) << ", " << tfem.t_(1) << ", " << tfem.yaw_;
            return o;
        }
        
        typedef boost::shared_ptr<TFEigenManager> Ptr;
        geometry_msgs::TransformStamped tf_;
        geometry_msgs::PoseStamped pose_;
    
    private:
        Eigen::Quaterniond q_;
        Eigen::Vector3d t_;
        double roll_, pitch_, yaw_;
        
};

#endif