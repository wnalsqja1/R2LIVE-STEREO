#include "parameters.h"
#include <opencv2/core/eigen.hpp>

std::string IMAGE1_TOPIC;
std::string IMAGE2_TOPIC; 
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;

Eigen::Matrix3d E;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int FLOW_BACK;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}
namespace {

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
}
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened()){
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["image0_topic"] >> IMAGE1_TOPIC;
    fsSettings["image1_topic"] >> IMAGE2_TOPIC; 
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];

    FLOW_BACK = fsSettings["flow_back"]; // added stereo system

    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "/fisheye_mask.jpg";

    /*adding stereo*/
    std::string cam0Calibfile, cam1Calibfile; 
    fsSettings["cam0_calib"] >> cam0Calibfile; 
    fsSettings["cam1_calib"] >> cam1Calibfile; 

    std::string cam0Path = VINS_FOLDER_PATH + "/" + cam0Calibfile; 
    std::string cam1Path = VINS_FOLDER_PATH + "/" + cam1Calibfile; 

    ROS_DEBUG("cam0Path: %s", cam0Path.c_str()); 
    ROS_DEBUG("cam1Path: %s", cam1Path.c_str());

    CAM_NAMES.push_back(cam0Path);
    CAM_NAMES.push_back(cam1Path);

    WINDOW_SIZE = 10;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 10;

    fsSettings.release();
    ROS_INFO("parameters.cpp :: success to load ROS VINS parameters");
}