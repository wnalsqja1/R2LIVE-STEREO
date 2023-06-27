#pragma once
#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/lidar_prior.hpp"

#include "factor/marginalization_factor.h"

#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"


#include "./fast_lio/fast_lio.hpp"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include "common_lib.h"
#include "tools_timer.hpp"

#include "vio_marginalization.hpp"

#include <../../camera_model/include/camodocal/camera_models/CameraFactory.h>
#include <../../camera_model/include/camodocal/camera_models/CataCamera.h>
#include <../../camera_model/include/camodocal/camera_models/PinholeCamera.h>

class Estimator
{
  public:
    Estimator();
    std::mutex m_process_;

    
    Fast_lio * m_fast_lio_instance = nullptr;
    void setParameter();
    void readIntrinsicParameter(vector<string>& calib_file); // added function 

    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header, const StatesGroup & lio_state = StatesGroup() );

    void solve_image_pose_1(const std_msgs::Header &header);

    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();

    void optimization();

    void slideWindow(); // check
    void slideWindowNew(); // check
    void slideWindowOld(); // check
    void optimization_LM();
    void vector2double();
    void double2vector();
    bool failureDetection(); // check


    int refine_vio_system(eigen_q q_extrinsic, vec_3 t_extrinsic);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj);

    void outliersRejection(set<int> &removeIndex);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    vector<camodocal::CameraPtr> m_camera;
    bool checkStructure();

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d m_gravity; // Vector3d g;

    MatrixXd Ap[2];
    VectorXd bp[2];

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d    Ps[(WINDOW_SIZE + 1)];
    Vector3d    Vs[(WINDOW_SIZE + 1)];
    Matrix3d    Rs[(WINDOW_SIZE + 1)];
    Vector3d    Bas[(WINDOW_SIZE + 1)];
    Vector3d    Bgs[(WINDOW_SIZE + 1)];
    StatesGroup m_lio_state_prediction_vec[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;


    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;


    double m_para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double m_para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double m_para_Feature[NUM_OF_F][SIZE_FEATURE];
    double m_para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double m_para_Retrive_Pose[SIZE_POSE];
    double m_para_Td[1][1];
    double m_para_Tr[1][1];

    int loop_window_index;

    vio_marginalization * m_vio_margin_ptr = nullptr;
    vector<double *> m_last_marginalization_parameter_blocks;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> m_all_image_frame;
    IntegrationBase *tmp_pre_integration;  

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;

    /*0511*/
    double prevTime, curTime;
    bool initFirstPoseFlag;
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector);
    bool start_initial_flag=false;

    /*0512*/
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
};
