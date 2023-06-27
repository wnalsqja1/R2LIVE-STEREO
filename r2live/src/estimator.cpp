#include "estimator.h"

extern Camera_Lidar_queue g_camera_lidar_queue;
extern MeasureGroup Measures;
extern StatesGroup g_lio_state;

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    m_process_.lock();

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        // tic[i] = READ_TIC[i];
        // ric[i] = READ_RIC[i];
    }

    f_manager.setRic(ric);

    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity(); 

    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();    

    td = TD;

    m_process_.unlock();
}

void Estimator::readIntrinsicParameter(vector<string>& calib_file)
{
    for(int i=0; i<calib_file.size(); i++){
        ROS_INFO("estimator.cpp: reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera); 
    }
}

void Estimator::clearState()
{
    m_process_.lock();

    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : m_all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    prevTime = -1;
    curTime = 0;
    first_imu = false;
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    m_all_image_frame.clear();

    td = TD;
    initFirstPoseFlag = true;



    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;

    tmp_pre_integration = nullptr;
    m_vio_margin_ptr = nullptr;

    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();    

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();

    m_process_.unlock();    
}


int Estimator::refine_vio_system(eigen_q q_extrinsic, vec_3 t_extrinsic)
{
    Eigen::Matrix<double, 3, 3> R_mat_e = q_extrinsic.toRotationMatrix();
    for (int win_idx = 0; win_idx <= WINDOW_SIZE; win_idx++)
    {
        Rs[win_idx] = Rs[win_idx] * R_mat_e;
        Ps[win_idx] = Ps[win_idx] + t_extrinsic;
        Vs[win_idx] = R_mat_e * Vs[win_idx];
    }

    for (int win_idx = 0; win_idx <= WINDOW_SIZE; win_idx++)
    {
        pre_integrations[win_idx]->delta_p = R_mat_e * pre_integrations[win_idx]->delta_p;
    }

    back_R0 = back_R0 * R_mat_e;
    last_R0 = last_R0 * R_mat_e;
    last_R = last_R * R_mat_e;

    back_P0 = back_P0 + t_extrinsic;
    last_P = last_P + t_extrinsic;
    last_P0 = last_P0 + t_extrinsic;
    return 1;
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (frame_count != 0)
    {
        // cout << "Preintergration count  = " << frame_count << endl;
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - m_gravity;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - m_gravity;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header, const StatesGroup & state_prior)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    imageframe.m_state_prior = state_prior;
    m_all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
}


/* Fusion with VINS_FUSION ( using ) */
void Estimator::solve_image_pose_1(const std_msgs::Header &header)
{
    if (solver_flag == INITIAL)
    {
        f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(Ps, tic, ric);

        if (frame_count == WINDOW_SIZE)
        {
            g_camera_lidar_queue.m_visual_init_time = 3e88; // Set state as uninitialized.
            g_camera_lidar_queue.m_camera_imu_td = 0;       

            map<double, ImageFrame>::iterator frame_it;
            int i = 0;
            for (frame_it = m_all_image_frame.begin(); frame_it != m_all_image_frame.end(); frame_it++)
            {
                frame_it->second.R = Rs[i];
                frame_it->second.T = Ps[i];
                i++;
            }

            solveGyroscopeBias(m_all_image_frame, Bgs);

            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
            }

            optimization();
            solver_flag=NON_LINEAR;
            slideWindow();

            ROS_INFO("estimation.cpp :: Initialization finish!");

            last_R = Rs[WINDOW_SIZE];
            last_P = Ps[WINDOW_SIZE];
            last_R0 = Rs[0];
            last_P0 = Ps[0];
        }

        if(frame_count < WINDOW_SIZE){

            frame_count++;
            int prev_frame = frame_count - 1;
            
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];            
        }
    }
    else
    {
        TicToc t_solve;
        f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(Ps, Rs, tic, ric);
        optimization();

        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        /* except */
        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            g_camera_lidar_queue.m_visual_init_time = 3e88; // Set state as uninitialized.
            g_camera_lidar_queue.m_camera_imu_td = 0;
     
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();        
        ROS_DEBUG("slidingwindow + removeFailures marginalization costs: %fms", t_margin.toc());

        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}


void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        m_para_Pose[i][0] = Ps[i].x();
        m_para_Pose[i][1] = Ps[i].y();
        m_para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        m_para_Pose[i][3] = q.x();
        m_para_Pose[i][4] = q.y();
        m_para_Pose[i][5] = q.z();
        m_para_Pose[i][6] = q.w();

        m_para_SpeedBias[i][0] = Vs[i].x();
        m_para_SpeedBias[i][1] = Vs[i].y();
        m_para_SpeedBias[i][2] = Vs[i].z();

        m_para_SpeedBias[i][3] = Bas[i].x();
        m_para_SpeedBias[i][4] = Bas[i].y();
        m_para_SpeedBias[i][5] = Bas[i].z();

        m_para_SpeedBias[i][6] = Bgs[i].x();
        m_para_SpeedBias[i][7] = Bgs[i].y();
        m_para_SpeedBias[i][8] = Bgs[i].z();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        m_para_Ex_Pose[i][0] = tic[i].x();
        m_para_Ex_Pose[i][1] = tic[i].y();
        m_para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        m_para_Ex_Pose[i][3] = q.x();
        m_para_Ex_Pose[i][4] = q.y();
        m_para_Ex_Pose[i][5] = q.z();
        m_para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        m_para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        m_para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(m_para_Pose[0][6],
                                                      m_para_Pose[0][3],
                                                      m_para_Pose[0][4],
                                                      m_para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(m_para_Pose[0][6],
                                       m_para_Pose[0][3],
                                       m_para_Pose[0][4],
                                       m_para_Pose[0][5]).toRotationMatrix().transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(m_para_Pose[i][6], m_para_Pose[i][3], m_para_Pose[i][4], m_para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = rot_diff * Vector3d(m_para_Pose[i][0] - m_para_Pose[0][0],
                                m_para_Pose[i][1] - m_para_Pose[0][1],
                                m_para_Pose[i][2] - m_para_Pose[0][2]) + origin_P0;

        Vs[i] = rot_diff * Vector3d(m_para_SpeedBias[i][0],
                                    m_para_SpeedBias[i][1],
                                    m_para_SpeedBias[i][2]);

        Bas[i] = Vector3d(m_para_SpeedBias[i][3],
                          m_para_SpeedBias[i][4],
                          m_para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(m_para_SpeedBias[i][6],
                          m_para_SpeedBias[i][7],
                          m_para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(m_para_Ex_Pose[i][0],
                          m_para_Ex_Pose[i][1],
                          m_para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(m_para_Ex_Pose[i][6],
                             m_para_Ex_Pose[i][3],
                             m_para_Ex_Pose[i][4],
                             m_para_Ex_Pose[i][5]).toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = m_para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = m_para_Td[0][0];

    // relative info between two loop frame
    if(relocalization_info)
    { 
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - m_para_Pose[0][0],
                                     relo_Pose[1] - m_para_Pose[0][1],
                                     relo_Pose[2] - m_para_Pose[0][2]) + origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());

        relocalization_info = 0;    
    }
}

bool Estimator::failureDetection()
{    
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        // return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        // return true;
    }

    // if( (tic[0] - TIC[0]).norm() > 1.0 )
    // {
    //     ROS_INFO(" big Extrinsic T error %f", (tic[0] - TIC[0]).norm());
    //     return true;
    // }

    // double ext_R_diff = Eigen::Quaterniond(ric[0]).angularDistance(Eigen::Quaterniond(RIC[0])) * 57.3;
    // if (ext_R_diff > 20)
    // {
    //     ROS_INFO(" big Extrinsic R error %f", ext_R_diff);
    //     return true;
    // }

    /*if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }*/
    Vector3d tmp_P = Ps[WINDOW_SIZE];

    // if ((tmp_P - last_P).norm() > 5.0)
    // {
    //     ROS_INFO(" big translation");
    //     return true;
    // }

    // if (abs(tmp_P.z() - last_P.z()) > 1.0)
    // {
    //     ROS_INFO(" big z translation");
    //     return true; 
    // }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    /* non keyframe */
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
                std::swap( m_lio_state_prediction_vec[i] ,  m_lio_state_prediction_vec[i + 1] ) ; //swap lio state
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
            m_lio_state_prediction_vec[WINDOW_SIZE] = m_lio_state_prediction_vec[WINDOW_SIZE-1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = m_all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;
 
                for (map<double, ImageFrame>::iterator it = m_all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                m_all_image_frame.erase(m_all_image_frame.begin(), it_0);
                m_all_image_frame.erase(t_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Vs[frame_count - 1] = Vs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];
            m_lio_state_prediction_vec[frame_count - 1] = m_lio_state_prediction_vec[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = m_para_Pose[i][j];
        }
    }
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    //return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index ++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;             
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                    Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                    depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if(it_per_frame.is_stereo)
            {
                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {            
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0], 
                                                        Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                        depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    //printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }       
            }
        }
        double ave_err = err / errCnt;
        if(ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                 Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                 double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

/* From VINS Fusion Optimization + LiDAR Prior constraints */
void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::HuberLoss(1.0);

    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(m_para_Pose[i], SIZE_POSE, local_parameterization); 
        problem.AddParameterBlock(m_para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(m_para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(m_para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }

    problem.AddParameterBlock(m_para_Td[0], 1);
    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(m_para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }


    /* VI-B: IMU measurement Residual  */
    for (int i = 0; i < frame_count; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;

        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]); // <15, 7, 9, 7, 9>
        problem.AddResidualBlock(imu_factor, NULL, m_para_Pose[i], m_para_SpeedBias[i], m_para_Pose[j], m_para_SpeedBias[j]);
    }

    /*  Add LiDAR prior ( fixed ) */ 
    for(int i=0;i<=frame_count;i++){ 
        LiDAR_prior_factor_15* lidar_prior_factor = new LiDAR_prior_factor_15(&m_lio_state_prediction_vec[i]); // public ceres::SizedCostFunction<15, 7, 9>
        // L_prior_factor l_prior_factor;LiDAR_prior_factor_15
        problem.AddResidualBlock(lidar_prior_factor, NULL, m_para_Pose[i], m_para_SpeedBias[i]);
    }    

    /* VI-C: IMU measurement Residual  */
    int f_m_cnt = 0;
    int feature_index = -1;

    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;


        /*VINS_FUSION STEREO factor */
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, m_para_Pose[imu_i], m_para_Pose[imu_j], m_para_Ex_Pose[0], m_para_Feature[feature_index], m_para_Td[0]);
            }

            if(it_per_frame.is_stereo)
            {                
                Vector3d pts_j_right = it_per_frame.pointRight;
                if(imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, m_para_Pose[imu_i], m_para_Pose[imu_j], m_para_Ex_Pose[0], m_para_Ex_Pose[1], m_para_Feature[feature_index], m_para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                 it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, m_para_Ex_Pose[0], m_para_Ex_Pose[1], m_para_Feature[feature_index], m_para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }


    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.num_threads = 2;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl;

    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("Ceres solver costs: %f", t_solver.toc());

    double2vector();

    if(frame_count < WINDOW_SIZE)
        return;

    /* Marginalization */
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid )
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == m_para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == m_para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{m_para_Pose[0], m_para_SpeedBias[0], m_para_Pose[1], m_para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
        
        // { 
        //     /*<15, 7, 9>*/
        //     LiDAR_prior_factor_15* lidar_prior_factor = new LiDAR_prior_factor_15(&m_lio_state_prediction_vec[0]); // public ceres::SizedCostFunction<15, 7, 9>
        //     // L_prior_factor l_prior_factor;LiDAR_prior_factor_15
        //     // problem.AddResidualBlock(lidar_prior_factor, NULL, m_para_Pose[i], m_para_SpeedBias[i]);
        //     ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(lidar_prior_factor, loss_function,
        //                                                                     vector<double *>{m_para_Pose[0], m_para_Pose[0]},
        //                                                                     vector<int>{0, 1});
        //     marginalization_info->addResidualBlockInfo(residual_block_info);            
        // }  

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if(imu_i != imu_j)
                    {
                        /*<2, 7, 7,// 7, 1, 1>*/
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{m_para_Pose[imu_i], m_para_Pose[imu_j], m_para_Ex_Pose[0], m_para_Feature[feature_index], m_para_Td[0]},
                                                                                        vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if(it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if(imu_i != imu_j)
                        {
                            /*<2, 7, 7,// 7, 7, 1, 1>*/
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{m_para_Pose[imu_i], m_para_Pose[imu_j], m_para_Ex_Pose[0], m_para_Ex_Pose[1], m_para_Feature[feature_index], m_para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            /*<2, 7, 7,// 1, 1>*/
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{m_para_Ex_Pose[0], m_para_Ex_Pose[1], m_para_Feature[feature_index], m_para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(m_para_Pose[i])] = m_para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(m_para_SpeedBias[i])] = m_para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(m_para_Ex_Pose[i])] = m_para_Ex_Pose[i];
        addr_shift[reinterpret_cast<long>(m_para_Td[0])] = m_para_Td[0];

        // if (ESTIMATE_TD)
        // {
        //     addr_shift[reinterpret_cast<long>(m_para_Td[0])] = m_para_Td[0];
        // }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), m_para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != m_para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == m_para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(m_para_Pose[i])] = m_para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(m_para_SpeedBias[i])] = m_para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(m_para_Pose[i])] = m_para_Pose[i];
                    addr_shift[reinterpret_cast<long>(m_para_SpeedBias[i])] = m_para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(m_para_Ex_Pose[i])] = m_para_Ex_Pose[i];
            // if (ESTIMATE_TD)
            // {
            //     addr_shift[reinterpret_cast<long>(m_para_Td[0])] = m_para_Td[0];
            // }
            addr_shift[reinterpret_cast<long>(m_para_Td[0])] = m_para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

/*'23.05.11.*/
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    //return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc); // gyro vector to Rotation 
    double yaw = Utility::R2ypr(R0).x(); // Rotation to Yaw Roll Pitch 
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // Yaw Roll Pitch to
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
    cout << "init P0 " << endl << Ps[0] << endl;
}


bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if(accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}