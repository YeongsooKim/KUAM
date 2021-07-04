#include <kuam_aruco_tracking/target.h>
#include <iostream>

using namespace std;

Target::Target(int id, float marker_size_m, int filter_buf_size, int noise_cnt_th,
    float noise_dist_th_m, int estimating_method, bool compare_mode) :
    m_id(id),
    m_marker_size_m(marker_size_m),
    MAF_BUF_SIZE(filter_buf_size),
    NOISE_CNT_TH(noise_cnt_th),
    ESTIMATING_METHOD(estimating_method),
    NOISE_DIST_TH_M(noise_dist_th_m),
    COMPARE_MODE(compare_mode)
{
    m_is_detected = false;
    m_is_lost = true;
    m_is_init_maf = false;
    m_is_init_emaf = false;
    m_is_init_check_noise = false;

    m_pos_buf.resize(MAF_BUF_SIZE);
    m_emaf_w = 2.0 / ((double)MAF_BUF_SIZE + 1.0);

    m_noise_cnt = 0;

    m_state.resize((int)EstimatingMethod::ItemNum);
}

Target::~Target()
{}

void Target::UpdateDetection(bool is_detected)
{
    if (is_detected){
        m_is_detected = true;
        m_is_lost = false;
        m_last_detected_time = chrono::steady_clock::now();
    }
    else{
        auto cur_time = chrono::steady_clock::now();
        chrono::duration<double> duration = cur_time - m_last_detected_time;
        if (duration.count() > 0.5){
            m_is_detected = false;
            m_is_lost = false;
        }
        else{
            m_is_lost = true;
        }
    }
}

void Target::TargetStateEstimating(double p_x, double p_y, double p_z, double q_x, double q_y, double q_z, double q_w)
{
    Eigen::Vector3d pos;
    pos << p_x, p_y, p_z;

    Point p(p_x, p_y, p_z);
    if (m_is_detected && !m_is_lost){
        if (COMPARE_MODE){
            WithoutFilter(pos);
            MovingAvgFilter(pos);
            ExpMovingAvgFilter(pos);
        }
        else {
            switch (ESTIMATING_METHOD){
            case (int)EstimatingMethod::WOF:
                WithoutFilter(pos);

                break;
            case (int)EstimatingMethod::MAF:
                MovingAvgFilter(pos);

                break;
            case (int)EstimatingMethod::EMAF:
                ExpMovingAvgFilter(pos);

                break;
            default:
                break;
            }
        }
        
        Orientation orientation(q_x, q_y, q_z, q_w);
        m_orientation = orientation;
    }
    else if (m_is_detected && m_is_lost){
        if (COMPARE_MODE){
            WithoutFilter(m_state[(int)EstimatingMethod::WOF].prev_position);
            MovingAvgFilter(m_state[(int)EstimatingMethod::MAF].prev_position);
            ExpMovingAvgFilter(m_state[(int)EstimatingMethod::EMAF].prev_position);
        }
        else {
            switch (ESTIMATING_METHOD){
            case (int)EstimatingMethod::WOF:
                WithoutFilter(m_state[(int)EstimatingMethod::WOF].prev_position);

                break;
            case (int)EstimatingMethod::MAF:
                MovingAvgFilter(m_state[(int)EstimatingMethod::MAF].prev_position);

                break;
            case (int)EstimatingMethod::EMAF:
                ExpMovingAvgFilter(m_state[(int)EstimatingMethod::EMAF].prev_position);

                break;
            default:
                break;
            }
        }
    }
}

bool Target::CheckNoise()
{
    if (!m_is_detected){
        if (!m_is_init_check_noise){
            m_is_init_check_noise = true;
            m_noise_cnt = 0;
            m_last_noise_check_time = chrono::steady_clock::now();
        }
        
        auto cur_time = chrono::steady_clock::now();
        chrono::duration<double> duration = cur_time - m_last_noise_check_time;

        if (duration.count() < 0.5){
            m_noise_cnt++;
            return true;
        }
        else {
            m_is_init_check_noise = false;
            
            if (m_noise_cnt >= NOISE_CNT_TH) return false;
            else return true;
        }
    }
    else {
        return false;
    }
}

/****************************************************************************/
/*                              Util functions                              */
/****************************************************************************/

bool Target::WithoutFilter(const Eigen::Vector3d pos)
{
    m_state[(int)EstimatingMethod::WOF].position = pos;
    m_state[(int)EstimatingMethod::WOF].prev_position = pos;
}

bool Target::MovingAvgFilter(const Eigen::Vector3d pos)
{
    // Initialize
    if (!m_is_init_maf){
        for (auto& p : m_pos_buf){
            p = pos;
        }

        m_is_init_maf = true;
    }

    // Shift
    for (int i = 0; i < MAF_BUF_SIZE - 1; i++){
        m_pos_buf.at(i) = m_pos_buf.at(i+1);
    }
    m_pos_buf.at(MAF_BUF_SIZE - 1) = pos;

    // Summation
    Eigen::Vector3d sum;
    sum << 0, 0, 0;
    for (auto p : m_pos_buf){
        sum += p;
    }

    // Average
    m_state[(int)EstimatingMethod::MAF].position = sum/(double)MAF_BUF_SIZE;
    m_state[(int)EstimatingMethod::MAF].prev_position = pos;
}

bool Target::ExpMovingAvgFilter(const Eigen::Vector3d pos)
{
    // Initialize
    if (!m_is_init_emaf){
        m_state[(int)EstimatingMethod::EMAF].prev_position = pos;
        
        m_is_init_emaf = true;
    }

    // exponential moving average filter
    m_state[(int)EstimatingMethod::EMAF].position = m_is_init_emaf*pos + (1.0 - m_is_init_emaf)*m_state[(int)EstimatingMethod::EMAF].prev_position;
    m_state[(int)EstimatingMethod::EMAF].prev_position = pos;
}

float Target::Distance3D(Point point1, Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0) + pow(delta_z, 2.0));
    
    return distance_m;
}