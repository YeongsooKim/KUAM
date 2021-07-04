#ifndef __TARGET_H__
#define __TARGET_H__

#include <Eigen/Dense>
#include <vector>
#include <chrono>

using namespace std;

enum class EstimatingMethod : int
{
    WOF,    // Without filter
    MAF,    // Moving average filter
    EMAF,   // Exponential moving average filter
    // KF,     // Kalman filter

    ItemNum
};

class Target
{
    // Target without filter
    struct State{
        Eigen::Vector3d position;
        Eigen::Vector3d prev_position;
    };

    struct Point{
        Point() = default;
        Point(double x, double y, double z) : x(x), y(y), z(z) {}
        double x;
        double y;
        double z;
    };

    struct Orientation{
        Orientation() = default;
        Orientation(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
        double x;
        double y;
        double z;
        double w;
    };

public:
    Target() = default;
    Target(int id, float marker_size_m, int filter_buf_size, int noise_cnt_th,
        float noise_dist_th_m, int estimating_method, bool compare_mode);
    virtual ~Target();

    void UpdateDetection(bool is_detected);
    void TargetStateEstimating(double p_x=0.0, double p_y=0.0, double p_z=0.0, double q_x=0.0, double q_y=0.0, double q_z=0.0, double q_w=0.0);
    bool CheckNoise();

private:
    // Const
    const int MAF_BUF_SIZE;
    const int NOISE_CNT_TH;
    const int ESTIMATING_METHOD;
    const float NOISE_DIST_TH_M;
    const bool COMPARE_MODE;

    // Flag
    bool m_is_detected;
    bool m_is_lost;
    bool m_is_init_maf;
    bool m_is_init_emaf;
    bool m_is_init_check_noise;

    // Time
    chrono::_V2::steady_clock::time_point m_last_detected_time;
    chrono::_V2::steady_clock::time_point m_last_noise_check_time;

    int m_id;
    float m_marker_size_m;
    vector<State> m_state;
    Orientation m_orientation;

    vector<Eigen::Vector3d> m_pos_buf;
    double m_emaf_w;
    unsigned int m_noise_cnt;

private:
    bool WithoutFilter(const Eigen::Vector3d pos);
    bool MovingAvgFilter(const Eigen::Vector3d pos);
    bool ExpMovingAvgFilter(const Eigen::Vector3d pos);
    float Distance3D(Point point1, Point point2);

    // Get Set
public:
    inline int GetId() { return m_id; }
    inline bool GetIsDetected() { return m_is_detected; }
    inline double GetX(int method) { return m_state[method].position.x(); }
    inline double GetY(int method) { return m_state[method].position.y(); }
    inline double GetZ(int method) { return m_state[method].position.z(); }
    inline double GetQX() { return m_orientation.x; }
    inline double GetQY() { return m_orientation.y; }
    inline double GetQZ() { return m_orientation.z; }
    inline double GetQW() { return m_orientation.w; }
};
#endif //  __TARGET_H__