#ifndef __FREQUENCY_DEGREE_H__
#define __FREQUENCY_DEGREE_H__

#include <cmath>
#include <complex>
#include <vector>

using namespace std;

class FrequencyDegree
{
public:
    FrequencyDegree() = default;
    FrequencyDegree(int id, int dft_buf_size, int frequency_degree_buf_size);
    virtual ~FrequencyDegree();

    void CalEstimatedFreqDegree(double height);
    void Init(int dft_integral_start_point, double frequency_degree_threshold, double difference_threshold_m, double height);
    
    static int valid_id;

private:
    // Const
    const int ID;
    const int DIFF_BUF_SIZE;
    const int FREQ_DEG_BUF_SIZE;

    // Flag
    bool m_is_valid;
    bool m_is_init_exp_maf;

    // Param
    int m_dft_integral_start_point_param;
    double m_freq_degree_th_param;
    double m_diff_th_m_param;

    // Variable
    double m_prev_height_m;
    vector<double> m_differences;
    vector<double> m_freq_degrees;
    double m_prev_freq_degree;
    double m_frequency_degree;
    double m_emaf_w;

private:
    vector<complex<double>> DFT();
    double Integral(const vector<complex<double>>& dft);
    void MovingAvgFilter(double freq_degree);
    void ExpMovingAvgFilter(double freq_degree);

    // Get Set
public:
    inline bool IsValid() { return m_is_valid; }
    inline int GetId() { return ID; }
    inline double GetValue() { return m_frequency_degree; }
};
#endif //  __FREQUENCY_DEGREE_H__