#include <ros/ros.h>
#include <kuam_aruco_tracking/frequency_degree.h>
#include <iostream>

using namespace std;

FrequencyDegree::FrequencyDegree(int id, int dft_buf_size, int frequency_degree_buf_size) :
    ID(id),
    DIFF_BUF_SIZE(dft_buf_size),
    FREQ_DEG_BUF_SIZE(frequency_degree_buf_size)
{}

FrequencyDegree::~FrequencyDegree()
{}

void FrequencyDegree::Init(int dft_integral_start_point, double frequency_degree_threshold, double difference_threshold_m, double height)
{
    m_dft_integral_start_point_param = dft_integral_start_point;
    m_freq_degree_th_param = frequency_degree_threshold;
    m_diff_th_m_param = difference_threshold_m;

    m_prev_height_m = height;
    m_is_valid = false;
    m_is_init_exp_maf = false;
}

void FrequencyDegree::CalEstimatedFreqDegree(double height)
{
    // Get difference between current height and previous height, and then update previous height
    ROS_INFO_THROTTLE(0.1, "## id: %d, CalEstimatedFreqDegree ##", ID);
    auto difference = abs(height - m_prev_height_m);
    if (difference > m_diff_th_m_param){
        m_is_valid = false;
        ROS_INFO_THROTTLE(0.1, "id: %d, is_valid = false, out of roi\n", ID);
        return;
    }
    m_prev_height_m = height;

    // Check buffer is full
    if (m_differences.size() < DIFF_BUF_SIZE){
        m_differences.push_back(difference);

        m_is_valid = false;
        ROS_INFO_THROTTLE(0.1, "id: %d, is_valid = false, buffer is not full\n", ID);
        return;
    }

    // Shift
    int i = 0;
    for ( ; i < DIFF_BUF_SIZE - 1; i++) m_differences[i] = m_differences[i+1];
    m_differences[i] = difference;

    // Discrete Fourier Transform
    auto dft = DFT();

    // Integral
    auto freq_degree = Integral(dft);

    // Filtering integral amplitude
    // MovingAvgFilter(freq_degree);
    ExpMovingAvgFilter(freq_degree);
}

void FrequencyDegree::MovingAvgFilter(double freq_degree)
{
    if (m_freq_degrees.size() < FREQ_DEG_BUF_SIZE){
        m_freq_degrees.push_back(freq_degree);
        m_prev_freq_degree = freq_degree;
        
        m_is_valid = false;

        return;
    }

    // Shift
    int i = 0;
    for ( ; i < FREQ_DEG_BUF_SIZE - 1; i++) m_freq_degrees[i] = m_freq_degrees[i+1];
    m_freq_degrees[i] = freq_degree;

    // Summation
    auto sum = 0;
    for (auto freq_degree : m_freq_degrees){
        sum += freq_degree;
    }

    // Calculate average and compare threshold
    auto avg_freq_degree = sum/FREQ_DEG_BUF_SIZE;
    m_frequency_degree = avg_freq_degree;

    if (avg_freq_degree > m_freq_degree_th_param){
        m_is_valid = false;
    }
    else{
        m_is_valid = true;
    }
}

void FrequencyDegree::ExpMovingAvgFilter(double freq_degree)
{
    ROS_INFO_THROTTLE(0.1, "### id: %d, ExpMovingAvgFilter ###", ID);
    // Integrals exponential moving average filter
    if (!m_is_init_exp_maf){
        m_prev_freq_degree = freq_degree;
        m_emaf_w = 2.0 / ((double)FREQ_DEG_BUF_SIZE + 1.0);
        m_is_init_exp_maf = true;
    }
    
    auto exp_moving_avg_integral_amp = m_emaf_w*freq_degree + (1.0 - m_emaf_w)*m_prev_freq_degree;
    m_prev_freq_degree = exp_moving_avg_integral_amp;
    m_frequency_degree = exp_moving_avg_integral_amp;

    if (exp_moving_avg_integral_amp > m_freq_degree_th_param){
        m_is_valid = false;
    }
    else{
        m_is_valid = true;
    }
    ROS_INFO_THROTTLE(0.1, "id: %d, is_valid = %d, threshold: %f, integral amp\n: %f", ID, m_is_valid, m_freq_degree_th_param, exp_moving_avg_integral_amp);
}

vector<complex<double>> FrequencyDegree::DFT()
{
    const int N = DIFF_BUF_SIZE;
    const int K = N;

    vector<complex<double>> output;
    for (int k = 0; k < K; k++){
        complex<double> sum(0, 0);
        for (int n = 0; n < N; n++){
            double real = 10*m_differences[n]*cos(((2*M_PI)/N)*k*n);
            double imag = 10*m_differences[n]*sin(((2*M_PI)/N)*k*n);
            complex<double> w (real, -imag);
            sum += w;
        }
        output.push_back(abs(sum));
    }

    return output;
}

double FrequencyDegree::Integral(const vector<complex<double>>& dft)
{
    double sum = 0;
    for (auto freq = m_dft_integral_start_point_param; freq < DIFF_BUF_SIZE/2; freq++){
        auto amp = abs(dft[freq]);
        sum += amp;
    }

    return sum;
}