#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <chrono>

//#include <mathlib/mathlib.h>

class KalmanFilter
{
public:
	/**
	 * Constructor, initialize state
	 */
	KalmanFilter(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> x0, Eigen::Matrix<double, 6, 6, Eigen::DontAlign> p0);

	/**
	 * Default desctructor
	 */
	virtual ~KalmanFilter() {};
	
	void predict();
	
	void update_gps(double meas, double measUnc);
	void update_baro(double meas, double measUnc);
	
	void getAltitude(double &altitude);
	void getCovariance(Eigen::Matrix<double, 6, 6, Eigen::DontAlign> &covariance);
	void getState(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &state);

private:
	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> compute_matrix_B(float dt);
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> compute_phi(Eigen::Matrix<double, 12, 12, Eigen::DontAlign> B);
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> compute_q(Eigen::Matrix<double, 12, 12, Eigen::DontAlign> B,
															Eigen::Matrix<double, 6, 6, Eigen::DontAlign> phi);

	std::chrono::time_point<std::chrono::steady_clock> _last_predict;

	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> _x_tilde;
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> _p_tilde;

	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> _x_hat;
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> _p_hat;

	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> _F;

	Eigen::Matrix<double, 6, 3, Eigen::DontAlign> _G;
	Eigen::Matrix<double, 3, 3, Eigen::DontAlign> _Q;
};
