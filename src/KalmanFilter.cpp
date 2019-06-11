/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file KalmanFilter.h
 * Simple Kalman Filter for variable gain low-passing
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 *
 */

#include "KalmanFilter.h"
#include <iostream>

#define SIGMA_A_DOT		1.0
#define SIGMA_P0_DOT	0.01
#define SIGMA_K_DOT		1e-9

#define GRAVITY_CSTE 	9.81

extern Eigen::IOFormat OctaveFmt;

KalmanFilter::KalmanFilter(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> x0, Eigen::Matrix<double, 6, 6, Eigen::DontAlign> p0)
{
	_x_tilde = x0;
	_p_tilde = p0;
	_x_hat = _x_tilde;
	_p_hat = _p_tilde;

	_F = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Zero();
	_F(0, 1) = 1.0;
	_F(1, 2) = 1.0;

	_G = Eigen::Matrix<double, 6, 3, Eigen::DontAlign>::Zero();
	_G(2, 0) = 1.0;
	_G(4, 1) = 1.0;
	_G(5, 2) = 1.0;

	Eigen::DiagonalMatrix<double, 3> Q(pow(SIGMA_A_DOT, 2), pow(SIGMA_K_DOT, 2), pow(SIGMA_P0_DOT, 2));
	_Q = Q;

	_last_predict = std::chrono::steady_clock::now();
}

Eigen::Matrix<double, 12, 12, Eigen::DontAlign> KalmanFilter::compute_matrix_B(float dt)
{
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> GQG = _G * _Q * _G.transpose();

	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> A = Eigen::Matrix<double, 12, 12, Eigen::DontAlign>::Zero();

	for(int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++) {
			A(i,j) = (-_F)(i, j);
			A(i + 6,j + 6) = (_F.transpose())(i, j);
			A(i,j + 6) = GQG(i, j);
		}
	}

	A = (A*dt).exp();

	return A;
}

Eigen::Matrix<double, 6, 6, Eigen::DontAlign> KalmanFilter::compute_phi(Eigen::Matrix<double, 12, 12, Eigen::DontAlign> B)
{
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> phi = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Zero();

	for(int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++) {
			phi(i, j) = B(i + 6, j + 6);
		}
	}

	return phi.transpose();
}

Eigen::Matrix<double, 6, 6, Eigen::DontAlign> KalmanFilter::compute_q(
		Eigen::Matrix<double, 12, 12, Eigen::DontAlign> B,
		Eigen::Matrix<double, 6, 6, Eigen::DontAlign> phi)
{
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> B12 = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Zero();

	for(int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++) {
			B12(i, j) = B(i, j + 6);
		}
	}
	return phi * B12;
}

void KalmanFilter::predict()
{
	std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
	double dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_predict).count() / 1000.0;
	//std::cout << "dt = " << dt << " - Predict : " << std::endl;

	Eigen::Matrix<double, 12, 12, Eigen::DontAlign> B = compute_matrix_B(dt);
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> phi = compute_phi(B);
	Eigen::Matrix<double, 6, 6, Eigen::DontAlign> Qw = compute_q(B, phi);

	_x_tilde = phi * _x_hat;
	_p_tilde = phi * _p_hat * phi.transpose() + Qw;

	//std::cout << _p_tilde.format(OctaveFmt) <<  std::endl << std::endl;

	_last_predict = now;
}

void KalmanFilter::update_gps(double meas, double measUnc) {
	predict();
	
	Eigen::MatrixXd H(1, 6);
	H(0) = 1.0;
	H(1) = 0.0;
	H(2) = 0.0;
	H(3) = 0.0;
	H(4) = 0.0;
	H(5) = 0.0;

	Eigen::Matrix<double, 1, 1, Eigen::DontAlign> temp = H * _p_tilde * H.transpose() + Eigen::Matrix<double, 1, 1, Eigen::DontAlign>(measUnc);
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> K = _p_tilde * H.transpose() * temp.inverse();

	_x_hat = _x_tilde + K * (Eigen::Matrix<double, 1, 1, Eigen::DontAlign>(meas) - H * _x_tilde);
	_p_hat = (Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Identity() - K * H) * _p_tilde;

	//std::cout << "Measure = " << meas << "m" << std::endl;
	//std::cout << "X tilde : " << std::endl << _x_tilde.format(OctaveFmt) << std::endl;
	//std::cout << "X hat : " << std::endl << _x_hat.format(OctaveFmt) << std::endl;
	////std::cout << "H : " << std::endl << H.format(OctaveFmt) << std::endl;
	//std::cout << "K : " << std::endl << K.format(OctaveFmt) << std::endl;
}

void KalmanFilter::update_baro(double meas, double measUnc) {
	predict();

	Eigen::MatrixXd H(1, 6);

	H(0) = -_x_tilde(4) * GRAVITY_CSTE * _x_tilde(5) * exp(-_x_tilde(4) * GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)));
	H(1) = 0.0;
	H(2) = 0.0;
	H(3) = _x_tilde(4) * GRAVITY_CSTE * _x_tilde(5) * exp(-_x_tilde(4) * GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)));
	H(4) = -GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)) * exp(-_x_tilde(4) * GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)));
	H(5) = exp(-_x_tilde(4) * GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)));

	double hx = _x_tilde(5) * exp(-_x_tilde(4) * GRAVITY_CSTE * (_x_tilde(0) - _x_tilde(3)));

	Eigen::Matrix<double, 1, 1, Eigen::DontAlign> temp = H * _p_tilde * H.transpose() + Eigen::Matrix<double, 1, 1, Eigen::DontAlign>(measUnc);
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> K = _p_tilde * H.transpose() * temp.inverse();

	_x_hat = _x_tilde + K * (meas - hx);
	_p_hat = (Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Identity() - K * H) * _p_tilde;

	//std::cout << "Measure = " << meas << "pa, estimate = " << hx << "pa" << std::endl;
	//std::cout << "Umvert : " << (H * _p_tilde * H.transpose()) << std::endl;
	//std::cout << "X tilde : " << std::endl << _x_tilde.format(OctaveFmt) << std::endl;
	//std::cout << "X hat : " << std::endl << _x_hat.format(OctaveFmt) << std::endl;
	//std::cout << "H : " << std::endl << H.format(OctaveFmt) << std::endl;
	//std::cout << "K : " << std::endl << K.format(OctaveFmt) << std::endl;
}

void KalmanFilter::getAltitude(double &altitude)
{
	altitude = _x_hat(0);
}

void KalmanFilter::getCovariance(Eigen::Matrix<double, 6, 6, Eigen::DontAlign> &covariance)
{
	covariance = _p_hat;
}

void KalmanFilter::getState(Eigen::Matrix<double, 6, 1, Eigen::DontAlign> &state)
{
	state = _x_hat;
}
