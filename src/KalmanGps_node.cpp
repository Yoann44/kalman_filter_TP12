#include "ros/ros.h"

#include <time.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>

#include "std_msgs/String.h"

#include "KalmanFilter.h"

#include <Eigen/Core>

#define AIR_MOLAR_MASS 	0.02897
#define GAZ_CSTE 		8.3144598

#define DEF_TEMP 		298.15

#define SIGMA_Z_GPS		5.0 	//[m]
#define SIGMA_Z_BARO	100.0		//[pa]

ros::Time _last_msg_time;

KalmanFilter* filter = 0;
ros::Publisher *_pubAltitude = nullptr;
ros::Publisher *_pubDebug = nullptr;

double pressure = 0;
bool useGPS = true;

Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");

void print_cov() {
	if(filter) {
		Eigen::Matrix<double, 6, 6, Eigen::DontAlign> cov;
		filter->getCovariance(cov);
		///std::cout << cov.format(OctaveFmt) <<  std::endl << std::endl;
	}
}

void publish_altitude(std_msgs::Header header) {
	//double altitude;
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> stateEstimate;

	//filter->getAltitude(altitude);
	filter->getState(stateEstimate);

	if(_pubAltitude) {
		geometry_msgs::Vector3Stamped data;

		data.header = header;
		data.vector.x = stateEstimate(2);
		data.vector.y = stateEstimate(1);
		data.vector.z = stateEstimate(0);

		_pubAltitude->publish(data);
	}

	if(_pubDebug) {
		geometry_msgs::Vector3Stamped data_debug;

		data_debug.header = header;
		data_debug.vector.x = stateEstimate(3);
		data_debug.vector.y = stateEstimate(4);
		data_debug.vector.z = stateEstimate(5);

		_pubDebug->publish(data_debug);
	}
}

void gps_update(const sensor_msgs::NavSatFix message) {
	if(filter) {
		ros::Time message_time = ros::Time(message.header.stamp.sec, message.header.stamp.nsec);

		if(useGPS && message_time > _last_msg_time) {

			filter->update_gps(message.altitude, SIGMA_Z_GPS);

			//std::cout << "GPS :" << std::endl;
			print_cov();
			
			publish_altitude(message.header);

			_last_msg_time = message_time;
		}
	}
	else if(pressure != 0) {
		// Init the filter
		Eigen::Matrix<double, 6, 1, Eigen::DontAlign> x0;
		x0(0) = message.altitude;
		x0(1) = 0.0;
		x0(2) = 0.0;
		x0(3) = message.altitude;
		x0(4) = AIR_MOLAR_MASS / (GAZ_CSTE * DEF_TEMP);
		x0(5) = pressure;

		Eigen::Matrix<double, 6, 6, Eigen::DontAlign> p0 = Eigen::Matrix<double, 6, 6, Eigen::DontAlign>::Zero();
		p0(0, 0) = pow(5.0, 2);
		p0(1, 1) = pow(0.5, 2);
		p0(2, 2) = pow(0.5, 2);
		p0(3, 3) = pow(5.0, 2);
		p0(4, 4) = pow(1e-6, 2);
		p0(5, 5) = pow(5.0, 2);
		
		filter = new KalmanFilter(x0, p0);

		ROS_INFO("Filter init !");

		Eigen::Matrix<double, 6, 6, Eigen::DontAlign> cov;
		filter->getCovariance(cov);
		//std::cout << cov.format(OctaveFmt) <<  std::endl << std::endl;
	}
}

void baro_update(const sensor_msgs::FluidPressure message) {
	if(filter) {
		ros::Time message_time = ros::Time(message.header.stamp.sec, message.header.stamp.nsec);
		if(message_time > _last_msg_time) {
			filter->update_baro(message.fluid_pressure, SIGMA_Z_BARO);

			//std::cout << "Baro :" << std::endl;
			print_cov();

			publish_altitude(message.header);

			_last_msg_time = message_time;
		}
	}
	else {
		_last_msg_time = ros::Time(message.header.stamp.sec, message.header.stamp.nsec);
		pressure = message.fluid_pressure;
	}
}

void use_gps(const std_msgs::Bool message) {
	useGPS = message.data;
	if(message.data) {
		ROS_INFO("Use GPS : enable");
	}
	else {
		ROS_INFO("Use GPS : disable");
	}
	
}

void show_debug(const std_msgs::Bool message) {
	if(message.data) {
		print_cov();
	}
	else {

	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KalmanGPS");

	ros::NodeHandle _n;
	ros::Publisher 	pubAltitude = _n.advertise<geometry_msgs::Vector3Stamped>("/groupe19n/height", 1000);
	ros::Publisher 	pubDebug = _n.advertise<geometry_msgs::Vector3Stamped>("/groupe19n/debug", 1000);
	_pubAltitude = &pubAltitude;
	_pubDebug = &pubDebug;

	ros::Subscriber _subNavSatFix = _n.subscribe<sensor_msgs::NavSatFix>("/iris_1/global_position/raw/fix", 1, gps_update);
	ros::Subscriber _subFluidPressure = _n.subscribe<sensor_msgs::FluidPressure>("/iris_1/imu/atm_pressure",1, baro_update);
	ros::Subscriber _subUseGps = _n.subscribe<std_msgs::Bool>("/useGPS",1, use_gps);
	ros::Subscriber _subShowDebug = _n.subscribe<std_msgs::Bool>("/showDebug",1, show_debug);

	ros::spin();

	return 0;
}