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
#define SIGMA_Z_BARO	5.0		//[pa]

ros::Time _last_msg_time;

KalmanFilter* filter = 0;
ros::Publisher *_pubAltitude = nullptr;

double pressure = 0;
bool useGPS = true;

void publish_altitude(float altitude, std_msgs::Header header) {
	geometry_msgs::Vector3Stamped data;

	data.header = header;
	data.vector.z = altitude;
	if(_pubAltitude) {
		_pubAltitude->publish(data);
	}
}

void gps_update(const sensor_msgs::NavSatFix message) {
	if(filter) {
		ros::Time message_time = ros::Time(message.header.stamp.sec, message.header.stamp.nsec);

		if(useGPS && message_time > _last_msg_time) {
			double altitude;

			filter->update_gps(message.altitude, SIGMA_Z_GPS);

			filter->getAltitude(altitude);
			publish_altitude(altitude, message.header);

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
	}
}

void baro_update(const sensor_msgs::FluidPressure message) {
	if(filter) {
		ros::Time message_time = ros::Time(message.header.stamp.sec, message.header.stamp.nsec);
		if(message_time > _last_msg_time) {
			double altitude;

			filter->update_baro(message.fluid_pressure, SIGMA_Z_BARO);

			filter->getAltitude(altitude);
			publish_altitude(altitude, message.header);
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KalmanGPS");

	ros::NodeHandle _n;
	ros::Publisher 	pubAltitude = _n.advertise<geometry_msgs::Vector3Stamped>("/height", 1000);
	_pubAltitude = &pubAltitude;

	ros::Subscriber _subNavSatFix = _n.subscribe<sensor_msgs::NavSatFix>("/iris_1/global_position/raw/fix", 1, gps_update);
	ros::Subscriber _subFluidPressure = _n.subscribe<sensor_msgs::FluidPressure>("/iris_1/imu/atm_pressure",1, baro_update);
	ros::Subscriber _subUseGps = _n.subscribe<std_msgs::Bool>("/useGPS",1, use_gps);

	ros::spin();

	return 0;
}