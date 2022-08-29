
/// \file PlannerCommonDef.h
/// \brief Definition file for control related data types
/// \author Hatem Darweesh
/// \date Dec 14, 2016


#ifndef PLANNERCOMMONDEF_H_
#define PLANNERCOMMONDEF_H_

#include <math.h>
#include <string>

namespace op
{

enum VEHICLE_TYPE {SMALL_VEHICLE,  NORMAL_VEHICLE,  MINI_VAN_VEHICLE,  BUS_VEHICLE,  ROBOT_VEHICLE };

class PID_CONST
{
public:
	double kP;
	double kI;
	double kD;

	PID_CONST()
	{
		kP = kI = kD = 0;
	}

	PID_CONST(const double& p, const double& i, const double& d)
	{
		kP = p;
		kI = i;
		kD = d;
	}
};

class ControllerParams
{
public:
	double SimulationSteeringDelay;
	double SteeringDelay;
	double minPursuiteDistance;
	double LowpassSteerCutoff;
	int ControlFrequency;
	double avg_engine_brake_accel; // also could work as general friction or drag
	double min_safe_follow_distance; // in follow mode, keep this distance to the object in front
	PID_CONST Steering_Gain;
	PID_CONST Velocity_Gain;

	PID_CONST Torque_Gain;
	PID_CONST Accel_Gain;
	PID_CONST Brake_Gain;
	PID_CONST Follow_Gain;

	double accel_init_delay;
	double accel_avg_delay;
	double avg_acceleration;

	double brake_init_delay;
	double brake_avg_delay;
	double avg_deceleration;

	double accelPushRatio;
	double brakePushRatio;

	ControllerParams()
	{
		SimulationSteeringDelay = 0.0;
		SteeringDelay = 0.8;
		LowpassSteerCutoff = 5.0;
		minPursuiteDistance = 2.0;
		ControlFrequency = 25;
		avg_engine_brake_accel = -0.5;
		min_safe_follow_distance = 2.0;

		accel_init_delay = 0.01;
		accel_avg_delay = 0.01;
		avg_acceleration = 1.0;

		brake_init_delay = 0.01;
		brake_avg_delay = 0.01;
		avg_deceleration = -1.0;

		accelPushRatio = 1.0;
		brakePushRatio = 1.0;
	}
};

class CAR_BASIC_INFO
{
public:
	VEHICLE_TYPE model = NORMAL_VEHICLE;

  double turning_radius = 5.2; // meters
  double wheel_base = 2.7; // meters
  double length = 4.54; // meters
  double width = 1.82; // meters
  double front_length = 0.96;
  double back_length = 0.89;
  double height = 1.47;

  double max_speed_forward = 10; // m/s
  double min_speed_forward = 0.0; // m/s speed that is considered stopping, to avoid small speed noise

  double max_steer_value = 11.52; // radians, total steering wheel angle to the right, equivilent to 660
  double max_wheel_angle = 0.42; // radians, max angle for the front wheel = asin(wheel_base/turning_radius);

  double max_accel_value = 100.0; // accel pedal stroke force
  double max_brake_value = 100; // brake pedal stroke force
  double max_steer_torque = 100; // steering torque force
  double min_steer_torque = -100; // steering torque force

  double max_acceleration = 1.5; // m/s*s
  double max_deceleration = -1.5; // m/s*s
};

} /* namespace op */

#endif /* PLANNERCOMMONDEF_H_ */
