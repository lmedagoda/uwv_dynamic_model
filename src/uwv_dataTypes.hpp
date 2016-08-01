/****************************************************************************/
/*  Data types for an underwater vehicle 	                           	   	*/
/*                                                                         	*/
/* FILE --- uwv_dataTypes.h		                                   		   	*/
/*                                                                         	*/
/* PURPOSE --- Header file for a data types used in modeling a 		   		*/
/*             underwater vehicle. 					   						*/
/*                                                                         	*/
/*  Sankaranarayanan Natarajan                                             	*/
/*  sankar.natarajan@dfki.de                                               	*/
/*  DFKI - BREMEN 2011                                                     	*/
/****************************************************************************/
#ifndef _UWV_DATATYPES_H_
#define _UWV_DATATYPES_H_

#include "base/Eigen.hpp"
#include <vector>

namespace underwaterVehicle
{
	/**
	 * Structure used to define if the specific parameter is for positive or negative movement
	 */
	struct Direction
	{
		double positive;
		double negative;
	};

	/**
	 * Structure that contains all the necessary information for simulating the motion model
	 */
	struct Parameters
	{
		/**
		 * Number of controllable inputs
		 */
		int ctrl_order;
		
		/**
	 	 * Number of thrusters
	 	 */
		int number_of_Thrusters;
	
		/**
	 	* Number of dive cells. If no cells leave this value zero.
	 	*/
		 int number_of_cells;

		/**
		 * Number of control vectoring servos. If no vectoring servos leave this value zero.
	  	 */
		 int number_of_vectoring;

		/**
		 * Sampling time used in the simulation (in seconds)
		 */
		double samplingtime;

		/**
		 * Number of RK4 iterations per sampling interval
		 */
		int sim_per_cycle;

		/**
		 * Intertia matrices for positive and negative speeds
		 */
		base::Matrix6d massMatrix;
		base::Matrix6d massMatrixNeg;

		/**
		 * Coriolis matrices for positive and negative speeds
		 * OBS: when setting coriolis matrix, make sure to set the added mass matrix
		 *      to zero and add the added mass terms in the inertia matrix
		 */
		base::Matrix6d coriolisMatrix;
		base::Matrix6d coriolisMatrixNeg;

		/**
		 * Added Mass matrices for positive and negative speeds
		 * OBS: only set theses matrices if you want to use the complete fossen's model,
		 * otherwise add the added mass terms in the inertiaMatrix.
		 */
		base::Matrix6d AddedMassMatrixPos;
		base::Matrix6d AddedMassMatrixNeg;

		/**
		 * Linear damping matrices for positive and negative speeds
		 */
		base::Matrix6d linDampMatrix;
		base::Matrix6d linDampMatrixNeg;

		/**
		 * Quadratic damping matrices for positive and negative speeds
		 */
		base::Matrix6d quadDampMatrix;
		base::Matrix6d quadDampMatrixNeg;

		/**
		 * Lift coefficients (Yuv, Zuw, Muw, Nuv)
		 */
		base::Vector4d LiftCoefficients;
		
		/**
		 * coefficient for model correction (Muq, Nur)
		 */
		base::VectorXd CorrectionCoefficients;

		/**
		 * Thrust configuration matrix
		 */
		base::MatrixXd thruster_control_matrix;

		/**
		 * Distance from the origin of the body-fixed frame to the center of buoyancy
		 */
		base::Vector3d distance_body2centerofbuoyancy;

		/**
		 * Distance from the origin of the body-fixed frame to the center of grabity
		 */
		base::Vector3d distance_body2centerofgravity;
		
		/**
		 * Vector with the transformation from the input command position of each dive cell to the weight force it produces. 
		 * This vector should have constants since the transformation is linear.
		 */
		base::VectorXd cellinput_to_force;
		
		/**
		 * Vector Matrix with the positions of the dive cells w.r.t the center of the vehicle.(this is different from the input command positions)
		 * Matrix dimensions should be 3xn, where n is the number of cells. Leave this matrix zero if the vehicle doesn't have any diving cells.
		 */
		base::MatrixXd cells_positions;

		/**
		 * Total mass of the vehicle
		 */
		double uwv_mass;

		/**
		 * Total volume of the vehicle
		 */
		double uwv_volume;

		/**
		 * Variable used to assume that the weight of the vehicle is equal to its buoyancy force
		 */
		bool uwv_float;

		/**
		 * Density of the water in kg/m3
		 */
		double waterDensity;

		/*
		 * Acceleration of gravity
		 */
		double gravity;

		/**
		 * Variable used to convert the PWM signal into its corresponding DC Voltage
		 */
		std::vector<double> thrusterVoltage;

		/**
		 * Independent coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> thruster_coefficients_pwm;

		/**
		 * Linear coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> linear_thruster_coefficients_pwm;
		
		/**
		 * Quadratic coefficient used to convert the thruster voltage into efforts
		 */
		std::vector<Direction> square_thruster_coefficients_pwm;
		
		/**
		 * Coefficients used to convert the thruster rotational speed into efforts
		 */
		std::vector<Direction> thruster_coefficient_rpm;

		/**
		 * Initial condition used for simulation
		 */
		double initial_condition[12];

		Parameters():
			ctrl_order(7),
			number_of_Thrusters(3),
			number_of_cells(2),
			number_of_vectoring(2),
			samplingtime(0.1),
			sim_per_cycle(10),
			massMatrix(Eigen::MatrixXd::Zero(6,6)),
			massMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrix(Eigen::MatrixXd::Zero(6,6)),
			coriolisMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			AddedMassMatrixPos(Eigen::MatrixXd::Zero(6,6)),
			AddedMassMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrix(Eigen::MatrixXd::Zero(6,6)),
			linDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrix(Eigen::MatrixXd::Zero(6,6)),
			quadDampMatrixNeg(Eigen::MatrixXd::Zero(6,6)),
			LiftCoefficients(Eigen::VectorXd::Zero(4)),
			CorrectionCoefficients(Eigen::VectorXd::Zero(2)),
			thruster_control_matrix(Eigen::MatrixXd::Zero(6,1)),
			distance_body2centerofbuoyancy(Eigen::VectorXd::Zero(3)),
			distance_body2centerofgravity(Eigen::VectorXd::Zero(3)),
			cellinput_to_force(Eigen::VectorXd::Zero(2)),
			cells_positions(Eigen::MatrixXd::Zero(3,2)),
			uwv_mass(0),
			uwv_volume(0),
			uwv_float(false),
			waterDensity(998.2),
			gravity(9.81),
			thrusterVoltage(0)
		{
			thruster_coefficients_pwm.resize(1);
			linear_thruster_coefficients_pwm.resize(1);
			square_thruster_coefficients_pwm.resize(1);
			thruster_coefficient_rpm.resize(1);

			thruster_coefficients_pwm[0].positive = 0;
			thruster_coefficients_pwm[0].negative = 0;
			linear_thruster_coefficients_pwm[0].positive = 0;
			linear_thruster_coefficients_pwm[0].negative = 0;
			square_thruster_coefficients_pwm[0].positive = 0;
			square_thruster_coefficients_pwm[0].negative = 0;
			thruster_coefficient_rpm[0].positive = 0;
			thruster_coefficient_rpm[0].negative = 0;

			for(int i = 0; i < 12; i++)
				initial_condition[i] = 0;
		};
	};								    	
	
};
#endif
