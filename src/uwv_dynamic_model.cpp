/***************************************************************************/
/*  Dynamic model for an underwater vehicle	                           */
/*                                                                         */
/* FILE --- uwv_dynamic_model.cpp	                                   */
/*                                                                         */
/* PURPOSE --- Source file for a Dynamic model of an 	                   */
/*             underwater vehicle. Based on T.I.Fossen & Giovanni Indiveri */
/*                                                                         */
/*  Sankaranarayanan Natarajan                                             */
/*  sankar.natarajan@dfki.de                                               */
/*  DFKI - BREMEN 2011                                                     */
/*                                                                         */
/*  This file was edited to include the full Fossen Model                  */
/*                                                                         */
/*  Bilal Wehbe                                                            */
/*  bilal.wehbe@dfki.de                                                    */
/*  DFKI - BREMEN 2015                                                     */
/***************************************************************************/


#include "uwv_dynamic_model.hpp"
#include <base/Logging.hpp>
#include <iostream>

namespace uwv_dynamic_model
{
DynamicModel::DynamicModel(uint controlOrder, double samplingTime,
		uint simPerCycle,  double initialTime)
: RK4_SIM(controlOrder, (samplingTime/(double)simPerCycle))
{
	// Error flags. The errorModelInit will be unset when the model is initialized
	errorModelInit 		= true;
	errorConstruction 	= false;
	errorControlInput 	= false;
	errorSetParameters 	= false;
	errorPWMCoeff 		= false;
	errorRPMCoeff 		= false;
	errorStatus 		= false;

	// Checks the arguments provided to the constructor and then initialize the
	// members of the class
	checkConstruction(samplingTime, simPerCycle, initialTime);

	if(!errorConstruction)
	{
		gSystemOrder = 12;
		gControlOrder = controlOrder;
		gSamplingTime = samplingTime;
		gSimPerCycle = simPerCycle;
		gCurrentTime = initialTime;

		// States variables
		Eigen::VectorXd statesInit = Eigen::VectorXd::Zero(12);
		updateStates(statesInit);

		gLinearAcceleration = Eigen::VectorXd::Zero(3);
		gAngularAcceleration = Eigen::VectorXd::Zero(3);

		gEfforts = Eigen::VectorXd::Zero(6);

		// Model parameters
		setInertiaMatrix(Eigen::MatrixXd::Zero(6,6));
		setCoriolisMatrix(Eigen::MatrixXd::Zero(6,6));
		setAddedMassMatrix(Eigen::MatrixXd::Zero(6,6));
		setLinDampingMatrix(Eigen::MatrixXd::Zero(6,6));
		setQuadDampingMatrix(Eigen::MatrixXd::Zero(6,6));
		setLiftCoefficients(Eigen::VectorXd::Zero(4));
		setAddedMassMatrix(Eigen::MatrixXd::Zero(6,6));
		gThrustConfigMatrix = Eigen::MatrixXd::Zero(6,1);

		// Thrusters' coefficients
		gThrusterCoeffPWM.positive = 0;
		gThrusterCoeffPWM.negative = 0;
		gLinThrusterCoeffPWM.positive = 0;
		gLinThrusterCoeffPWM.negative = 0;
		gQuadThrusterCoeffPWM.positive = 0;
		gQuadThrusterCoeffPWM.negative = 0;
		gThrusterCoeffRPM.positive = 0;
		gThrusterCoeffRPM.negative = 0;
		gThrusterVoltage = 0;

		// Restoring forces' variables
		gWeight = 0;
		gBuoyancy = 0;
		gCenterOfGravity = Eigen::VectorXd::Zero(3);
		gCenterOfBuoyancy = Eigen::VectorXd::Zero(3);
		gUWVFloat = false;
		gUWVMass = 0;
		gUWVVolume = 0;
		gGravity = 0;
		gWaterDensity = 0;
	}
}

bool DynamicModel::initParameters(const uwv_dynamic_model::Parameters &uwvParameters)
{
	// Checks if the model wasn't initialized yet
	if(errorModelInit)
	{
		// Checks if there was any error in the library
		if(!errorConstruction)
		{
			// Unsets the errorModelInit because the model is being initialized
			errorModelInit = false;

			Eigen::VectorXd statesInit = uwvParameters.initialStates;

			// Updates initial system states
			updateStates(statesInit);

			// Sets the uwv parameters
			setUWVParameters(uwvParameters);

			// Checks if the positive inertia, positive linear damping and thrust
			// configuration matrices were set
			checkPositiveMatrices();

			if(!errorStatus)
				return true;
			else
				return false;
		}
		else
			return false;
	}
	else
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The model was already initialized, use the function"
				" setUWVParameters instead if you want to change any"
				" UWV parameter.\x1b[0m\n\n");
		return false;
	}
}

bool DynamicModel::sendPWMCommands(const base::samples::Joints &controlInput)
{
	// Checks if the control input is valid
	checkControlInput(controlInput, "raw");

	// Checks if the PWM coefficients were properly set
	checkPWMCoefficients();

	// Checks if there is any error flag activated
	checkErrors();

	// Checks if there was any error in the library
	if(!errorStatus)
	{
		Eigen::VectorXd thrustersForce 	=	Eigen::VectorXd::Zero(gControlOrder);
		Eigen::VectorXd dcVoltage 		=	Eigen::VectorXd::Zero(gControlOrder);
		Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(12);

		// Calculates the forces and moments generated by the thrusters
		pwmToDC(dcVoltage, controlInput);
		dcToThrustForce(thrustersForce, dcVoltage);
		thrustForceToEffort(gEfforts, thrustersForce);

		// Gets a vector with the current system states (pose and velocities)
		getStates(systemStates);

		// Performs iterations to calculate the new system's states
		for (int i=0; i < gSimPerCycle ; i++)
			calcStates(systemStates, gCurrentTime, gEfforts);

		// Updates the new system's states
		updateStates(systemStates);

		return true;
	}
	else
		return false;
}

bool DynamicModel::sendRPMCommands(const base::samples::Joints &controlInput)
{
	// Checks if the control input is valid
	checkControlInput(controlInput, "speed");

	// Checks if the RPM coefficients were properly set
	checkRPMCoefficients();

	// Checks if there is any error flag activated
	checkErrors();

	// Checks if there was any error in the library
	if(!errorStatus)
	{
		Eigen::VectorXd thrustersForce = Eigen::VectorXd::Zero(gControlOrder);
		Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(12);

		// Calculates the forces and moments generated by the thrusters
		rpmToThrustForce(thrustersForce, controlInput);
		thrustForceToEffort(gEfforts, thrustersForce);

		// Gets a vector with the current system states (pose and velocities)
		getStates(systemStates);

		// Performs iterations to calculate the new system's states
		for (int ii=0; ii < gSimPerCycle; ii++)
			calcStates(systemStates, gCurrentTime, gEfforts);

		// Updates the new system's states
		updateStates(systemStates);

		return true;
	}
	else
		return false;
}

bool DynamicModel::sendEffortCommands(const base::samples::Joints &controlInput)
{
	// Checks if the control input is valid
	checkControlInput(controlInput, "effort");

	// Checks if there is any error flag activated
	checkErrors();

	// Checks if there was any error in the library
	if(!errorStatus)
	{
		Eigen::VectorXd systemStates = Eigen::VectorXd::Zero(12);

		// Puts the efforts in a vector
		for (int i = 0; i < gSystemOrder/2; i++)
			gEfforts[i] = controlInput[i].effort;

		// Gets a vector with the current system states (pose and velocities)
		getStates(systemStates);

		// Performs iterations to calculate the new system's states
		for (int ii=0; ii < gSimPerCycle; ii++)
			calcStates(systemStates, gCurrentTime, gEfforts);

		// Updates the new system's states
		updateStates(systemStates);

		return true;
	}
	else
		return false;
}

void DynamicModel::calcAcceleration(Eigen::VectorXd &velocityAndAcceleration,
		const base::Vector6d &velocity,
		const base::Vector6d &controlInput)
{
	/**
	 * velocityAndAcceleration:
	 *
	 * [0] = u		[6]  = u_dot	(SURGE)
	 * [1] = v		[7]  = v_dot	(SWAY)
	 * [2] = w		[8]  = w_dot	(HEAVE)
	 * [3] = p		[9]  = p_dot	(ROLL)
	 * [4] = q		[10] = q_dot	(PITCH)
	 * [5] = r		[11] = r_dot	(YAW)
	 *
	 */

	// Forces and Moments vectors
	base::Matrix6d invInertiaMatrix 	= 	Eigen::MatrixXd::Zero(6,6);
	base::Vector6d coriolisEffect		=	Eigen::VectorXd::Zero(6);
	base::Vector6d RBCoriolis 	        =	Eigen::VectorXd::Zero(6);
	base::Vector6d AddedMassCoriolis	=	Eigen::VectorXd::Zero(6);
	base::Vector6d linDamping		=	Eigen::VectorXd::Zero(6);
	base::Vector6d quadDamping		=	Eigen::VectorXd::Zero(6);
	base::Vector6d LiftEffect		=	Eigen::VectorXd::Zero(6);
	base::Vector6d gravityBuoyancy		=	Eigen::VectorXd::Zero(6);
	base::Vector6d acceleration		=	Eigen::VectorXd::Zero(6);
	base::Vector6d worldVelocity		=	Eigen::VectorXd::Zero(6);
	base::Vector6d ModelCorrection		=	Eigen::VectorXd::Zero(6);

	// Calculating the efforts for each one of the hydrodynamics effects
	calcInvInertiaMatrix(invInertiaMatrix, velocity);
	calcCoriolisEffect(coriolisEffect, velocity);
	calcRBCoriolis(RBCoriolis, velocity);
	calcAddedMassCoriolis(AddedMassCoriolis, velocity);
	calcLinDamping(linDamping, velocity);
	calcQuadDamping(quadDamping, velocity);
	calcLiftEffect(LiftEffect, velocity);
	calcGravityBuoyancy(gravityBuoyancy, gEulerOrientation);
	calcModelCorrection(ModelCorrection, velocity);

	// Calculating the acceleration based on all the hydrodynamics effects
	acceleration  = invInertiaMatrix * ( gEfforts - coriolisEffect - RBCoriolis - AddedMassCoriolis - LiftEffect  -
			linDamping - quadDamping - gravityBuoyancy - ModelCorrection);

	// Converting the body velocity to world velocity. This is necessary because
	// when the integration takes place in order to find the position, the velocity
	// should be expressed in the world frame, just like the position is.
	convBodyToWorld(worldVelocity, velocity, gEulerOrientation);

	// Updating the RK4 vector with the velocity and acceleration values
	for (int i = 0; i < 6; i++)
	{
		velocityAndAcceleration[i] = worldVelocity[i];
		velocityAndAcceleration[i+6] = acceleration[i];
	}

	// Updating global acceleration variables
	for(int i = 0; i < 3; i++)
	{
		gLinearAcceleration[i]  = acceleration[i];
		gAngularAcceleration[i] = acceleration[i+3];
	}
}

bool DynamicModel::setUWVParameters(const uwv_dynamic_model::Parameters &uwvParameters)
{
	// Checks if there is any parameter inconsistency
	checkParameters(uwvParameters);

	// Checks if there is any error flag activated
	checkErrors();

	// Checks if the model was already initialized
	if(!errorModelInit)
	{
		if(!errorStatus)
		{
			setInertiaMatrix(uwvParameters.inertiaMatrixPos,
					uwvParameters.inertiaMatrixNeg);
			setCoriolisMatrix(uwvParameters.coriolisMatrixPos,
					uwvParameters.coriolisMatrixNeg);
			setAddedMassMatrix(uwvParameters.AddedMassMatrixPos,
					uwvParameters.AddedMassMatrixNeg);
			setLiftCoefficients(uwvParameters.LiftCoefficients);
			setLinDampingMatrix(uwvParameters.linDampMatrixPos,
					uwvParameters.linDampMatrixPos);
			setQuadDampingMatrix(uwvParameters.quadDampMatrixPos,
					uwvParameters.quadDampMatrixNeg);

			gThrustConfigMatrix 	= uwvParameters.thrustConfigMatrix;

			gThrusterCoeffPWM 		= uwvParameters.thrusterCoeffPWM;
			gLinThrusterCoeffPWM 	= uwvParameters.linThrusterCoeffPWM;
			gQuadThrusterCoeffPWM 	= uwvParameters.quadThrusterCoeffPWM;

			gThrusterCoeffRPM 		= uwvParameters.thrusterCoeffRPM;
			gThrusterVoltage 		= uwvParameters.thrusterVoltage;

			gWeight	  = uwvParameters.uwvMass*uwvParameters.gravity;
			gBuoyancy =	uwvParameters.waterDensity*uwvParameters.gravity*
					uwvParameters.uwvVolume;

			gCenterOfGravity  	= uwvParameters.centerOfGravity;
			gCenterOfBuoyancy 	= uwvParameters.centerOfBuoyancy;

			gUWVFloat 		  	= uwvParameters.uwvFloat;
			gUWVMass 			= uwvParameters.uwvMass;
			gUWVVolume 			= uwvParameters.uwvVolume;
			gGravity 			= uwvParameters.gravity;
			gWaterDensity		= uwvParameters.waterDensity;
			gSimPerCycle		= uwvParameters.simPerCycle;
			return true;
		}
		else
			return false;
	}
	else
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The model wasn't initialized yet. Use the function"
				" initParameters first to set the model parameters,"
				" and then, if you want to modify any of them, use the"
				" function setUWVParameters.\x1b[0m\n\n");
		return false;
	}
}

void DynamicModel::resetStates()
{
	Eigen::Vector3d resetVector3d = Eigen::VectorXd::Zero(3);

	gPosition 			= resetVector3d;
	gEulerOrientation 	= resetVector3d;
	gLinearVelocity 	= resetVector3d;
	gAngularVelocity 	= resetVector3d;
}

void DynamicModel::getUWVParameters(uwv_dynamic_model::Parameters &uwvParameters)
{
	uwvParameters.inertiaMatrixPos 		= gInertiaMatrixPos;
	uwvParameters.inertiaMatrixNeg 		= gInertiaMatrixNeg;
	uwvParameters.coriolisMatrixPos 	= gCoriolisMatrixPos;
	uwvParameters.coriolisMatrixNeg 	= gCoriolisMatrixNeg;
	uwvParameters.AddedMassMatrixPos 	= gAddedMassMatrixPos;
	uwvParameters.AddedMassMatrixNeg 	= gAddedMassMatrixNeg;
	uwvParameters.linDampMatrixPos 		= gLinDampMatrixPos;
	uwvParameters.linDampMatrixNeg	 	= gLinDampMatrixNeg;
	uwvParameters.quadDampMatrixPos 	= gQuadDampMatrixPos;
	uwvParameters.quadDampMatrixNeg 	= gQuadDampMatrixNeg;
	uwvParameters.thrustConfigMatrix 	= gThrustConfigMatrix;
	uwvParameters.LiftCoefficients  	= gLiftCoefficients;

	uwvParameters.thrusterCoeffPWM 		= gThrusterCoeffPWM;
	uwvParameters.linThrusterCoeffPWM 	= gLinThrusterCoeffPWM;
	uwvParameters.quadThrusterCoeffPWM 	= gQuadThrusterCoeffPWM;
	uwvParameters.thrusterCoeffRPM 		= gThrusterCoeffRPM;
	uwvParameters.thrusterVoltage 		= gThrusterVoltage;

	uwvParameters.centerOfGravity 		= gCenterOfGravity;
	uwvParameters.centerOfBuoyancy 		= gCenterOfBuoyancy;
	uwvParameters.uwvFloat 				= gUWVFloat;
	uwvParameters.uwvMass 				= gUWVMass;
	uwvParameters.uwvVolume 			= gUWVVolume;
	uwvParameters.gravity 				= gGravity;
	uwvParameters.waterDensity 			= gWaterDensity;
	uwvParameters.simPerCycle			= gSimPerCycle;

	uwvParameters.controlOrder			= gControlOrder;
	uwvParameters.samplingTime			= gSamplingTime;
}

void DynamicModel::getPosition(base::Position &position)
{
	position = gPosition;
}

void DynamicModel::getEulerOrientation(Eigen::Vector3d &eulerOrientation)
{
	eulerOrientation = gEulerOrientation;
}

void DynamicModel::getQuatOrienration(base::Orientation &quatOrientation)
{
	eulerToQuaternion(quatOrientation, gEulerOrientation);
}

void DynamicModel::getLinearVelocity(base::Vector3d &linearVelocity)
{
	linearVelocity = gLinearVelocity;
}

void DynamicModel::getAngularVelocity(base::Vector3d &angularVelocity)
{
	angularVelocity = gAngularVelocity;
}

void DynamicModel::getLinearAcceleration(base::Vector3d &linearAcceleration)
{
	linearAcceleration = gLinearAcceleration;
}

void DynamicModel::getAngularAcceleration(base::Vector3d &angularAcceleration)
{
	angularAcceleration = gAngularAcceleration;
}

void DynamicModel::getStates(Eigen::VectorXd &systemStates)
{
	systemStates.segment(0,3) = gPosition;
	systemStates.segment(3,3) = gEulerOrientation;
	systemStates.segment(6,3) = gLinearVelocity;
	systemStates.segment(9,3) = gAngularVelocity;
}

void DynamicModel::getEfforts(base::Vector6d &efforts)
{
	efforts = gEfforts;
}

void DynamicModel::getSimulationTime(double &simulationTime)
{
	simulationTime = gCurrentTime;
}

void DynamicModel::getSamplingTime(double &samplingTime)
{
	samplingTime = gSamplingTime;
}

void DynamicModel::getSimPerCycle(int &simPerCycle)
{
	simPerCycle = gSimPerCycle;
}

void DynamicModel::eulerToQuaternion(base::Quaterniond &quaternion,
		const Eigen::Vector3d &eulerAngles)
{

	quaternion.w() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
			( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
	quaternion.x() = ( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) -
			( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
	quaternion.y() = ( cos(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) ) +
			( sin(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) );
	quaternion.z() = ( cos(eulerAngles(0)/2)*cos(eulerAngles(1)/2)*sin(eulerAngles(2)/2) ) -
			( sin(eulerAngles(0)/2)*sin(eulerAngles(1)/2)*cos(eulerAngles(2)/2) );
}

void DynamicModel::calcInvInertiaMatrix(base::Matrix6d &invInertiaMatrix,
		const base::Vector6d &velocity)
{
	Eigen::MatrixXd inertiaMatrix = Eigen::MatrixXd::Zero(6,6);
	Eigen::MatrixXd AddedMassMatrix = Eigen::MatrixXd::Zero(6,6);

	for(int i = 0; i < 6; i++)
	{
		if(velocity(i) > -0.001)
			inertiaMatrix.block(0 , i , 6, 1) = gInertiaMatrixPos.block(0 , i , 6, 1);
		else
			inertiaMatrix.block(0 , i , 6, 1) = gInertiaMatrixNeg.block(0 , i , 6, 1);
	}

	for(int i = 0; i < 6; i++)
	{
		if(velocity(i) > -0.001)
			AddedMassMatrix.block(0 , i , 6, 1) = gAddedMassMatrixPos.block(0 , i , 6, 1);
		else
			AddedMassMatrix.block(0 , i , 6, 1) = gAddedMassMatrixNeg.block(0 , i , 6, 1);
	}

	inertiaMatrix = inertiaMatrix + AddedMassMatrix;

	invInertiaMatrix = inertiaMatrix.inverse();

}

void DynamicModel::calcCoriolisEffect(base::Vector6d &coriolisEffect,
		const base::Vector6d &velocity)
{
	Eigen::MatrixXd coriolisMatrix = Eigen::MatrixXd::Zero(6,6);

	for(int i = 0; i < 6; i++)
	{
		if(velocity(i) > -0.001)
			coriolisMatrix.block(0 , i , 6, 1) = gCoriolisMatrixPos.block(0 , i , 6, 1);
		else
			coriolisMatrix.block(0 , i , 6, 1) = gCoriolisMatrixNeg.block(0 , i , 6, 1);
	}
	coriolisEffect = coriolisMatrix*velocity;
}

void DynamicModel::calcLinDamping(base::Vector6d &linDamping,
		const base::Vector6d &velocity)
{
	Eigen::MatrixXd linDampMatrix = Eigen::MatrixXd::Zero(6,6);

	for(int i = 0; i < 6; i++)
	{

		if(velocity(i) > -0.001)
			linDampMatrix.block(0 , i , 6, 1) = gLinDampMatrixPos.block(0, i , 6, 1);
		else
			linDampMatrix.block(0 , i , 6, 1) = gLinDampMatrixNeg.block(0 , i , 6, 1);

	}
	linDamping = linDampMatrix*velocity;
}

void DynamicModel::calcQuadDamping(base::Vector6d &quadDamping,
		const base::Vector6d &velocity)
{
	Eigen::MatrixXd quadDampMatrix = Eigen::MatrixXd::Zero(6,6);
	Eigen::VectorXd absoluteVelocity = Eigen::VectorXd::Zero(6);

	for(int i = 0; i < 6; i++)
	{
		if(velocity(i) > -0.001)
			quadDampMatrix.block(0 , i , 6, 1) = gQuadDampMatrixPos.block(0, i , 6, 1);
		else
			quadDampMatrix.block(0 , i , 6, 1) = gQuadDampMatrixNeg.block(0 , i , 6, 1);

		absoluteVelocity[i] = fabs(velocity[i]);
	}

	quadDamping = quadDampMatrix*absoluteVelocity.asDiagonal()*velocity;
}

void DynamicModel::calcLiftEffect(base::Vector6d &LiftEffect,
		const base::Vector6d &velocity)
{
	double u = velocity[0];
	double v = velocity[1];
	double w = velocity[2];

	LiftEffect(0) = 0;
	LiftEffect(1) = gLiftCoefficients(0)*u*v;
	LiftEffect(2) = gLiftCoefficients(1)*u*w;
	LiftEffect(3) = 0;
	LiftEffect(4) = gLiftCoefficients(2)*u*w;
	LiftEffect(5) = gLiftCoefficients(3)*u*v;
}

void DynamicModel::calcRBCoriolis(base::Vector6d &RBCoriolis,
		const base::Vector6d &velocity)
{

	float xg = gCenterOfGravity(0);
	float yg = gCenterOfGravity(1);
	float zg = gCenterOfGravity(2);

	float m = gInertiaMatrixPos(0,0);
	float Ix = gInertiaMatrixPos(3,3);
	float Iy = gInertiaMatrixPos(4,4);
	float Iz = gInertiaMatrixPos(5,5);
	float Ixy = -gInertiaMatrixPos(3,4);
	float Iyz = -gInertiaMatrixPos(4,5);
	float Izx = -gInertiaMatrixPos(3,5);

	double u = velocity[0];
	double v = velocity[1];
	double w = velocity[2];

	double p = velocity[3];
	double q = velocity[4];
	double r = velocity[5];


	RBCoriolis(0) 	=  m*(-v*r + w*q - xg*(q*q + r*r) +yg*p*q + zg*p*r);
	RBCoriolis(1) 	=  m*(-w*p + u*r - yg*(r*r + p*p) +zg*q*r + xg*q*p);
	RBCoriolis(2) 	=  m*(-u*q + v*p - zg*(p*p + q*q) +xg*r*p + yg*r*q);
	RBCoriolis(3) 	=  (Iz - Iy)*q*r - p*q*Izx + (r*r - q*q)*Iyz + p*r*Ixy + m*(yg*(-u*q + v*p) - zg*(-w*p + u*r));
	RBCoriolis(4) 	=  (Ix - Iz)*r*p - q*r*Ixy + (p*p - r*r)*Izx + q*p*Iyz + m*(zg*(-v*r + w*q) - xg*(-u*q + v*p));
	RBCoriolis(5) 	=  (Iy - Ix)*p*q - r*p*Iyz + (q*q - p*p)*Ixy + r*q*Izx + m*(xg*(-w*p + u*r) - yg*(-v*r + w*q));
}

void DynamicModel::calcAddedMassCoriolis(base::Vector6d &AddedMassCoriolis,
		const base::Vector6d &velocity)
{

	double u = velocity[0];
	double v = velocity[1];
	double w = velocity[2];

	double p = velocity[3];
	double q = velocity[4];
	double r = velocity[5];

	Eigen::MatrixXd gAddedMassMatrix = Eigen::MatrixXd::Zero(6,6);

	for(int i = 0; i < 6; i++)
	{
		if(velocity(i) > -0.001)
			gAddedMassMatrix.block(0 , i , 6, 1) = gAddedMassMatrixPos.block(0 , i , 6, 1);
		else
			gAddedMassMatrix.block(0 , i , 6, 1) = gAddedMassMatrixNeg.block(0 , i , 6, 1);
	}


	double a1 = gAddedMassMatrix(0,0)*u + gAddedMassMatrix(0,1)*v + gAddedMassMatrix(0,2)*w + gAddedMassMatrix(0,3)*p + gAddedMassMatrix(0,4)*q + gAddedMassMatrix(0,5)*r;
	double a2 = gAddedMassMatrix(0,1)*u + gAddedMassMatrix(1,1)*v + gAddedMassMatrix(1,2)*w + gAddedMassMatrix(1,3)*p + gAddedMassMatrix(1,4)*q + gAddedMassMatrix(1,5)*r;
	double a3 = gAddedMassMatrix(0,2)*u + gAddedMassMatrix(1,2)*v + gAddedMassMatrix(2,2)*w + gAddedMassMatrix(2,3)*p + gAddedMassMatrix(2,4)*q + gAddedMassMatrix(2,5)*r;
	double b1 = gAddedMassMatrix(0,3)*u + gAddedMassMatrix(1,3)*v + gAddedMassMatrix(2,3)*w + gAddedMassMatrix(3,3)*p + gAddedMassMatrix(3,4)*q + gAddedMassMatrix(3,5)*r;
	double b2 = gAddedMassMatrix(0,4)*u + gAddedMassMatrix(1,4)*v + gAddedMassMatrix(2,4)*w + gAddedMassMatrix(3,4)*p + gAddedMassMatrix(4,4)*q + gAddedMassMatrix(4,5)*r;
	double b3 = gAddedMassMatrix(0,5)*u + gAddedMassMatrix(1,5)*v + gAddedMassMatrix(2,5)*w + gAddedMassMatrix(3,5)*p + gAddedMassMatrix(4,5)*q + gAddedMassMatrix(5,5)*r;


	AddedMassCoriolis(0) 	= -(-a3*q + a2*r);
	AddedMassCoriolis(1) 	= -(a3*p - a1*r);
	AddedMassCoriolis(2) 	= -(-a2*p + a1*q);
	AddedMassCoriolis(3) 	= -(-a3*v + a2*w - b3*q + b2*r);
	AddedMassCoriolis(4) 	= -(a3*u - a1*w + b3*p - b1*r);
	AddedMassCoriolis(5) 	= -(-a2*u + a1*v - b2*p + b1*q);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------
// New function to correct model forces

void DynamicModel::calcModelCorrection(base::Vector6d &ModelCorrection,
		const base::Vector6d &velocity)
{

	double u = velocity[0];
	//double v = velocity[1];
	//double w = velocity[2];

	//double p = velocity[3];
	double q = velocity[4];
	double r = velocity[5];

	ModelCorrection(0) 	= 0;
	ModelCorrection(1) 	= 0;
	ModelCorrection(2) 	= 0;
	ModelCorrection(3) 	= 0;
	ModelCorrection(4) 	= 21.36*u*q;
	ModelCorrection(5) 	= 41.93*u*r;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------
void DynamicModel::calcGravityBuoyancy(base::Vector6d &gravitybuoyancy,
		const Eigen::Vector3d &eulerOrientation)
{
	float e1 = eulerOrientation(0);
	float e2 = eulerOrientation(1);
	float xg = gCenterOfGravity(0);
	float yg = gCenterOfGravity(1);
	float zg = gCenterOfGravity(2);
	float xb = gCenterOfBuoyancy(0);
	float yb = gCenterOfBuoyancy(1);
	float zb = gCenterOfBuoyancy(2);

	if (gUWVFloat == true)
		gBuoyancy = gWeight;

	gravitybuoyancy(0) 	= 	(gWeight - gBuoyancy) * sin(e2);
	gravitybuoyancy(1) 	=  -(gWeight - gBuoyancy) * (cos(e2)*sin(e1));
	gravitybuoyancy(2) 	=  -(gWeight - gBuoyancy) * (cos(e2)*cos(e1));
	gravitybuoyancy(3) 	=  -((yg*gWeight - yb*gWeight)*cos(e2)*cos(e1)) +
			((zg*gWeight - zb*gBuoyancy) * cos(e2)*sin(e1));
	gravitybuoyancy(4) 	=   ((zg*gWeight - zb*gWeight)*sin(e2)) +
			((xg*gWeight - xb*gBuoyancy)*cos(e2)*cos(e1));
	gravitybuoyancy(5) 	=  -((xg*gWeight - xb*gWeight)*cos(e2)*sin(e1)) -
			((yg*gWeight - yb*gBuoyancy)* sin(e2));


}

void DynamicModel::convBodyToWorld(base::Vector6d &worldCoordinates,
		const base::Vector6d &bodyCoordinates,
		const base::Vector3d &eulerAngles)
{
	base::Matrix6d transfMatrix = Eigen::MatrixXd::Zero(6,6);

	calcTransfMatrix(transfMatrix, eulerAngles);

	worldCoordinates = transfMatrix * bodyCoordinates;
}

void DynamicModel::convWorldToBody(base::Vector6d &bodyCoordinates,
		const base::Vector6d &worldCoordinates,
		const base::Vector3d &eulerAngles)
{
	base::Matrix6d transfMatrix = Eigen::MatrixXd::Zero(6,6);
	base::Matrix6d invTransfMatrix = Eigen::MatrixXd::Zero(6,6);

	calcTransfMatrix(transfMatrix, eulerAngles);
	invTransfMatrix = transfMatrix.inverse();

	bodyCoordinates = invTransfMatrix * worldCoordinates;
}

void DynamicModel::calcTransfMatrix(base::Matrix6d &transfMatrix,
		const base::Vector3d &eulerAngles)
{
	double phi   = eulerAngles[0];
	double theta = eulerAngles[1];
	double psi   = eulerAngles[2];
	base::Matrix3d J1;
	base::Matrix3d J2;

	J1 << cos(psi)*cos(theta),   -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi),   sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
			sin(psi)*cos(theta),    cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi),  -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi),
			-sin(theta)     ,                 cos(theta)*sin(phi)                ,               cos(theta)*cos(phi)                 ;


	J2 <<  1,       sin(phi)*tan(theta),       cos(phi)*tan(theta),
			0,            cos(phi)      ,       		-sin(phi)	  ,
			0,       sin(phi)/cos(theta),       cos(phi)/cos(theta);

	transfMatrix.block<3,3>(0,0) = J1;
	transfMatrix.block<3,3>(3,3) = J2;
}

void DynamicModel::pwmToDC(Eigen::VectorXd &dcVolt,
		const base::samples::Joints &controlInput)
{
	for ( int i = 0; i < gControlOrder; i ++)
		dcVolt[i] = gThrusterVoltage * controlInput[i].raw;
}

void DynamicModel::dcToThrustForce(Eigen::VectorXd &thrustForces,
		const Eigen::VectorXd &dcVolt)
{
	for ( int i = 0; i < gControlOrder; i ++)
	{
		float factor = 0.01;

		if ((dcVolt[i] >= -factor) && ( dcVolt[i] <= factor ))
			thrustForces[i] = 0.0;
		else
		{
			double thruster_coefficient;
			if(dcVolt[i] > 0)
				thruster_coefficient = gThrusterCoeffPWM.positive
				+ gLinThrusterCoeffPWM.positive*dcVolt[i]
													   + gQuadThrusterCoeffPWM.positive*dcVolt[i]*fabs(dcVolt[i]);
			else
				thruster_coefficient = gThrusterCoeffPWM.negative
				+ gLinThrusterCoeffPWM.negative*dcVolt[i]
													   + gQuadThrusterCoeffPWM.negative*dcVolt[i]*fabs(dcVolt[i]);

			thrustForces[i] = thruster_coefficient*fabs(dcVolt[i])*dcVolt[i];
		}
	}
}

void DynamicModel::rpmToThrustForce(Eigen::VectorXd &thrustForces,
		const base::samples::Joints &controlInput)
{
	for (int i = 0; i < gControlOrder; i++)
	{
		if(controlInput[i].speed > -0.001)
			thrustForces[i] = gThrusterCoeffRPM.positive *
			(fabs(controlInput[i].speed) * controlInput[i].speed);
		else
			thrustForces[i] = gThrusterCoeffRPM.negative *
			(fabs(controlInput[i].speed) * controlInput[i].speed);
	}
}

void DynamicModel::thrustForceToEffort(base::Vector6d &forcesAndMoments,
		const Eigen::VectorXd &thrustInput)
{
	forcesAndMoments = gThrustConfigMatrix*thrustInput;
}

void DynamicModel::updateStates(Eigen::VectorXd &newSystemStates)
{
	gPosition			= newSystemStates.segment(0,3);
	gEulerOrientation	= newSystemStates.segment(3,3);
	gLinearVelocity		= newSystemStates.segment(6,3);
	gAngularVelocity	= newSystemStates.segment(9,3);
}

void DynamicModel::setInertiaMatrix(const base::Matrix6d &inertiaMatrixPos,
		const base::Matrix6d &inertiaMatrixNeg)
{
	gInertiaMatrixPos = inertiaMatrixPos;
	gInertiaMatrixNeg = inertiaMatrixNeg;

	checkNegativeMatrices(gInertiaMatrixNeg, gInertiaMatrixPos);
}

void DynamicModel::setCoriolisMatrix(const base::Matrix6d &coriolisMatrixPos,
		const base::Matrix6d &coriolisMatrixNeg)
{
	gCoriolisMatrixPos = coriolisMatrixPos;
	gCoriolisMatrixNeg = coriolisMatrixNeg;

	checkNegativeMatrices(gCoriolisMatrixNeg, gCoriolisMatrixPos);
}

void DynamicModel::setAddedMassMatrix(const base::Matrix6d &AddedMassMatrixPos,
		const base::Matrix6d &AddedMassMatrixNeg)
{
	gAddedMassMatrixPos = AddedMassMatrixPos;
	gAddedMassMatrixNeg = AddedMassMatrixNeg;

	checkNegativeMatrices(gAddedMassMatrixNeg, gAddedMassMatrixPos);
}

void DynamicModel::setLiftCoefficients(const base::Vector4d &LiftCoefficients)
{
	gLiftCoefficients = LiftCoefficients;
}

void DynamicModel::setLinDampingMatrix(const base::Matrix6d &linDampingMatrixPos,
		const base::Matrix6d &linDampingMatrixNeg)
{
	gLinDampMatrixPos = linDampingMatrixPos;
	gLinDampMatrixNeg = linDampingMatrixNeg;

	checkNegativeMatrices(gLinDampMatrixNeg, gLinDampMatrixPos);
}

void DynamicModel::setQuadDampingMatrix(const base::Matrix6d &quadDampingMatrixPos,
		const base::Matrix6d &quadDampingMatrixNeg)
{
	gQuadDampMatrixPos = quadDampingMatrixPos;
	gQuadDampMatrixNeg = quadDampingMatrixNeg;

	checkNegativeMatrices(gQuadDampMatrixNeg, gQuadDampMatrixPos);
}

void DynamicModel::checkConstruction(double &samplingTime, uint &simPerCycle,
		double &initialTime)
{
	std::string textElement;
	std::string textComparison;
	bool checkError = false;

	if (samplingTime <= 0)
	{
		textElement = "sampling time";
		textComparison = "greater than";
		checkError = true;
	}
	else if (simPerCycle == 0)
	{
		textElement = "number of simulations per cycle";
		textComparison = "greater than";
		checkError = true;
	}
	else if (initialTime < 0)
	{
		textElement = "initial time";
		textComparison = "greater or equal to";
		checkError = true;
	}

	if(checkError)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s should be %s zero.\x1b[0m\n\n"
				, textElement.c_str(), textComparison.c_str());
		errorConstruction = true;
	}
}

void DynamicModel::checkParameters(const uwv_dynamic_model::Parameters &pwvParameters)
{
	std::string textElement;
	bool checkError = false;

	errorSetParameters = false;

	// Checking if the inertia matrices are invertible
	if(pwvParameters.inertiaMatrixPos.determinant() == 0)
	{
		textElement = "positive";
		checkError = true;
	}
	else if(pwvParameters.inertiaMatrixNeg != Eigen::MatrixXd::Zero(6,6) &&
			pwvParameters.inertiaMatrixNeg.determinant() == 0)
	{
		textElement = "negative";
		checkError = true;
	}

	if(checkError && !errorSetParameters)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s inertia matrix is not invertible.\x1b[0m\n\n",
				textElement.c_str());
		errorSetParameters = true;
	}

	// Checking if the thrust configuration matrix has a coherent size

	if (pwvParameters.thrustConfigMatrix.rows() != 6 && !errorSetParameters)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The thruster configuration matrix should have 6"
				" rows, in order to be equal to the number of degrees"
				" of freedom. The provided matrix has %i rows.\x1b[0m\n\n"
				, pwvParameters.thrustConfigMatrix.rows());

		errorSetParameters = true;
	}
	else if(pwvParameters.thrustConfigMatrix.cols() != gControlOrder
			&& !errorSetParameters)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The thruster configuration matrix should have %i"
				" columns, in order to be equal to the number of control"
				" inputs. The provided matrix has %i columns.\x1b[0m\n\n"
				, gControlOrder, pwvParameters.thrustConfigMatrix.cols());
		errorSetParameters = true;
	}

	// Checking if other parameters were properly set (no negative values)

	checkError = false;

	if(pwvParameters.uwvMass < 0)
	{
		textElement = "uwvMass";
		checkError = true;
	}
	else if(pwvParameters.uwvVolume < 0)
	{
		textElement = "uwvVolume";
		checkError = true;
	}
	else if(pwvParameters.waterDensity < 0)
	{
		textElement = "waterDensity";
		checkError = true;
	}
	else if(pwvParameters.gravity < 0)
	{
		textElement = "gravity";
		checkError = true;
	}
	else if(pwvParameters.thrusterVoltage < 0)
	{
		textElement = "thrusterVoltage";
		checkError = true;
	}
	else if(pwvParameters.thrusterCoeffPWM.positive < 0)
	{
		textElement = "thrusterCoeffPWM.positive";
		checkError = true;
	}
	else if(pwvParameters.thrusterCoeffPWM.negative < 0)
	{
		textElement = "thrusterCoeffPWM.negative";
		checkError = true;
	}
	else if(pwvParameters.linThrusterCoeffPWM.positive < 0)
	{
		textElement = "linThrusterCoeffPWM.positive";
		checkError = true;
	}
	else if(pwvParameters.linThrusterCoeffPWM.negative < 0)
	{
		textElement = "linThrusterCoeffPWM.negative";
		checkError = true;
	}
	else if(pwvParameters.quadThrusterCoeffPWM.positive < 0)
	{
		textElement = "quadThrusterCoeffPWM.positive";
		checkError = true;
	}
	else if(pwvParameters.quadThrusterCoeffPWM.negative < 0)
	{
		textElement = "quadThrusterCoeffPWM.negative";
		checkError = true;
	}
	else if(pwvParameters.thrusterCoeffRPM.positive < 0)
	{
		textElement = "thrusterCoeffRPM.positive";
		checkError = true;
	}
	else if(pwvParameters.thrusterCoeffRPM.negative < 0)
	{
		textElement = "thrusterCoeffRPM.negative";
		checkError = true;
	}
	else if(pwvParameters.simPerCycle == 0)
	{
		textElement = "simPerCycle";
		checkError = true;
	}

	if(checkError && !errorSetParameters)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s should be a positive value. If this parameter"
				" is irrelevant for your purpose, just make it null."
				"\x1b[0m\n\n", textElement.c_str());
		errorSetParameters = true;
	}
	if(pwvParameters.uwvMass!=0 && pwvParameters.uwvVolume==0 && pwvParameters.uwvFloat != true)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The uwvMass was set, but the uwvVolume was not. Please set a positive"
				" value for it, or set uwvFloat: TRUE if weight and buoyancy are equal."
				"\x1b[0m\n\n");
		errorSetParameters = true;
	}
}

void DynamicModel::checkPositiveMatrices(void)
{
	std::string textElement;
	bool checkError = false;

	if (gInertiaMatrixPos == Eigen::MatrixXd::Zero(6,6))
	{
		textElement = "positive inertia matrix";
		checkError= true;
	}
	else if (gThrustConfigMatrix.size() == 0)
	{
		textElement = "thrust configuration matrix";
		checkError= true;
	}

	if(checkError)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s was not set.\x1b[0m\n\n", textElement.c_str());
		errorStatus = true;
	}
}

void DynamicModel::checkNegativeMatrices(base::Matrix6d &negativeMatrix,
		const base::Matrix6d &positiveMatrix)
{
	if (negativeMatrix == Eigen::MatrixXd::Zero(6, 6))
		negativeMatrix = positiveMatrix;
}

void DynamicModel::checkControlInput(const base::samples::Joints &controlInput,
		std::string jointsElement)
{
	int inputSize = controlInput.size();
	std::string textElement;
	bool checkError = false;

	// Checking controlInput size
	if(!errorStatus)
	{
		if(jointsElement == "effort")
		{
			if (inputSize != 6)
			{
				LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
						" The controlInput should have 6 elements, one for each"
						" degree of freedom. Currently, it has %i elements."
						" \x1b[0m\n\n", inputSize);
				errorControlInput = true;
			}
		}
		else
		{
			if (inputSize != gControlOrder)
			{
				LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
						" The system has %i control inputs, and not %i like it was"
						" provided by the controlInput argument.\x1b[0m\n\n"
						, gControlOrder, inputSize);
				errorControlInput = true;
			}
		}
	}

	// Checking if the proper field is set
	if(!errorControlInput && !errorStatus)
	{
		if(jointsElement == "effort")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(!controlInput.elements[i].hasEffort())
				{
					textElement = jointsElement;
					checkError = true;
					break;
				}
			}
		}
		else if(jointsElement == "raw")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(!controlInput.elements[i].hasRaw())
				{
					textElement = jointsElement;
					checkError = true;
					break;
				}
			}
		}
		else if(jointsElement == "speed")
		{
			for (uint i = 0; i < controlInput.size(); i++)
			{
				if(!controlInput.elements[i].hasSpeed())
				{
					textElement = jointsElement;
					checkError = true;
					break;
				}
			}
		}

		// If there was an error and the errorControlInput is still not set...
		if(checkError)
		{
			LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
					" The field %s of the controlInput was not set."
					" \x1b[0m\n\n", textElement.c_str());
			errorControlInput = true;
		}
	}
}

void DynamicModel::checkPWMCoefficients(void)
{
	std::string textElement;
	bool checkError = false;

	if(gThrusterCoeffPWM.positive +
			gLinThrusterCoeffPWM.positive +
			gQuadThrusterCoeffPWM.positive == 0)
	{
		textElement = "positive";
		checkError = true;
	}
	else if (gThrusterCoeffPWM.negative +
			gLinThrusterCoeffPWM.negative +
			gQuadThrusterCoeffPWM.negative == 0)
	{
		textElement = "negative";
		checkError = true;
	}

	// If there was an error and the errorPWMCoeff is still not set...
	if(checkError && !errorStatus)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s PWM coefficients were not set. You should set"
				" at least one of the %s PWM coefficients.\x1b[0m\n\n",
				textElement.c_str(), textElement.c_str());
		errorPWMCoeff = true;
	}
	// If there is no error anymore and the errorPWMCoeff is set...
	else if (!checkError && errorPWMCoeff)
		errorPWMCoeff = false;

	if(gThrusterVoltage == 0 && !errorStatus)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The thruster voltage should be greater than zero,"
				" but its current value is %f.\x1b[0m\n\n"
				, gThrusterVoltage);
		errorPWMCoeff = true;
	}
}

void DynamicModel::checkRPMCoefficients(void)
{
	std::string textElement;
	bool checkError = false;

	if(gThrusterCoeffRPM.positive == 0)
	{
		textElement = "positive";
		checkError = true;
	}
	else if (gThrusterCoeffRPM.negative == 0)
	{
		textElement = "negative";
		checkError = true;
	}

	// If there was an error and the errorRPMCoeff is still not set...
	if(checkError && !errorPWMCoeff)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The %s RPM coefficient was not set.\x1b[0m\n\n",
				textElement.c_str());
		errorRPMCoeff = true;
	}
	// If there is no error anymore and the errorRPMCoeff is set...
	else if (!checkError && errorRPMCoeff)
		errorRPMCoeff = false;

}

void DynamicModel::checkErrors(void)
{
	std::string textElement;
	bool checkError = false;

	if (errorModelInit && !errorStatus)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" The model wasn't initialized. Call the method initParameters"
				" in order to do so or check if there was an error while doing"
				" it.\x1b[0m\n\n");
		errorStatus = true;
	}
	else if(errorConstruction && !errorStatus)
	{
		textElement = "during the construction of the class";
		checkError = true;
	}
	else if(errorSetParameters && !errorStatus)
	{
		textElement = "while setting the model parameters";
		checkError = true;
	}
	else if(errorPWMCoeff && !errorStatus)
	{
		textElement = "with the PWM coefficients";
		checkError = true;
	}
	else if(errorRPMCoeff && !errorStatus)
	{
		textElement = "with the RPM coefficients";
		checkError = true;
	}
	else if(errorControlInput && !errorStatus)
	{
		textElement = "with the provided control input";
		checkError = true;
	}
	if(checkError && !errorStatus)
	{
		LOG_ERROR("\n\n\x1b[31m (Library: uwv_dynamic_model.cpp)"
				" There was an error %s.\x1b[0m\n\n", textElement.c_str());
		errorStatus = true;
	}

	if(!errorModelInit 		&&
			!errorConstruction 	&&
			!errorSetParameters 	&&
			!errorPWMCoeff 		&&
			!errorRPMCoeff		&&
			!errorControlInput 	&&
			errorStatus)
	{
		errorStatus = false;
	}
}
};



