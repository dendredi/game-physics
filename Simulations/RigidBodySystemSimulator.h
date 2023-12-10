#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class ExternalForce {
public:
	Vec3 force;
	Vec3 position;

	ExternalForce(Vec3 force, Vec3 position);

	Vec3 convertToTorque(Vec3 centerOfMass);
};

class RigidBody {
public:
	Mat4 Inverse_I_0;

	Vec3 position_x;
	Vec3 size;
	Quat orientation_r;

	float mass_m;

	Vec3 linearVelocity_v;
	Vec3 angularVelocity_w;
	Vec3 angularMomentum_L;

	std::vector<ExternalForce*> externalForces;

	RigidBody(Vec3 position_x, Quat orientation_r, Vec3 size, float mass_m);
	
	Mat4 getInverseInertiaTensorRotated();
	Mat4 getObject2WorldMatrix();
	Quat getAngularVelocityQuat();

	void applyExternalForce(ExternalForce *force);
	Vec3 sumTotalForce_F();
	Vec3 sumTotalTorque_q();

	Vec3 localToWoldPosition(Vec3 localPosition);
	Vec3 getTotalVelocityAtLocalPositiion(Vec3 localPosition);

	void printState();

private:
	void initInverse_I_0();
};


class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Own stuff
	float timeFactor;

	std::vector<RigidBody*> rigidBodies;

	void initSingleBodySetup();
	void initTwoBodySetup();
	void initManyBodySetup();
	void runDemo1();
	void handleCollisions();
	void simulateTimestep_Impl(float timeStep);

	ExternalForce *additionalExternalForce;
	Vec3  m_vfMovableObjectFinalPos;


	};
#endif