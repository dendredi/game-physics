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

	ExternalForce(Vec3 force, Vec3 position) {
		this->force = force;
		this->position = position;
	}

	// TODO: Check if x_cm - position is needed
	Vec3 convertToTorque() {
		return cross(force, position);
	}
};

class RigidBody {
public:
	Mat4 Inverse_I_0;

	Mat4 position_x;
	Quat orientation_r;

	float mass_m;

	Vec3 linearVelocity_v;
	Vec3 angularVelocity_w;
	Vec3 angularMomentum_L;

	RigidBody(Mat4 position_x, Quat orientation_r, float width, float height, float depth, float mass_m);
	
	Mat4 getInverseInertiaTensor();
	Mat4 getObject2WorldMatrix();
	Quat getAngularVelocityQuat();

	void applyExternalForce(ExternalForce force);
	Vec3 sumTotalTorque();

	void printState();

private:
	void initInverse_I_0(float width, float height, float depth);
	std::vector<ExternalForce> externalForces;
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
	std::vector<RigidBody*> rigidBodies;

	void initSimpleSetup();
	void runDemo1();

	};
#endif