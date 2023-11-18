#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassPoint {
public:
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
	float mass;
	Vec3 force;

	MassPoint(Vec3 position, Vec3 velocity, bool isFixed, float mass);
	void clearForce(bool isGravityEnabled);
	void applyForce(Vec3 force);
	Vec3 getAcceleration();
	std::string toString();
};

class Spring {
public:
	MassPoint* masspoint1;
	MassPoint* masspoint2;
	float initialLength;
	float stiffness;

	Spring(MassPoint* masspoint1, MassPoint* masspoint2, float initialLength, float stiffness);
	void addElasticForceToPoints();
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// Custom stuff added by us
	std::vector<MassPoint*> massPoints;
	std::vector<Spring*> springs;
	std::vector<Vec3> externalForces;
	bool isFirstStep = true; // Only needed in order to print only the first step's results to the console.
	bool isGravityEnabled = false;
	bool isCollisionEnabled = false;

	void resetEnvironment();
	void setupSimpleEnvironment();
	void setupComplexEnvironment();
	void integrateEuler(float timeStep);
	void integrateMidpoint(float timeStep);
	void integrateLeapfrog(float timeStep);
	void handleCollisions();
	void printMasspointStates();

	MassPoint *teapot;
	Vec3  m_vfMovableObjectFinalPos;
	void addSpringToTeapot(int masspoint, float initialLength, float stiffness);
	// Custom stuff added by us

};
#endif