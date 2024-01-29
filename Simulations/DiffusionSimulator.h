#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"
//#include "collisionDetect.h"

// --- Rigid Body ---

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

	void applyExternalForce(ExternalForce* force);
	Vec3 sumTotalForce_F();
	Vec3 sumTotalTorque_q();

	Vec3 localToWoldPosition(Vec3 localPosition);
	Vec3 getTotalVelocityAtLocalPositiion(Vec3 localPosition);

	void printState();

private:
	void initInverse_I_0();
};

// --- PDE ---

class Grid {
private:
	std::vector<Real> matrix;
public:
	int rows; // y
	int cols; // x

	Grid();
	Grid(int numRows, int numCols);
	Grid(int numRows, int numCols, Real *matrix);

	~Grid();

	Real get(int row, int col) const;
	void set(int row, int col, Real value);

	// Overloading the + operator
	Grid operator*(const Real scalar) const;

	// Min and max values
	std::pair<Real, Real> getValueInterval();

	Grid convolution(Grid window);

	std::string to_string();

	std::vector<Real> to_vector();
	void update_from_vector(std::vector<Real>);
};

class GridPixel {
private:
	Grid *grid;
	int x, y;
	std::pair<Real, Real> normInterval;
	Mat4 object2WorldMatrix;
	Vec3 color;

public:
	GridPixel(Grid* grid, int x, int y, std::pair<Real, Real> normInterval);
	void update();
	void draw(DrawingUtilitiesClass *DUC);

	static std::vector<GridPixel*> initPixelsFromGrid(Grid* grid);
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void simulateTimestep_PDE(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	// Specific Functions
	void drawObjects_PDE();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

	void updateDimensions(int m, int n);

	void callbackSetM(const void* value, void* clientData);
	void callbackSetN(const void* value, void* clientData);

	// --- Rigid Bodies ---
	void initSetup_RB();
	std::vector<RigidBody*> rigidBodies;
	void drawObjects_RB();
	Vec3 getPositionOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void handleCollisions();
	void handleOneCollision(int indexA, int indexB);
	void simulateTimestep_RB(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	bool chargingForce = false;
	// --- Rigid Bodies ---

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;

	int newColumSize = 32;
	int newRowSize = 32;

	float alpha;
	

	std::vector<GridPixel*> pixels;
	void updatePixels();
};

#endif