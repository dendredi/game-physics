#include "RigidBodySystemSimulator.h"
#include <cmath>


RigidBody::RigidBody(Mat4 position_x, Quat orientation_r, float width, float height, float depth, float mass_m) {
	this->position_x = position_x;
	this->orientation_r = orientation_r;
	this->mass_m = mass_m;

	this->initInverse_I_0(width, height, depth);
}

void RigidBody::initInverse_I_0(float width, float height, float depth) {
	// Specific to rectangles
	float fak = mass_m / 12;

	float I_11 = fak * (std::pow(height, 2) + std::pow(depth, 2));
	float I_22 = fak * (std::pow(width, 2) + std::pow(height, 2));
	float I_33 = fak * (std::pow(width, 2) + std::pow(depth, 2));

	double arr[16] = { I_11, 0, 0, 0, 0, I_22, 0, 0, 0, 0, I_33, 0, 0, 0, 0, 1 };

	Inverse_I_0 = Mat4();
	Inverse_I_0.initFromArray(arr);
	Inverse_I_0 = Inverse_I_0.inverse(); // Debug !

	std::cout << "Inverse_I_0: ";
	std::cout << Inverse_I_0;
	std::cout << endl;

}

Mat4 RigidBody::getInverseInertiaTensor() {
	auto rotMat = orientation_r.getRotMat();
	return rotMat * Inverse_I_0 * rotMat.inverse(); // TODO Transpose !?
}

Mat4 RigidBody::getObject2WorldMatrix() {
	return orientation_r.getRotMat() * position_x; // scaleMat * rotMat * translatMat;
}

Quat RigidBody::getAngularVelocityQuat()
{
	return Quat(0, angularVelocity_w.x, angularVelocity_w.y, angularVelocity_w.z);
}

void RigidBody::applyExternalForce(ExternalForce force)
{
	externalForces.push_back(force);
}

Vec3 RigidBody::sumTotalTorque()
{
	Vec3 out;
	for each (auto force in externalForces) {
		out += force.convertToTorque();
	}

	return out;
}

void RigidBody::printState()
{
	// linearVelocity_v; angularVelocity_w; angularMomentum_L;
	std::cout << "RigidBody state - x: " << endl;
	std::cout << position_x << "; r: " << orientation_r;
	std::cout << "; L: " << angularMomentum_L << "; w: " << angularVelocity_w << "; InvI (rot):";
	std::cout << getInverseInertiaTensor();
	std::cout << "; v: " << linearVelocity_v;
	std::cout << endl;

}



RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iTestCase == 0) {
		return;
	}

	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for each(auto body in rigidBodies) {
		DUC->drawRigidBody(body->getObject2WorldMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	if (testCase == 0) {
		initSimpleSetup();
		runDemo1();
		return;
	}

	initSimpleSetup();

	if (testCase == 1) {
		// ...
	}

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) {
		return;
	}

	for each (auto body in rigidBodies) {

		//body->printState();

		// Integrate Orientation
		body->orientation_r += (body->getAngularVelocityQuat() * body->orientation_r) * (timeStep / 2);
		body->orientation_r /= body->orientation_r.norm(); // Normalize

		// Integrate Angular Momentum 
		body->angularMomentum_L += body->sumTotalTorque() * timeStep;
			
		// Update angular velocity using I and L
		body->angularVelocity_w = body->getInverseInertiaTensor() * body->angularMomentum_L;
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// Getters / Setters

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
	return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return Vec3();

}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return Vec3();

}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {

}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {

}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {

}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {

}


// Custom functions added by us

void RigidBodySystemSimulator::runDemo1() {
	simulateTimestep(2);
	for each (auto body in rigidBodies) body->printState();
}

void RigidBodySystemSimulator::initSimpleSetup() {
	rigidBodies.clear();


	Mat4 position = Mat4();
	position.initTranslation(0,0,0);

	Mat4 rotMat = Mat4();
	rotMat.initRotationZ(90);

	RigidBody *rect = new RigidBody(position, Quat(rotMat), 1, 0.6, 0.5, 2);

	rigidBodies.push_back(rect);

	ExternalForce force = ExternalForce(Vec3(1,1,0), Vec3(0.3, 0.5, 0.25));
	rect->applyExternalForce(force);
}
