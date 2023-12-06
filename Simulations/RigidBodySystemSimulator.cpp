#include "RigidBodySystemSimulator.h"

RigidBody::RigidBody(Vec3 position, Quat rotation, float width, float height, float depth) {
	this->position = position;
	this->rotation = rotation;

	this->init_I_0(width, height, depth);
}

void RigidBody::init_I_0(float width, float height, float depth) {
	// Specific to rectangles
	I_0 = Mat4(); // TODO!
}

Mat4 RigidBody::getObject2WorldMatrix() {
	return Mat4(); // rotation.getRotMat()* position; // scaleMat * rotMat * translatMat;
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
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for each(auto body in rigidBodies) {
		DUC->drawRigidBody(body->getObject2WorldMatrix());
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	if (testCase == 0) {
		runDemo1();
		return;
	}

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

}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {

}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {

}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {

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
	// TODO
}
