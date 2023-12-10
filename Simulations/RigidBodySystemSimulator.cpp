#include "RigidBodySystemSimulator.h"
#include <cmath>
#include "collisionDetect.h"

ExternalForce::ExternalForce(Vec3 force, Vec3 position) {
	this->force = force;
	this->position = position;
}

Vec3 ExternalForce::convertToTorque(Vec3 centerOfMass) {
	Vec3 localSpacePos = position - centerOfMass;
	return cross(localSpacePos, force);
}


RigidBody::RigidBody(Vec3 position_x, Quat orientation_r, Vec3 size, float mass_m) {
	this->position_x = position_x;
	this->size = size;
	this->orientation_r = orientation_r;
	this->mass_m = mass_m;

	this->initInverse_I_0();
}

// TODO: Check if Mat3 instead of Mat4 is necessary/possible!
void RigidBody::initInverse_I_0() {
	// Specific to rectangles

	float width = size[0];
	float height = size[1];
	float depth = size[2];

	float fak = mass_m / 12;

	float I_11 = fak * (std::pow(height, 2) + std::pow(depth, 2));
	float I_22 = fak * (std::pow(width, 2) + std::pow(height, 2));
	float I_33 = fak * (std::pow(width, 2) + std::pow(depth, 2));

	double arr[16] = { I_11, 0, 0, 0, 0, I_22, 0, 0, 0, 0, I_33, 0, 0, 0, 0, 1 };

	Inverse_I_0 = Mat4();
	Inverse_I_0.initFromArray(arr);
	Inverse_I_0 = Inverse_I_0.inverse();
}

Mat4 RigidBody::getInverseInertiaTensorRotated() {
	auto rotMat = orientation_r.getRotMat();
	auto rotMat_T = orientation_r.getRotMat();
	rotMat_T.transpose();
	return rotMat * Inverse_I_0 * rotMat_T;
}

Mat4 RigidBody::getObject2WorldMatrix() {
	Mat4 scaleMat = Mat4();
	scaleMat.initScaling(size.x, size.y, size.z);

	Mat4 translatMat = Mat4();
	translatMat.initTranslation(position_x.x, position_x.y, position_x.z);

	return scaleMat * orientation_r.getRotMat() * translatMat; // scaleMat * rotMat * translatMat;
}

// TODO: Verify
Quat RigidBody::getAngularVelocityQuat()
{
	return Quat(0, angularVelocity_w.x, angularVelocity_w.y, angularVelocity_w.z);
}

void RigidBody::applyExternalForce(ExternalForce *force)
{
	externalForces.push_back(force);
}

Vec3 RigidBody::sumTotalForce_F()
{
	Vec3 out;
	for each (auto eForce in externalForces) {
		out += eForce->force;
	}

	return out;
}

Vec3 RigidBody::sumTotalTorque_q()
{
	Vec3 out;
	for each (auto eForce in externalForces) {
		out += eForce->convertToTorque(position_x);
	}

	return out;
}

Vec3 RigidBody::localToWoldPosition(Vec3 localPosition)
{
	return position_x + orientation_r.getRotMat().transformVector(localPosition);
}

Vec3 RigidBody::getTotalVelocityAtLocalPositiion(Vec3 localPosition)
{
	return linearVelocity_v + cross(angularVelocity_w, localPosition);
}

void RigidBody::printState()
{
	// linearVelocity_v; angularVelocity_w; angularMomentum_L;
	std::cout << "position x: " << position_x << endl;
	std::cout << "v: " << linearVelocity_v << endl;
	std::cout << "r: " << orientation_r << endl;
	std::cout << "L: " << angularMomentum_L << endl;
	std::cout << "w: " << angularVelocity_w << "; InvI (rot):";
	std::cout << "InvI(rot) :";
	std::cout << getInverseInertiaTensorRotated() << endl;
}



RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	timeFactor = 1.0f;

	// For Demo 2
	this->additionalExternalForce = new ExternalForce(Vec3(1, 1, 1), Vec3());
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Time Factor", TW_TYPE_FLOAT, &timeFactor, "min=0.1 step=0.1");
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
		for each (auto eForce in body->externalForces) {

			// Draw connection of force point and midpoint
			DUC->beginLine();
			DUC->drawLine(body->position_x, Vec3(255, 0, 0), eForce->position, Vec3(255, 0, 0));
			DUC->endLine();

			// Draw arrow of force

			DUC->beginLine();
			auto pointTo = eForce->position;
			auto pointFrom = pointTo - eForce->force;
			DUC->drawLine(pointFrom, Vec3(255, 255, 255), pointTo, Vec3(255, 255, 255));
			DUC->endLine();

			DUC->drawSphere(pointTo, Vec3(.02, .02, .02));
		}
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	if (testCase == 0) {
		initSingleBodySetup();
		runDemo1();
		return;
	}

	if (testCase == 1) {
		initSingleBodySetup();
		additionalExternalForce = new ExternalForce(Vec3(1, 1, 1), Vec3());
		rigidBodies[0]->applyExternalForce(additionalExternalForce);
	}
	else if (testCase == 2) {
		initTwoBodySetup();
	}
	else if (testCase == 3) {
		initManyBodySetup();
	}
	else {
		throw std::invalid_argument("Received unexpected test case: " + testCase);
	}

}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{

	if (m_iTestCase == 1) {
		// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
		Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
			// find a proper scale!
			float inputScale = 0.001f;
			inputWorld = inputWorld * inputScale;
			additionalExternalForce->position = m_vfMovableObjectFinalPos + inputWorld;
		}
		else {
			m_vfMovableObjectFinalPos = additionalExternalForce->position;
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) {
		return;
	}

	if (m_iTestCase == 2) {
		handleCollisions();
	}

	simulateTimestep_Impl(timeStep * timeFactor);
}

void RigidBodySystemSimulator::simulateTimestep_Impl(float timeStep)
{
	for each (auto body in rigidBodies) {

		/* Linear Part */

		// Integrate positon of x_cm (linear translation)
		body->position_x += timeStep * body->linearVelocity_v;
		// Integrate velocity of x_cm
		Vec3 acceleration = body->sumTotalForce_F() / body->mass_m;
		body->linearVelocity_v += timeStep * acceleration;

		/* Angular Part */

		// Integrate Orientation r
		body->orientation_r += (body->getAngularVelocityQuat() * body->orientation_r) * (timeStep / 2); // TODO: Verify
		body->orientation_r /= body->orientation_r.norm(); // Normalize

		// Integrate Angular Momentum L
		body->angularMomentum_L += body->sumTotalTorque_q() * timeStep;

		// Update angular velocity using I and L
		body->angularVelocity_w = body->getInverseInertiaTensorRotated().transformVector(body->angularMomentum_L);

		int x = 0;
	}
}


void RigidBodySystemSimulator::handleCollisions()
{
	RigidBody *A = rigidBodies[0];
	RigidBody *B = rigidBodies[1];

	auto info = checkCollisionSAT(A->getObject2WorldMatrix(), B->getObject2WorldMatrix());
	if (!info.isValid) return;

	float c = 0.1;
	const Vec3 n = info.normalWorld; // From B to A

	Vec3 x_a = info.collisionPointWorld - A->position_x;
	Vec3 x_b = info.collisionPointWorld - B->position_x;

	std::cout << "x_a: " << x_a << endl;
	std::cout << "x_b: " << x_b << endl;

	std::cout << "n: " << n << endl;
	
	Vec3 v_rel = A->getTotalVelocityAtLocalPositiion(x_a) - B->getTotalVelocityAtLocalPositiion(x_b);
	
	auto v_rel_dot_n = dot(v_rel, n);

	if (v_rel_dot_n > 0) {
		// Bodies are already separating
		return;
	}

	// Further compute formula
	Vec3 intermediate = cross(A->getInverseInertiaTensorRotated().transformVector(cross(x_a, n)), x_a) + 
		cross(B->getInverseInertiaTensorRotated().transformVector(cross(x_b, n)), x_b);
	auto J = (-(1+c) * v_rel_dot_n) / ((1 / A->mass_m) + (1 / B->mass_m) + dot(intermediate, n));

	std::cout << "J: " << J << endl;
	std::cout << "J * n : " << (J * n) << std::endl;


	/* Update */

	// Linear
	A->linearVelocity_v += J * n / A->mass_m;
	B->linearVelocity_v -= J * n / B->mass_m;

	std::cout << "v_A: " << A->linearVelocity_v << endl;
	std::cout << "v_B: " << B->linearVelocity_v << endl;


	// Angular
	A->angularMomentum_L += cross(x_a, J * n);
	B->angularMomentum_L -= cross(x_b, J * n);

	std::cout << "L_A: " << A->angularMomentum_L << endl;
	std::cout << "L_B: " << B->angularMomentum_L << endl;
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
	return rigidBodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
	return rigidBodies[i]->position_x;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i) {
	return rigidBodies[i]->linearVelocity_v;

}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
	return rigidBodies[i]->angularVelocity_w;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	rigidBodies[i]->applyExternalForce(new ExternalForce(force, loc));
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
	RigidBody *body = new RigidBody(position, Quat(), size, mass);
	rigidBodies.push_back(body);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation) {
	rigidBodies[i]->orientation_r = orientation;
}

// TODO: angular or linear velocity ?!
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
	rigidBodies[i]->angularVelocity_w = velocity;
}


// Custom functions added by us

void RigidBodySystemSimulator::runDemo1() {
	simulateTimestep_Impl(2);

	std::cout << "Linear velocity: ";
	std::cout << rigidBodies[0]->linearVelocity_v;
	std::cout << endl;

	std::cout << "Angular velocity: ";
	std::cout << rigidBodies[0]->angularVelocity_w;
	std::cout << endl;

	Vec3 testVec_1 = Vec3(0.3, 0.5, 0.25);
	Vec3 testVec_2 = Vec3(-0.3, -0.5, -0.25);

	std::cout << "World space (total) velocity of (local) point " << testVec_1 << ": ";
	std::cout << rigidBodies[0]->getTotalVelocityAtLocalPositiion(testVec_1);
	std::cout << endl;

	std::cout << "World space (total) velocity of (local) point " << testVec_2 << ": ";
	std::cout << rigidBodies[0]->getTotalVelocityAtLocalPositiion(testVec_2);
	std::cout << endl;

	std::cout << "--------" << endl;
	std::cout << "Complete state: " << endl;
	rigidBodies[0]->printState();

}

void RigidBodySystemSimulator::initSingleBodySetup() {
	rigidBodies.clear();

	Vec3 position = Vec3(0, 0, 0);
	Vec3 size = Vec3(1, 0.6, 0.5);

	Mat4 rotMat = Mat4();
	rotMat.initRotationZ(90);

	RigidBody *rect = new RigidBody(position, Quat(rotMat), size, 2);

	ExternalForce *force = new ExternalForce(Vec3(1,1,0), Vec3(0.3, 0.5, 0.25));
	rect->applyExternalForce(force);
	
	rigidBodies.push_back(rect);
}

void RigidBodySystemSimulator::initTwoBodySetup()
{
	rigidBodies.clear();

	/*
	
	rot=x grün=y blau=z

	*/

	// Right
	Vec3 position_1 = Vec3(1, 0.25, 0);
	Vec3 size_1 = Vec3(0.5, 0.5, 0.5);
	Mat4 rotMat_1 = Mat4();
	rotMat_1.initRotationXYZ(0, 45, 45);
	Quat orientation_1 = Quat(rotMat_1);
	float mass_1 = 1; //size_1.x * size_1.y * size_1.z;

	// Left 
	Vec3 position_2 = Vec3(-0.5, 0, 0);
	Vec3 size_2 = Vec3(0.1, 1, 1);
	Mat4 rotMat_2 = Mat4();
	rotMat_2.initRotationXYZ(0, 0, 0);
	Quat orientation_2 = Quat(rotMat_2);
	float mass_2 = 1; // size_2.x* size_2.y* size_2.z;

	RigidBody* body_right = new RigidBody(position_1, orientation_1, size_1, mass_1);
	RigidBody* body_left = new RigidBody(position_2, orientation_2, size_2, mass_2);

	body_right->linearVelocity_v = Vec3(-1, 0, 0);

	rigidBodies.push_back(body_right);
	rigidBodies.push_back(body_left);
}

void RigidBodySystemSimulator::initManyBodySetup()
{
	rigidBodies.clear();
	// Todo
}


