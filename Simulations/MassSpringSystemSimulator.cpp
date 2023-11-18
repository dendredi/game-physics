#include "MassSpringSystemSimulator.h"
#include <queue>

constexpr auto FLOOR_Y = -1;
constexpr auto MASSPOINT_RADIUS = .1;

MassPoint::MassPoint(Vec3 position, Vec3 velocity, bool isFixed, float mass) {
	this->position = position;
	this->velocity = velocity;
	this->isFixed = isFixed;
	this->mass = mass;

	clearForce(false);
}

void MassPoint::clearForce(bool isGravityEnabled) {
	this->force = isGravityEnabled ? mass * Vec3(0, -9.81, 0) : Vec3(0, 0, 0);
}

void MassPoint::applyForce(Vec3 force) {
	this->force += force;
}

Vec3 MassPoint::getAcceleration() {
	return force / mass;
}

std::string MassPoint::toString() {
	return "position = " + position.toString() + "; velocity = " + velocity.toString();
}

Spring::Spring(MassPoint* masspoint1, MassPoint* masspoint2, float initialLength, float stiffness) {
	this->masspoint1 = masspoint1;
	this->masspoint2 = masspoint2;
	this->initialLength = initialLength;
	this->stiffness = stiffness;
}

void Spring::addElasticForceToPoints() {
	Vec3 direction = masspoint1->position - masspoint2->position;
	float distance = sqrt(masspoint1->position.squaredDistanceTo(masspoint2->position));

	// Hooke's Law
	Vec3 force1to2 = -stiffness * (distance - this->initialLength) * (direction / distance);

	masspoint1->applyForce(force1to2);
	masspoint2->applyForce(-force1to2);

	/*
	
	Vec3 direction = masspoint1->position - masspoint2->position;

	// Hooke's Law
	Vec3 force1to2 = -stiffness * (normalize(direction) - this->initialLength) * getNormalized(direction);

	masspoint1->applyForce(force1to2);
	masspoint2->applyForce(-force1to2);

	*/
}

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 10.0;
	m_fStiffness = 40.0;
	m_fDamping = 0.0;
	m_iIntegrator = EULER;

	teapot = NULL;

}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1,Demo2,Demo3,Demo4,Demo5";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwEnumVal enumVals[] = {
		{ EULER, "Euler" },
		{ MIDPOINT, "Midpoint" },
	};
	TwType TW_TYPE_TESTCASE = TwDefineEnum("Sim.Meth.", enumVals, 2);
	TwAddVarRW(DUC->g_pTweakBar, "Sim.Meth.", TW_TYPE_TESTCASE, &m_iIntegrator, "");


	//TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Sim.Meth.", "Euler,Midpoint");
	//TwAddVarRW(DUC->g_pTweakBar, "Sim.Meth.", TW_TYPE_TESTCASE, &simMethDemo4, "");

}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));

	// Draw teapot
	if (teapot) {
		DUC->drawTeapot(teapot->position,Vec3(), Vec3(0.2, 0.2, 0.2));
	}
 

	// Draw mass points
	for each (auto *p in massPoints)
	{
		DUC->drawSphere(p->position, Vec3(MASSPOINT_RADIUS, MASSPOINT_RADIUS, MASSPOINT_RADIUS));
	}

	// Draw springs
	for (auto spring : this->springs) {
		DUC->beginLine();
		auto point1 = spring->masspoint1->position;
		auto point2 = spring->masspoint2->position;
		this->DUC->drawLine(point1, Vec3(255, 255, 255), point2, Vec3(255, 255, 255));
		this->DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	// Reset, so console log is activated again for next demo 
	isFirstStep = true;

	if (testCase == 0) {
		isGravityEnabled = false;
		isCollisionEnabled = false;
		m_iIntegrator = EULER;
		runDemo1();
		return;
	}

	if (testCase == 2 || testCase == 3) {
		m_iIntegrator = MIDPOINT;
	}
	else {
		m_iIntegrator = EULER;
	}

	if (testCase == 3) {
		isGravityEnabled = true;
		isCollisionEnabled = true;
		setupComplexEnvironment();
	}
	else {
		isGravityEnabled = false;
		isCollisionEnabled = false;
		setupSimpleEnvironment();
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	if (teapot) {
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
			teapot->position = m_vfMovableObjectFinalPos + inputWorld;
		}
		else {
			m_vfMovableObjectFinalPos = teapot->position;
		}
	}
}

void MassSpringSystemSimulator::integrateEuler(float timeStep) {
		for each (MassPoint * p in massPoints) {
			// Integrate Position
			p->position += timeStep * p->velocity;
			// Integrate Velocity
			p->velocity += timeStep * p->getAcceleration();
		}
}

void MassSpringSystemSimulator::integrateLeapfrog(float timeStep) {
	// TODO
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep) {
	std::queue<Vec3> initialPositions;
	std::queue<Vec3> initialVelocities;

	for each (MassPoint * p in massPoints) {
		Vec3 initialVelocity = p->velocity;
		Vec3 initialPosition = p->position;

		initialPositions.push(initialPosition);
		initialVelocities.push(initialVelocity);

		// Approximate midpoints
		p->position += (timeStep / 2) * p->velocity;
		p->velocity += (timeStep / 2) * p->getAcceleration();

		p->clearForce(isGravityEnabled);
	}

	for each (Spring * s in springs) s->addElasticForceToPoints();

	for each (MassPoint * p in massPoints) {
		// Compute derivatives at midpoints
		p->position = initialPositions.front() + timeStep * p->velocity;
		p->velocity = initialVelocities.front() + timeStep * p->getAcceleration();

		initialPositions.pop();
		initialVelocities.pop();
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_iTestCase == 0) {
		return;
	}

	for each (MassPoint * p in massPoints) p->clearForce(isGravityEnabled);
	for each (Spring *s in springs) s->addElasticForceToPoints();

	switch (m_iIntegrator)
	{
	case EULER: integrateEuler(timeStep);
		break;
	case LEAPFROG: integrateLeapfrog(timeStep);
		break;
	case MIDPOINT: integrateMidpoint(timeStep);
		break;
	default: integrateEuler(timeStep);
		break;
	}

	if (isFirstStep) {
		printMasspointStates();
		isFirstStep = false;
	}

	if (isCollisionEnabled) {
		handleCollisions();
	}
}

void MassSpringSystemSimulator::handleCollisions() {
	for each (MassPoint * p in massPoints) {
		Real minY = FLOOR_Y + MASSPOINT_RADIUS;
		if (p->position.y < minY) {
			p->position.y = minY;
		}
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// unsure if the following setter methods are supposed to be more or not

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
	for each (MassPoint * p in massPoints) p->mass = mass;

}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
	for each (Spring * s in springs) s->stiffness = stiffness;

}


void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	massPoints.push_back(new MassPoint(position, Velocity, isFixed, m_fMass));
	return massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	springs.push_back(new Spring(massPoints[masspoint1], massPoints[masspoint2], initialLength, m_fStiffness));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return massPoints[index]->position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return massPoints[index]->velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	externalForces.push_back(force); 
}


// Custom functions added by us

void MassSpringSystemSimulator::resetEnvironment()
{
	massPoints.clear();
	springs.clear();
	externalForces.clear();
	teapot = NULL;
}

void MassSpringSystemSimulator::setupSimpleEnvironment() {
	resetEnvironment();

	addSpring(
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false),
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false),
		1
	);	
}

void MassSpringSystemSimulator::setupComplexEnvironment() {
	resetEnvironment();

	//teapot = new MassPoint(Vec3(0, 1.5, 0), Vec3(), false, 10);	
	//int root = addMassPoint(Vec3(0, 1.25, 0), Vec3(), false);
	//addSpringToTeapot(root, 0.1, 100);

	int p1 = addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p2 = addMassPoint(Vec3(1.0f, 2.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p3 = addMassPoint(Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p4 = addMassPoint(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p5 = addMassPoint(Vec3(1.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p6 = addMassPoint(Vec3(2.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p7 = addMassPoint(Vec3(-1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p8 = addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p9 = addMassPoint(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p10 = addMassPoint(Vec3(2.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p11 = addMassPoint(Vec3(0.0f, -1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);
	int p12 = addMassPoint(Vec3(1.0f, -1.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), false);

	addSpring(p1, p2, 1);
	addSpring(p1, p4, 1);
	addSpring(p2, p5, 1);
	addSpring(p3, p4, 1);
	addSpring(p4, p5, 1);
	addSpring(p5, p6, 1);
	addSpring(p3, p7, 1);
	addSpring(p4, p8, 1);
	addSpring(p5, p9, 1);
	addSpring(p6, p10, 1);
	addSpring(p7, p8, 1);
	addSpring(p8, p9, 1);
	addSpring(p9, p10, 1);
	addSpring(p8, p11, 1);
	addSpring(p9, p12, 1);
	addSpring(p11, p12, 1);

	teapot = new MassPoint(Vec3(0, 1.5, 0), Vec3(), false, 10);
	addSpringToTeapot(p1, 0.1, 100);
	addSpringToTeapot(p4, 0.1, 100);
	addSpringToTeapot(p8, 0.1, 100);
	addSpringToTeapot(p12, 0.1, 100);
}

void MassSpringSystemSimulator::printMasspointStates() {
	for (int i = 0; i < massPoints.size(); ++i) std::cout << "Point " + std::to_string(i) + ": " + massPoints[i]->toString() + "\n";
}

void MassSpringSystemSimulator::addSpringToTeapot(int masspoint, float initialLength, float stiffness)
{
	springs.push_back(new Spring(massPoints[masspoint], teapot, initialLength, stiffness));
}

void MassSpringSystemSimulator::runDemo1() {
	setupSimpleEnvironment();
	for each (MassPoint * p in massPoints) p->clearForce(isGravityEnabled);
	for each (Spring * s in springs) s->addElasticForceToPoints();
	integrateEuler(0.1);
	std::cout << "Euler:" << std::endl;
	printMasspointStates();

	setupSimpleEnvironment();
	for each (MassPoint * p in massPoints) p->clearForce(isGravityEnabled);
	for each (Spring * s in springs) s->addElasticForceToPoints();
	integrateMidpoint(0.1);
	std::cout << "Midpoint:" << std::endl;
	printMasspointStates();
}

