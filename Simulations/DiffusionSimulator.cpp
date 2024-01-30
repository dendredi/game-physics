#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

#define INIT_M 32
#define INIT_N 32

#define ALPHA 15
#define INIT_HIGH_TEMP 10

// --- Rigid body ---

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

// 0 at last position!!
Quat RigidBody::getAngularVelocityQuat()
{
	return Quat(angularVelocity_w.x, angularVelocity_w.y, angularVelocity_w.z, 0);
}

void RigidBody::applyExternalForce(ExternalForce* force)
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

Vec3 DiffusionSimulator::getPositionOfRigidBody(int i) {
	return rigidBodies[i]->position_x;
}

void DiffusionSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
	rigidBodies[i]->applyExternalForce(new ExternalForce(force, loc));
}

void DiffusionSimulator::initSetup_RB() {
	rigidBodies.clear();

	Vec3 position = Vec3(-1, 1, 0);
	Vec3 size = Vec3(.1, 0.1, 0.1);

	Mat4 rotMat = Mat4();
	rotMat.initRotationZ(90);

	RigidBody* rect = new RigidBody(position, Quat(rotMat), size, 2);

	Vec3 force_dir = Vec3(.5, -1, 0) * 2;

	// Initial velocity
	rect->linearVelocity_v = force_dir;

	ExternalForce* force = new ExternalForce(force_dir, Vec3(-1.1, 1, 0));
	rect->applyExternalForce(force);

	rigidBodies.push_back(rect);
}

Quat normalzeQuat(Quat quaternion) {
	auto norm = quaternion.norm();
	if (norm > 0) {
		quaternion /= norm;
	}
	return quaternion;
}

void DiffusionSimulator::simulateTimestep_RB(float timeStep)
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
		Quat wr = body->getAngularVelocityQuat() * body->orientation_r;
		body->orientation_r += timeStep / 2 * wr; // TODO: Verify
		body->orientation_r = normalzeQuat(body->orientation_r);

		// Integrate Angular Momentum L
		body->angularMomentum_L += body->sumTotalTorque_q() * timeStep;

		// Update angular velocity using I and L
		body->angularVelocity_w = body->getInverseInertiaTensorRotated().transformVector(body->angularMomentum_L);
	}
}

void DiffusionSimulator::handleCollisions()
{
	for (int i = 0; i < rigidBodies.size(); i++) {
		for (int j = 0; j < i; j++) {
			handleOneCollision(i, j);
		}
	}
}



void DiffusionSimulator::handleOneCollision(int indexA, int indexB)
{
	
	RigidBody* A = rigidBodies[indexA];
	RigidBody* B = rigidBodies[indexB];

	auto info = checkCollisionSAT(A->getObject2WorldMatrix(), B->getObject2WorldMatrix());
	if (!info.isValid) return;

	float c = 0.1;
	const Vec3 n = info.normalWorld; // From B to A

	Vec3 x_a = info.collisionPointWorld - A->position_x;
	Vec3 x_b = info.collisionPointWorld - B->position_x;

	Vec3 v_rel = A->getTotalVelocityAtLocalPositiion(x_a) - B->getTotalVelocityAtLocalPositiion(x_b);

	auto v_rel_dot_n = dot(v_rel, n);

	if (v_rel_dot_n > 0) {
		// Bodies are already separating
		return;
	}

	// Further compute formula
	Vec3 intermediate = cross(A->getInverseInertiaTensorRotated().transformVector(cross(x_a, n)), x_a) +
		cross(B->getInverseInertiaTensorRotated().transformVector(cross(x_b, n)), x_b);
	auto J = (-(1 + c) * v_rel_dot_n) / ((1 / A->mass_m) + (1 / B->mass_m) + dot(intermediate, n));
	
	// Update

	Vec3 Jn = J * n;

	// Linear
	A->linearVelocity_v += Jn / A->mass_m;
	B->linearVelocity_v -= Jn / B->mass_m;

	// Angular
	A->angularMomentum_L += cross(x_a, Jn);
	B->angularMomentum_L -= cross(x_b, Jn);

	
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	

	if (!chargingForce) {
		chargingForce = true;

		if (duringCreationRigidBody == nullptr) {
			// get coords in worldspace from screen space coord :O
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			
			// Todo get actual screen width and height
			/*
			RECT rect;
			if (GetWindowRect(hwnd, &rect))
			{
				int width = rect.right - rect.left;
				int height = rect.bottom - rect.top;
			}*/

			Vec3 position = Vec3(x, y, 0);
			// Vec3 halfScreen = Vec3(width/2, height/2, 1);
			Vec3 halfScreen = Vec3(630, 320, 1);

			Vec3 homoneneousPosition = (position - halfScreen) / halfScreen;
			// so sry about magic numbers, but that's what these are ... :D
			//homoneneousPosition.z = 2;
			//homoneneousPosition.y = -0.8 * homoneneousPosition.y;
			//homoneneousPosition.x = 1.65 * homoneneousPosition.x;
			homoneneousPosition.z = 1.5;
			homoneneousPosition.y = -0.4 * homoneneousPosition.z * homoneneousPosition.y;
			homoneneousPosition.x = 0.77 * homoneneousPosition.z * homoneneousPosition.x;

			std::cout << "position : " << position << "; homoneneous position: " << homoneneousPosition << std::endl;

			Vec3 worldPosition = worldViewInv.transformVector(homoneneousPosition);
			

			Mat4 rotation = Mat4();

			duringCreationRigidBody = new RigidBody(worldPosition, Quat(rotation), Vec3(0.1, 0.1, 0.1), 0.1f);
			std::cout << "world position: " << duringCreationRigidBody->position_x << std::endl;
		}
		else if (duringCreationRigidBody != nullptr) {
			// Todo change speed preview and tempereateur witch happens to be same thing xD
		}
	}
}

void DiffusionSimulator::onMouse(int x, int y)
{
	if (chargingForce) {
		chargingForce = false;

		// a new rigid bodie is born
		int id = rigidBodies.size();
		rigidBodies.push_back(duringCreationRigidBody);

		// calculate
		Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
		if (mouseDiff.x != 0 || mouseDiff.y != 0)
		{
			Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
			worldViewInv = worldViewInv.inverse();
			Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
			Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
			//std::cout << "Force Vec in world coords: " << inputWorld << std::endl;
			 
			applyForceOnBody(id, getPositionOfRigidBody(id), inputWorld * -0.01);
			// Todo this Force seems to just keep existing, it should'nt?
		}

		duringCreationRigidBody = nullptr;
	}

	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// --- PDE ---

void GridPixel::update() {
	Real value = grid->get(x, y);

	Real scaleX = 1.0 / grid->cols;
	Real scaleY = 1.0 / grid->rows;

	Real normalizedValue = (value - normInterval.first) / (normInterval.second - normInterval.first);

	Mat4 posMat = Mat4();
	posMat.initTranslation(
		x * scaleX - 0.5 + (scaleX * 0.5),
		normalizedValue * 0.5 - 0.5,
		y * scaleY - 0.5 + (scaleY * 0.5)
	);

	Mat4 sizeMat = Mat4();
	sizeMat.initScaling(scaleX, normalizedValue, scaleY);

	object2WorldMatrix = sizeMat * posMat;
	color = Vec3(normalizedValue, 0, 1.0f - normalizedValue);
}

void GridPixel::draw(DrawingUtilitiesClass* DUC)
{
	DUC->setUpLighting(color, color, 100, color);
	DUC->drawRigidBody(object2WorldMatrix);
}

std::vector<GridPixel*> GridPixel::initPixelsFromGrid(Grid* grid)
{
	std::vector<GridPixel*> pixels;

	auto normInterval = grid->getValueInterval();

	for (int i = 0; i < grid->rows; ++i) {
		for (int j = 0; j < grid->cols; ++j) {
			pixels.push_back(new GridPixel(grid, i, j, normInterval));
		}
	}

	return pixels;
}

GridPixel::GridPixel(Grid *grid, int x, int y, std::pair<Real, Real> normInterval) : grid(grid), x(x), y(y), normInterval(normInterval) {
	update();
}

// --

Grid::Grid() : rows(0), cols(0) {
}

// Constructor to initialize the matrix with dynamic size
Grid::Grid(int numRows, int numCols) : rows(numRows), cols(numCols) {
	// Allocate memory for rows
	matrix.clear();
	matrix.assign(numRows * numCols, 0);
}

Grid::Grid(int numRows, int numCols, Real *initMatrix) : Grid(numRows, numCols) {
	for (int i = 0; i < numRows; ++i) {
		for (int j = 0; j < numCols; ++j) {
			matrix.at(i * cols + j) = initMatrix[i * cols + j];
		}
	}
}

Grid::~Grid() {
	//delete matrix;
}

// Accessor and mutator functions for the matrix elements
Real Grid::get(int row, int col) const {
	return matrix.at(row * cols + col);
}

void Grid::set(int row, int col, Real value) {
	matrix.at(row * cols + col) = value;
}

Grid Grid::operator*(const Real scalar) const {
	Grid result(rows, cols);
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			result.set(i, j, get(i, j) * scalar);
		}
	}
	return result;
}

std::pair<Real, Real> Grid::getValueInterval()
{
	Real min = matrix[0], max = matrix[0];

	for each (Real value in matrix)
	{
		if (value < min) min = value;
		if (value > max) max = value;
	}

	return std::pair<Real, Real>(min, max);
}

Grid Grid::convolution(Grid window)
{
	Grid out(rows - window.rows + 1, cols - window.cols + 1);
	for (int out_i = 0; out_i < out.rows; ++out_i) {
		for (int out_j = 0; out_j < out.cols; ++out_j) {
			Real value = 0;
			for (int w_i = 0; w_i < window.rows; ++w_i) {
				for (int w_j = 0; w_j < window.cols; ++w_j) {
					value += window.get(w_i, w_j) * get(out_i + w_i, out_j + w_j);
				}
			}
			out.set(out_i, out_j, value);
		}
	}
	return out;
}

std::string Grid::to_string()
{
	std::string out = "";
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			out += std::to_string(get(i, j));
			out += "\t";
		}
		out += "\n";
	}

	return out;
}

std::vector<Real> Grid::to_vector()
{
	return matrix; // TODO: CHECK!!!
}

void Grid::update_from_vector(std::vector<Real> newVector)
{
	matrix = newVector;
}

// --

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	updateDimensions(INIT_M, INIT_N); // TODO: Change this.
	
	/*
	float matrixArr[25] = { 
		3.0, 3.0, 2.0, 1.0, 0.0,
		0.0, 0.0, 1.0, 3.0, 1.0,
		3.0, 1.0, 2.0, 2.0, 3.0,
		2.0, 0.0, 0.0, 2.0, 2.0,
		2.0, 0.0, 0.0, 0.0, 1.0,
	};
	float windowArr[9] = { 0.0, 1.0, 2.0, 2.0, 2.0, 0.0, 0.0, 1.0, 2.0 };

	Grid matrix = Grid(5, 5, matrixArr);
	Grid window = Grid(3, 3, windowArr);
	Grid result = matrix.convolution(window);

	std::cout << "Matrix:" << endl << matrix.to_string();
	std::cout << "Window:" << endl << window.to_string();
	std::cout << "Result:" << endl << result.to_string();
	*/
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::callbackSetM(const void* value, void* clientData) {
	//updateDimensions(value, T.cols);
}

void DiffusionSimulator::callbackSetN(const void* value, void* clientData) {
	//updateDimensions(T.rows, value);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	updateDimensions(T.rows, T.cols);

	// TODO
	//TwAddVarCB(DUC->g_pTweakBar, "m", TW_TYPE_INT32, callbackSetM, NULL, NULL, "min=10 max=100");

	initSetup_RB();
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Real window[9] = {0.0, 1.0, 0.0, 1.0, -4.0, 1.0, 0.0, 1.0, 0.0};
	Grid laplace = T.convolution(Grid(3, 3, window)) * (1.0f / timeStep * timeStep); // TODO: Check !!! 

	for (int i = 0; i < T.rows; ++i) {
		for (int j = 0; j < T.cols; ++j) {
			if (i == 0 || j == 0 || i == T.rows - 1 || j == T.cols - 1) {
				// Do not touch borders
			}
			else {
				Real currentValue = T.get(i, j);
				Real time_derivative_ij = ALPHA * laplace.get(i - 1, j - 1);
				Real newValue = currentValue + time_derivative_ij * timeStep;
				T.set(i, j, newValue);
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {
	// solve A x = b

	const int N = T.cols * T.rows;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	Real lambda = ALPHA * timeStep / (1 * 1); // TODO: WTF

	/*
	assemble the system matrix A 
	*/
	
	// First row
	std::vector<int> indices{ 0, 1 };
	std::vector<Real> values{ 1.0f + 2.0f * lambda, - lambda };
	A.add_sparse_row(0, indices, values);

	// Last row
	indices = std::vector<int>{ N-2, N-1 };
	values = std::vector<Real>{ -lambda,  1.0f + 2.0f * lambda };
	A.add_sparse_row(N - 1, indices, values);

	// In between
	values = std::vector<Real>{ -lambda,  1.0f + 2.0f * lambda, -lambda };
	for (int i = 1; i < N - 1; ++i) {
		indices = std::vector<int>{ i - 1, i, i + 1 };
		A.add_sparse_row(i, indices, values);
	}

	/*
	assemble the right - hand side b
	*/

	b = T.to_vector();


	/* --- Do not touch --- */
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	/* --- Do not touch --- */


	T.update_from_vector(x);
}

void diffuseTemperatureImplicit_TEMPLATE(float timeStep) {
	// solve A x = b

	// This is just an example to show how to work with the PCG solver,
	const int nx = 5;
	const int ny = 5;
	const int nz = 5;
	const int N = nx * ny * nz;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);

	// This is the part where you have to assemble the system matrix A and the right-hand side b!

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	// Final step is to extract the grid temperatures from the solution vector x
	// to be implemented
}

void DiffusionSimulator::updateDimensions(int m, int n)
{
	this->T = Grid(m, n); // All 0
	
	for (int i = 0.5 * T.rows; i < 0.8 * T.rows; ++i) {
		for (int j = 0.5 * T.cols; j < 0.8 * T.cols; ++j) {
			T.set(i, j, INIT_HIGH_TEMP);
		}
	}
	
	this->pixels = GridPixel::initPixelsFromGrid(&T); // TODO Check if this is ok
}

void DiffusionSimulator::updatePixels()
{
	for each (auto pixel in pixels) pixel->update();
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	simulateTimestep_PDE(timeStep);
	simulateTimestep_RB(timeStep);
}

void DiffusionSimulator::simulateTimestep_PDE(float timeStep) {
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit(timeStep);
		break;
	}

	updatePixels();
}


void DiffusionSimulator::drawObjects_PDE()
{
	for each (auto pixel in pixels)
	{
		pixel->draw(DUC);
	}
}

void DiffusionSimulator::drawObjects_RB() {
	for (int i = 0; i < rigidBodies.size(); ++i) {
		auto body = rigidBodies.at(i);
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
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

	if (duringCreationRigidBody != nullptr) {
		auto body = duringCreationRigidBody;
		// Todo get Color from Temperature
		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(body->getObject2WorldMatrix());
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects_PDE();
	drawObjects_RB();
}
