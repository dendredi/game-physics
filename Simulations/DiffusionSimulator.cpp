#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

#define INIT_M 32
#define INIT_N 32

#define ALPHA 15
#define INIT_HIGH_TEMP 10

void GridPixel::update() {
	float value = grid->get(x, y);

	float scaleX = 1.0 / grid->cols;
	float scaleY = 1.0 / grid->rows;

	float normalizedValue = (value - normInterval.first) / (normInterval.second - normInterval.first);

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

GridPixel::GridPixel(Grid *grid, int x, int y, std::pair<float, float> normInterval) : grid(grid), x(x), y(y), normInterval(normInterval) {
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

Grid::Grid(int numRows, int numCols, float *initMatrix) : Grid(numRows, numCols) {
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
float Grid::get(int row, int col) const {
	return matrix.at(row * cols + col);
}

void Grid::set(int row, int col, float value) {
	matrix.at(row * cols + col) = value;
}

std::pair<float, float> Grid::getValueInterval()
{
	float min = matrix[0], max = matrix[0];

	for each (float value in matrix)
	{
		if (value < min) min = value;
		if (value > max) max = value;
	}

	return std::pair<float, float>(min, max);
}

Grid Grid::convolution(Grid window)
{
	Grid out(rows - window.rows + 1, cols - window.cols + 1);
	for (int out_i = 0; out_i < out.rows; ++out_i) {
		for (int out_j = 0; out_j < out.cols; ++out_j) {
			float value = 0;
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

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	updateDimensions(T.rows, T.cols); // TODO: Change this.

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
	float window[9] = {0.0, 1.0, 0.0, 1.0, -4.0, 1.0, 0.0, 1.0, 0.0};
	Grid laplace = T.convolution(Grid(3, 3, window));

	for (int i = 0; i < T.rows; ++i) {
		for (int j = 0; j < T.cols; ++j) {
			if (i == 0 || j == 0 || i == T.rows - 1 || j == T.cols - 1) {
				// Do not touch borders
			}
			else {
				float currentValue = T.get(i, j);
				float time_derivative_ij = ALPHA * laplace.get(i - 1, j - 1);
				float newValue = currentValue + time_derivative_ij * timeStep;
				T.set(i, j, newValue);
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {
	// solve A T = b

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

void DiffusionSimulator::drawObjects()
{
	for each (auto pixel in pixels)
	{
		pixel->draw(DUC);
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
