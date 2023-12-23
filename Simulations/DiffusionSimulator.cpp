#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

#define INIT_M 32
#define INIT_N 32

#define ALPHA 15
#define INIT_HIGH_TEMP 10

void GridPixel::update() {
	Real value = grid->get(x, y);

	Real scaleX = 1.0 / INIT_N;
	Real scaleY = 1.0 / INIT_M;

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

	alpha = ALPHA;

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

	TwAddVarRW(DUC->g_pTweakBar, "m", TW_TYPE_INT32, &newColumSize, "min=10 max=100");
	TwAddVarRW(DUC->g_pTweakBar, "n", TW_TYPE_INT32, &newRowSize, "min=10 max=100");
	TwAddButton(DUC->g_pTweakBar, "set alpha to 3000 for unstable test", NULL, NULL, "");
	TwAddVarRW(DUC->g_pTweakBar, "alpha", TW_TYPE_INT32, &alpha, "min=5 max=5000");


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
		//alpha = 300;
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Real window[9] = {0.0, 1.0, 0.0, 1.0, -4.0, 1.0, 0.0, 1.0, 0.0};
	Grid laplace = T.convolution(Grid(3, 3, window));

	for (int i = 0; i < T.rows; ++i) {
		for (int j = 0; j < T.cols; ++j) {
			if (i == 0 || j == 0 || i == T.rows - 1 || j == T.cols - 1) {
				// Do not touch borders
			}
			else {
				Real currentValue = T.get(i, j);
				Real time_derivative_ij = alpha * laplace.get(i - 1, j - 1);
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

	Real lambda = alpha * timeStep; 

	/*
	assemble the system matrix A 
	*/
	
	/*
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
	*/

	int setter;
	for (int i = 0; i < N; i++) {

		int j = i % T.cols;
		//int k = (i - j) / T.cols;
		//std::cout << j << "|" << k << std::endl;

		A.set_element(i, i, (1.0f + 4.0f * lambda)); // 2 2

		if (i != 0 && j != 0) {
			A.set_element(i, i - 1, (-lambda));	// 2 1
		}

		if (i != N - 1 && j != N - 1) {
			A.set_element(i, i + 1, (-lambda)); // 2 3
		}

		setter = i - T.cols;
		if (setter >= 0) {
			A.set_element(i, setter, (-lambda)); // 1 2
		}

		setter = i + T.cols; 
		if (setter < N) {
			A.set_element(i, setter, (-lambda)); // 3 2
		}

		/*
		setter = i - 1; //1
		if (setter >= 0)
			A.set_element(setter, i, (-lambda));//richtige coordinate offset? // 1 2
		setter = i - T.cols;//richtige coordinate offset? // 2 - m ?? - value
		if (setter >= 0) // no
			A.set_element(setter, i, (-lambda));      // no
		setter = i + 1; // 3
		if (setter < N)
			A.set_element(setter, i, (-lambda));
		setter = i + T.cols;
		if (setter < N)
			A.set_element(setter, i, (-lambda));
			*/
	}

	//A.set_element(0, 0, 1.0f);
	//A.set_element(N - 1, N - 1, 1.0f);

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

	// null for boundary
	for (int i = 0; i < T.rows; i++) {
		T.set(i, 0, 0.0f);
		T.set(i, T.cols - 1, 0.0f);
	}
	for (int j = 0; j < T.cols; j++) {
		T.set(0, j, 0.0f);
		T.set(T.rows - 1, j, 0.0f);
	}
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

	if (T.rows != newRowSize || T.cols != newColumSize) {
		updateDimensions(newRowSize, newColumSize);
	}
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
