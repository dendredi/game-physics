#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
private:
	int rows;
	int cols;
	float** matrix; // Pointer to a pointer for 2D array

public:

	Grid() : rows(0), cols(0) {

	}

	// Constructor to initialize the matrix with dynamic size
	Grid(int numRows, int numCols) : rows(numRows), cols(numCols) {
		// Allocate memory for rows
		matrix = new float* [rows];

		// Allocate memory for each column
		for (int i = 0; i < rows; ++i) {
			matrix[i] = new float[cols];
		}
	}

	// Destructor to free allocated memory
	~Grid() {
		// Deallocate memory for each column
		for (int i = 0; i < rows; ++i) {
			delete[] matrix[i];
		}

		// Deallocate memory for rows
		delete[] matrix;
	}

	// Accessor and mutator functions for the matrix elements
	int get(int row, int col) const {
		return matrix[row][col];
	}

	void set(int row, int col, float value) {
		matrix[row][col] = value;
	}
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
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
};

#endif