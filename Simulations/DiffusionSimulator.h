#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
private:
	std::vector<float> matrix;
public:
	int rows; // y
	int cols; // x

	Grid();
	Grid(int numRows, int numCols);
	Grid(int numRows, int numCols, float *matrix);

	~Grid();

	float get(int row, int col) const;
	void set(int row, int col, float value);

	// Min and max values
	std::pair<float, float> getValueInterval();

	Grid convolution(Grid window);

	std::string to_string();
};

class GridPixel {
private:
	Grid *grid;
	int x, y;
	std::pair<float, float> normInterval;
	Mat4 object2WorldMatrix;
	Vec3 color;

public:
	GridPixel(Grid* grid, int x, int y, std::pair<float, float> normInterval);
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
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);

	void updateDimensions(int m, int n);

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;

	std::vector<GridPixel*> pixels;
	void updatePixels();
};

#endif