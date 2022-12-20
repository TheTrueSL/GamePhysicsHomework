#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "util/vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid();
	int getSizeX();
	int getSizeY();
	int getSizeZ();
	void setSize(int sizeX, int sizeY, int sizeZ);
	void setZero();
	float getValueIndex(int gridX, int gridY, int gridZ);
	void setValueIndex(int gridX, int gridY, int gridZ, float v);
	~Grid();
private:
	// Attributes
	int _sizeX;
	int _sizeY;
	int _sizeZ;
	float*** _data;
};



class DiffusionSimulator :public Simulator {
public:
	// Construtors
	DiffusionSimulator();
	~DiffusionSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	void swapBuffer();
	void diffuseTemperatureExplicit(const float dt);
	void diffuseTemperatureImplicit(const float dt);

	// demo scenes
	void setupDemo1();
	void setupDemo2();
	void setupDemo3();
	void setupDemo4();
private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

private:
	// UI properties
	int _sizeX;
	int _sizeY;
	int _sizeZ;
	float _h;
	float _diffusionCoef;
	int _integrator;

private:// Simulation
	Grid* frontT; //save results of every time step
	Grid* backT; //save results of every time step
};

#endif
