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
	void setSize(int sizeX, int sizeY);
	void setZero();
	float getValueIndex(int gridX, int gridY);
	void setValueIndex(int gridX, int gridY, float v);
	~Grid();
private:
	// Attributes
	int _sizeX;
	int _sizeY;
	float** _data;
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
	float _h;
	float _diffusionCoef;

private:// Simulation
	Grid* frontT; //save results of every time step
	Grid* backT; //save results of every time step
};

#endif
