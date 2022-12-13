#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

void HSVtoRGB(float H, float S, float V, float& R, float& G, float& B) {
	//H(Hue): 0-360 degrees
	//S(Saturation) : 0 - 100 percent
	//V(Value) : 0 - 100 percent

	R = 0;
	G = 0;
	B = 0;

	if (H > 360 || H < 0 || S>100 || S < 0 || V>100 || V < 0) {
		cout << "The givem HSV values are not in valid range" << endl;
		return;
	}

	float s = S * 0.01;
	float v = V * 0.01;
	float C = s * v;
	float X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	float m = v - C;
	float r, g, b;
	if (H >= 0 && H < 60) {
		r = C, g = X, b = 0;
	}
	else if (H >= 60 && H < 120) {
		r = X, g = C, b = 0;
	}
	else if (H >= 120 && H < 180) {
		r = 0, g = C, b = X;
	}
	else if (H >= 180 && H < 240) {
		r = 0, g = X, b = C;
	}
	else if (H >= 240 && H < 300) {
		r = X, g = 0, b = C;
	}
	else {
		r = C, g = 0, b = X;
	}

	R = (r + m);
	G = (g + m);
	B = (b + m);
}

Grid::Grid() {
	_sizeX = 0;
	_sizeY = 0;
	_data = nullptr;
}

int Grid::getSizeX()
{
	return _sizeX;
}

int Grid::getSizeY()
{
	return _sizeY;
}

void Grid::setSize(int sizeX, int sizeY)
{
	if (_sizeX != sizeX || _sizeY != sizeY) {
		if (_data != nullptr) {
			for (int i = 0; i < _sizeX; i++) {
				delete[] _data[i];
				_data[i] = nullptr;
			}
			delete[] _data;
			_data = nullptr;
		}

		if (sizeX > 0 && sizeY > 0) {
			_data = new float* [sizeX];
			for (int i = 0; i < sizeX; i++) {
				_data[i] = new float[sizeY];
			}
		}
	}

	_sizeX = sizeX;
	_sizeY = sizeY;

	setZero();
}

void Grid::setZero()
{
	for (int i = 0; i < _sizeX; i++) {
		for (int j = 0; j < _sizeY; j++) {
			_data[i][j] = 0;
		}
	}
}

float Grid::getValueIndex(int gridX, int gridY)
{
	return _data[gridX][gridY];
}

void Grid::setValueIndex(int gridX, int gridY, float v)
{
	_data[gridX][gridY] = v;
}

Grid::~Grid()
{
	if (_data != nullptr) {
		for (int i = 0; i < _sizeX; i++) {
			delete[] _data[i];
			_data[i] = nullptr;
		}
		delete[] _data;
		_data = nullptr;
	}
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	frontT = new Grid();
	backT = new Grid();
	_h = 0.025;
	_sizeX = 24;
	_sizeY = 24;
	_diffusionCoef = 1.0;
}

DiffusionSimulator::~DiffusionSimulator()
{
	if (frontT != nullptr) {
		delete frontT;
	}
	if (backT != nullptr) {
		delete backT;
	}
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "DiffCoef", TW_TYPE_FLOAT, &_diffusionCoef, "");
	TwAddVarRW(DUC->g_pTweakBar, "sizeX", TW_TYPE_INT32, &_sizeX, "");
	TwAddVarRW(DUC->g_pTweakBar, "sizeY", TW_TYPE_INT32, &_sizeY, "");
	TwAddVarRW(DUC->g_pTweakBar, "Length", TW_TYPE_FLOAT, &this->_h, "");
}

void DiffusionSimulator::setupDemo1() {
	frontT->setValueIndex(frontT->getSizeX() / 2, frontT->getSizeY() / 2, 100);
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	reset();

	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	frontT->setSize(_sizeX, _sizeY);
	frontT->setZero();
	backT->setSize(_sizeX, _sizeY);

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		setupDemo1();
		break;
	case 1:
		cout << "Implicit solver!\n";
		setupDemo1();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::swapBuffer()
{
	Grid* temp = frontT;
	frontT = backT;
	backT = temp;
}

void DiffusionSimulator::diffuseTemperatureExplicit(const float dt) {//add your own parameters
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	const int SIZE_X = frontT->getSizeX();
	const int SIZE_Y = frontT->getSizeY();

	const float INV_HH = 1.0;// 1.0 / (_h * _h)

	for (int i = 1; i < SIZE_X - 1; i++) {
		for (int j = 1; j < SIZE_Y - 1; j++) {
			float laplacian =
				(frontT->getValueIndex(i + 1, j) + frontT->getValueIndex(i - 1, j)
					+ frontT->getValueIndex(i, j + 1) + frontT->getValueIndex(i, j - 1)
					- 4 * frontT->getValueIndex(i, j)) * INV_HH;
			float newValue = frontT->getValueIndex(i, j) +
				_diffusionCoef * laplacian * dt;

			backT->setValueIndex(i, j, newValue);
		}
	}
	// boundary
	for (int i = 0; i < SIZE_X - 1; i++) {
		backT->setValueIndex(i, 0, 0);
		backT->setValueIndex(i, SIZE_Y - 1, 0);
	}
	for (int j = 1; j < SIZE_Y - 1; j++) {
		backT->setValueIndex(0, j, 0);
		backT->setValueIndex(SIZE_X - 1, j, 0);
	}
	backT->setValueIndex(0, 0, 0);
	backT->setValueIndex(SIZE_X - 1, 0, 0);
	backT->setValueIndex(0, SIZE_Y - 1, 0);
	backT->setValueIndex(SIZE_X - 1, SIZE_Y - 1, 0);

	swapBuffer();
}

void setupB(std::vector<Real>& b, Grid* grid) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	for (int i = 0; i < SIZE_X; i++) {
		for (int j = 0; j < SIZE_Y; j++) {
			int idx = i * SIZE_Y + j;
			b.at(idx) = grid->getValueIndex(i, j);
		}
	}
}

void fillT(const std::vector<Real>& x, Grid* grid) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	for (int i = 1; i < SIZE_X - 1; i++) {
		for (int j = 1; j < SIZE_Y - 1; j++) {
			int idx = i * SIZE_Y + j;
			grid->setValueIndex(i, j, x[idx]);
		}
	}
	for (int i = 0; i < SIZE_X - 1; i++) {
		grid->setValueIndex(i, 0, 0);
		grid->setValueIndex(i, SIZE_Y - 1, 0);
	}
	for (int j = 1; j < SIZE_Y - 1; j++) {
		grid->setValueIndex(0, j, 0);
		grid->setValueIndex(SIZE_X - 1, j, 0);
	}
	grid->setValueIndex(0, 0, 0);
	grid->setValueIndex(SIZE_X - 1, 0, 0);
	grid->setValueIndex(0, SIZE_Y - 1, 0);
	grid->setValueIndex(SIZE_X - 1, SIZE_Y - 1, 0);
}

void setupA(SparseMatrix<Real>& A, Grid* grid, double factor) {//add your own parameters
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	const int LAPLACE_OFFSETS[3] = {
		1,
		SIZE_Y,
	};
	for (int i = 1; i < SIZE_X-1; i++) {
		for (int j = 1; j < SIZE_Y-1; j++) {
			int idx = i * SIZE_Y + j;
			A.set_element(idx, idx, 1 + 4 * factor); // set diagonal
			for (int dim = 0; dim < 2; dim++) {
				A.set_element(idx, idx + LAPLACE_OFFSETS[dim], -factor);
				A.set_element(idx, idx - LAPLACE_OFFSETS[dim], -factor);
			}
		}
	}
	for (int i = 0; i < SIZE_X - 1; i++) {
		int idx0 = i * SIZE_Y;
		int idx1 = i * SIZE_Y + SIZE_Y - 1;
		A.set_element(idx0, idx0, 1);
		A.set_element(idx1, idx1, 1);
	}
	for (int j = 1; j < SIZE_Y - 1; j++) {
		int idx0 = j;
		int idx1 = (SIZE_X - 1) * SIZE_Y + j;
		A.set_element(idx0, idx0, 1);
		A.set_element(idx1, idx1, 1);
	}
	{

		int idx0 = 0;
		int idx1 = (SIZE_X - 1) * SIZE_Y;
		int idx2 = (SIZE_Y - 1);
		int idx3 = (SIZE_X - 1) * SIZE_Y + (SIZE_Y - 1);

		A.set_element(idx0, idx0, 1);
		A.set_element(idx1, idx1, 1);
		A.set_element(idx2, idx2, 1);
		A.set_element(idx3, idx3, 1);
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(const float dt) {//add your own parameters
	// solve A T = b
	// to be implemented
	//N = sizeX*sizeY*sizeZ
	const int N = frontT->getSizeX() * frontT->getSizeY();
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N, Real(0));

	//setupA(*A, frontT, _diffusionCoef * dt / (_h * _h));
	setupA(*A, frontT, _diffusionCoef * dt);
	setupB(*b, frontT);

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
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x, frontT);//copy x to T
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
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
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	const int SIZE_X = frontT->getSizeX();
	const int SIZE_Y = frontT->getSizeY();

	float r, g, b;
	Vec3 sphereSize(_h * 0.5, _h * 0.5, _h * 0.5);
	Vec3 offset(_h * _sizeX * -0.5, -0.25, _h * _sizeY * -0.5);
	float c = 180.0 / 1.5;
	float s = 1.0 / 1.5f;
	for (int i = 0; i < SIZE_X; i++) {
		for (int j = 0; j < SIZE_Y; j++) {
			float v = frontT->getValueIndex(i, j);
			float l = log10f(abs(v) + 1) * 2.0;
			float sign = v > 0 ? 1 : -1;
			HSVtoRGB(max(min(l * c + 179, 360.0f), 0.0f), 100, 100, r, g, b);
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, Vec3(r, g, b));
			DUC->drawSphere(Vec3(i * _h, 0, j * _h) + offset, 
				sphereSize * (l + 5e-1) * s);
		}
	}
	DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 0, Vec3(1, 1, 1));
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
