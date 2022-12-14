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
	_sizeZ = 0;
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

int Grid::getSizeZ()
{
	return _sizeZ;
}

void Grid::setSize(int sizeX, int sizeY, int sizeZ)
{
	if (_sizeX != sizeX || _sizeY != sizeY || _sizeZ != sizeZ) {
		if (_data != nullptr) {
			for (int i = 0; i < _sizeX; i++) {
				for (int j = 0; j < _sizeY; j++) {
					delete[] _data[i][j];
				}
				delete[] _data[i];
			}
			delete[] _data;
			_data = nullptr;
		}

		if (sizeX > 0 && sizeY > 0 && sizeZ > 0) {
			_data = new float**[sizeX];
			for (int i = 0; i < sizeX; i++) {
				_data[i] = new float*[sizeY];
				for (int j = 0; j < sizeY; j++) {
					_data[i][j] = new float[sizeZ];
				}
			}
		}
	}

	_sizeX = sizeX;
	_sizeY = sizeY;
	_sizeZ = sizeZ;

	setZero();
}

void Grid::setZero()
{
	for (int i = 0; i < _sizeX; i++) {
		for (int j = 0; j < _sizeY; j++) {
			for (int k = 0; k < _sizeY; k++) {
				_data[i][j][k] = 0;
			}
		}
	}
}

float Grid::getValueIndex(int gridX, int gridY, int gridZ)
{
	return _data[gridX][gridY][gridZ];
}

void Grid::setValueIndex(int gridX, int gridY, int gridZ, float v)
{
	_data[gridX][gridY][gridZ] = v;
}

Grid::~Grid()
{
	if (_data != nullptr) {
		for (int i = 0; i < _sizeX; i++) {
			for (int j = 0; j < _sizeY; j++) {
				delete[] _data[i][j];
			}
			delete[] _data[i];
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
	_sizeX = 16;
	_sizeY = 16;
	_sizeZ = 16;
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
	TwAddVarRW(DUC->g_pTweakBar, "sizeZ", TW_TYPE_INT32, &_sizeZ, "");
	TwAddVarRW(DUC->g_pTweakBar, "Length", TW_TYPE_FLOAT, &this->_h, "");
}

void DiffusionSimulator::setupDemo1() {
	frontT->setValueIndex(frontT->getSizeX() / 2, 
		frontT->getSizeY() / 2, 
		frontT->getSizeZ() / 2, 100);
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
	_sizeX = max(1, min(50, _sizeX));
	_sizeY = max(1, min(50, _sizeY));
	_sizeZ = max(1, min(50, _sizeZ));

	frontT->setSize(_sizeX, _sizeY, _sizeZ);
	frontT->setZero();
	backT->setSize(_sizeX, _sizeY, _sizeZ);
	backT->setZero();

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
	const int SIZE_Z = frontT->getSizeZ();

	const float INV_HH = 1.0;// 1.0 / (_h * _h)

	for (int i = 1; i < SIZE_X - 1; i++) {
		for (int j = 1; j < SIZE_Y - 1; j++) {
			for (int k = 1; k < SIZE_Z - 1; k++) {
				float laplacian =
					(frontT->getValueIndex(i + 1, j, k) + frontT->getValueIndex(i - 1, j, k)
						+ frontT->getValueIndex(i, j + 1, k) + frontT->getValueIndex(i, j - 1, k)
						+ frontT->getValueIndex(i, j, k + 1) + frontT->getValueIndex(i, j, k - 1)
						- 6 * frontT->getValueIndex(i, j, k)) * INV_HH;
				float newValue = frontT->getValueIndex(i, j, k) +
					_diffusionCoef * laplacian * dt;

				backT->setValueIndex(i, j, k, newValue);
			}
		}
	}
	swapBuffer();
}

void setupB(std::vector<Real>& b, Grid* grid) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	const int SIZE_Z = grid->getSizeZ();
	for (int i = 0; i < SIZE_X; i++) {
		for (int j = 0; j < SIZE_Y; j++) {
			for (int k = 0; k < SIZE_Z; k++) {
				int idx = i * SIZE_Y * SIZE_Z + j * SIZE_Z + k;
				b.at(idx) = grid->getValueIndex(i, j, k);
			}
		}
	}
}

void fillT(const std::vector<Real>& x, Grid* grid) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	const int SIZE_Z = grid->getSizeZ();
	for (int i = 1; i < SIZE_X - 1; i++) {
		for (int j = 1; j < SIZE_Y - 1; j++) {
			for (int k = 1; k < SIZE_Z - 1; k++) {
				int idx = i * SIZE_Y * SIZE_Z + j * SIZE_Z + k;
				grid->setValueIndex(i, j, k, x[idx]);
			}
		}
	}
}

void setupA(SparseMatrix<Real>& A, Grid* grid, double factor) {//add your own parameters
	const int SIZE_X = grid->getSizeX();
	const int SIZE_Y = grid->getSizeY();
	const int SIZE_Z = grid->getSizeZ();
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	const int LAPLACE_OFFSETS[3] = {
		1,
		SIZE_Y,
		SIZE_Y * SIZE_Z,
	};
	const int X_MAX = SIZE_X - 1;
	const int Y_MAX = SIZE_Y - 1;
	const int Z_MAX = SIZE_Z - 1;
	for (int i = 0; i < SIZE_X; i++) {
		for (int j = 0; j < SIZE_Y; j++) {
			for (int k = 0; k < SIZE_Z; k++) {
				int idx = i * SIZE_Y * SIZE_Z + j * SIZE_Z + k;
				if (i == 0 || i == X_MAX || 
					j == 0 || j == Y_MAX || 
					k == 0 || k == Z_MAX) { // boundary
					A.set_element(idx, idx, 1);
				}
				else {
					A.set_element(idx, idx, 1 + 6 * factor); // set diagonal
					for (int dim = 0; dim < 3; dim++) {
						A.set_element(idx, idx + LAPLACE_OFFSETS[dim], -factor);
						A.set_element(idx, idx - LAPLACE_OFFSETS[dim], -factor);
					}
				}
			}
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(const float dt) {//add your own parameters
	// solve A T = b
	// to be implemented
	//N = sizeX*sizeY*sizeZ
	const int N = frontT->getSizeX() * frontT->getSizeY() * frontT->getSizeZ();
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
	const int SIZE_Z = frontT->getSizeZ();

	float r, g, b;
	Vec3 sphereSize(_h * 0.5, _h * 0.5, _h * 0.5);
	Vec3 offset(_h * _sizeX * -0.5, _h * _sizeX * -0.5 - 0.2, _h * _sizeY * -0.5);
	float c = 180.0 / 1.5;
	float s = 1.0 / 1.5f;
	for (int i = 0; i < SIZE_X; i++) {
		for (int j = 0; j < SIZE_Y; j++) {
			for (int k = 0; k < SIZE_Z; k++) {
				float v = frontT->getValueIndex(i, j, k);
				float l = log10f(abs(v) + 1) * 2.0;
				float sign = v > 0 ? 1 : -1;
				HSVtoRGB(max(min(l * c + 179, 360.0f), 0.0f), 100, 100, r, g, b);
				DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, Vec3(r, g, b));
				DUC->drawSphere(Vec3(i * _h, k * _h, j * _h) + offset,
					sphereSize * (l + 5e-2) * s);
			}
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
