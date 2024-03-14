#include "MagnetometerCallibrator.h"

void MagnetometerCallibrator::collectData(const int16_t xMagn, const int16_t yMagn)
{
	sumOfXMagn += xMagn;
	sumOfYMagn += yMagn;


	magnetometerData.conservativeResize(2, magnetometerData.cols() + 1);
	magnetometerData.col(magnetometerData.cols() - 1) << xMagn, yMagn;

	numOfSamples++;
}

void MagnetometerCallibrator::callibrate()
{
	xOffset = sumOfXMagn / numOfSamples;
	yOffset = sumOfYMagn / numOfSamples;

	//Eigen::MatrixXd covarianceMatrix = magnetometerData * magnetometerData.transpose();

	//Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
	//Eigen::VectorXd eigenvalues = solver.eigenvalues();
	//Eigen::MatrixXd eigenvectors = solver.eigenvectors();

	//Eigen::MatrixXd selectedEigenvectors = eigenvectors.rightCols<2>();

	//// Budowanie macierzy skalowania
	//Eigen::VectorXd scalingFactors = eigenvalues.cwiseSqrt().cwiseInverse();
	//Eigen::MatrixXd scalingMatrix = scalingFactors.asDiagonal();


}

void MagnetometerCallibrator::setBiasToNorth(const int16_t bias)
{
	biasToNorth = bias;
}

void MagnetometerCallibrator::setCurrentAzimuth(const double azimuth)
{
	currentAzimuth = azimuth;
}
