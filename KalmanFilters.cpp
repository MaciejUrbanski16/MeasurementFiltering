#include "KalmanFilters.h"

void KalmanFilters::makePositionFiltration(std::pair<double, double> gpsBasedPosition, const TransformedAccel& transformedAccel, const double Xacc, const double Yacc, uint32_t deltaTimeUint)
{

    kf::Matrix<DIM_X, DIM_X> A;

    const double deltaTimeMs = static_cast<double>(deltaTimeUint) / 1000.0;
    A << 1.0F, deltaTimeMs, (deltaTimeMs * deltaTimeMs) / 2, 0.0F, 0.0F, 0.0F,
        0.0F, 1.0F, deltaTimeMs, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 1.0F, deltaTimeMs, (deltaTimeMs * deltaTimeMs) / 2,
        0.0F, 0.0F, 0.0F, 0.0F, 1.0F, deltaTimeMs,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

    const auto matRFromGui{ kalmanFilterSetupGui.getMatRacc() };
    const auto matQFromGui{ kalmanFilterSetupGui.getMatQacc() };

    kalmanFilterForPosition.predictLKF(A, matQFromGui.value());

    kf::Vector<DIM_Z> vecZ;
    vecZ << transformedAccel.xAcc, transformedAccel.yAcc, gpsBasedPosition.first, gpsBasedPosition.second;

    kf::Matrix<DIM_Z, DIM_X> matH;
            //accX velX posX  accY  velY  posY
    matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z GPS to zero
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F; //

    kalmanFilterForPosition.correctLKF(vecZ, matRFromGui.value(), matH);
}

void KalmanFilters::makeAzimuthFitration(const double xAngleVelocityDegPerSec, const double yAngleVelocityDegPerSec, 
                                         const double zAngleVelocityDegPerSec, const double azimuthFromMagn, const uint32_t deltaTime)
{
    double deltaTimeMs = static_cast<double>(deltaTime) / 1000.0;

    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> A;
    A << 1.0F, 0.0F, 0.0F, deltaTimeMs, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F, 0.0F, deltaTimeMs, 0.0F,
        0.0F, 0.0F, 1.0F, 0.0F, 0.0F, deltaTimeMs,
        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

    const auto matRAzzFromGui{ kalmanFilterSetupGui.getMatRazimuth() };
    const auto matQAzzFromGui{ kalmanFilterSetupGui.getMatQazimuth() };

    kalmanFilterForAzimuth.predictLKF(A, matQAzzFromGui.value());

    kf::Vector<DIM_Z_azimuth> vecZ;
    vecZ << xAngleVelocityDegPerSec, zAngleVelocityDegPerSec, azimuthFromMagn;

    kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;
    matH << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0;

    kalmanFilterForAzimuth.correctLKF(vecZ, matRAzzFromGui.value(), matH);
}

void KalmanFilters::makeGyroFiltration(const double xAngleVelocityDegPerSec, const double yAngleVelocityDegPerSec,
                                       const double zAngleVelocityDegPerSec, uint32_t deltaTimeMs)
{
    double deltaTimeSec = static_cast<double>(deltaTimeMs) / 1000.0F;

    kf::Matrix<DIM_X_gyro, DIM_X_gyro> A;
    A << 1.0F, deltaTimeSec, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 1.0F, deltaTimeSec, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 1.0F, deltaTimeSec,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;

    kf::Matrix<DIM_X_gyro, DIM_X_gyro> Q;

    double process_variance = 0.02F;
    deltaTimeMs = deltaTimeMs / 100.0F;
    Q << pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6, 0, 0, 0,
        pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2, 0, 0, 0,
        pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2), 0, 0, 0,
        0, 0, 0, pow(deltaTimeMs, 6) / 36, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 6,
        0, 0, 0, pow(deltaTimeMs, 5) / 12, pow(deltaTimeMs, 4) / 4, pow(deltaTimeMs, 3) / 2,
        0, 0, 0, pow(deltaTimeMs, 4) / 6, pow(deltaTimeMs, 3) / 2, pow(deltaTimeMs, 2);

    Q *= process_variance;

    kalmanFilterForGyro.predictLKF(A, Q);

    kf::Vector<DIM_Z_gyro> vecZ;

    vecZ << xAngleVelocityDegPerSec, yAngleVelocityDegPerSec, zAngleVelocityDegPerSec;

    kf::Matrix<DIM_Z_gyro, DIM_Z_gyro> matR;
    matR << 0.1F, 0, 0,
            0, 0.1F, 0,
            0, 0, 0.1F;
    kf::Matrix<DIM_Z_gyro, DIM_X_gyro> matH;
            
    matH << 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0;

    kalmanFilterForGyro.correctLKF(vecZ, matR, matH);
}
