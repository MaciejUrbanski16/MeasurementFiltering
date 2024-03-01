#include "KalmanFilters.h"

void KalmanFilters::makePositionFiltration(const std::optional<GpsDistanceAngular> gpsBasedPosition, const TransformedAccel& transformedAccel, const double Xacc, const double Yacc, uint32_t deltaTimeUint)
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

    kf::Matrix<DIM_Z, DIM_X> matH;
    kf::Vector<DIM_Z> vecZ;
    if (gpsBasedPosition)
    {
                //accX velX posX  accY  velY  posY
        matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z sensor�w to zero
                0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z GPS to zero
                0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F; //
        vecZ << transformedAccel.xAcc, transformedAccel.yAcc, gpsBasedPosition.value().xPosition, gpsBasedPosition.value().yPosition;
    }
    else
    {
                //accX velX posX  accY  velY  posY
        matH << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z sensor�w to zero
                0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
                0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z GPS to zero
                0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F; //
        vecZ << transformedAccel.xAcc, transformedAccel.yAcc, 0, 0;
    }

    kalmanFilterForPosition.setMatH(matH);
    kalmanFilterForPosition.correctLKF(vecZ, matRFromGui.value());
}

void KalmanFilters::makeAzimuthFitration(const std::optional<GpsDistanceAngular> gpsBasedPosition,
                                         const double zAngleVelocityDegPerSec, const double azimuthFromMagn, const uint32_t deltaTime)
{
    //X = [yaw,
    //     dYaw]       

    double deltaTimeSec = static_cast<double>(deltaTime) / 1000.0;

    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> A;
    A << 1.0F, deltaTimeSec,
         0.0F, 1.0F;


    //const auto matRAzzFromGui{ kalmanFilterSetupGui.getMatRazimuth() };
    //const auto matQAzzFromGui{ kalmanFilterSetupGui.getMatQazimuth() };
    kf::Matrix<DIM_Z_azimuth, DIM_Z_azimuth> matRAzimuthPedestrian;
    kf::Matrix<DIM_X_azimuth, DIM_X_azimuth> matQAzimuthPedestrian;
    matRAzimuthPedestrian << 
        10.0F, 0, 0,
        0, 1.0F, 0,
        0, 0, 0.1F;

    double process_variance = 0.00002F;
    double coefficient = 5.0F;
    matQAzimuthPedestrian << 
        coefficient, 0.0F,
        0.0F, coefficient;

    matQAzimuthPedestrian *= process_variance;

    kalmanFilterForAzimuth.predictLKF(A, matQAzimuthPedestrian);
    kf::Matrix<DIM_Z_azimuth, DIM_X_azimuth> matH;


    kf::Vector<DIM_Z_azimuth> vecZ;
    if (gpsBasedPosition)
    {
        //when gps angle available
        vecZ << zAngleVelocityDegPerSec, azimuthFromMagn, gpsBasedPosition.value().angle;
        matH <<
            0, 1,
            1, 0,
            1, 0;
    }
    else
    {
        vecZ << zAngleVelocityDegPerSec, azimuthFromMagn, 0.0F;
        matH <<
            0, 1,
            1, 0,
            0, 0;
    }

    kalmanFilterForAzimuth.setMatH(matH);
    kalmanFilterForAzimuth.correctLKF(vecZ, matRAzimuthPedestrian);
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

    kalmanFilterForGyro.setMatH(matH);
    kalmanFilterForGyro.correctLKF(vecZ, matR);
}
