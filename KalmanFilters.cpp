#include "KalmanFilters.h"

bool KalmanFilters::makePositionFiltration(
    const MeasurementsController& rawMeasurement,
    const std::optional<GpsDistanceAngular> gpsBasedPosition,
    const TransformedAccel& transformedAccel,
    const double filteredAzimuth,
    const double Xacc, const double Yacc,
    uint32_t deltaTimeUint)
{
    bool doIncrement{ false };
    kf::Matrix<DIM_X, DIM_X> A;

    /*
    * xpos,xvel, ypos, yvel
    */


    double deltaTimeMs = static_cast<double>(deltaTimeUint) / 1000.0;
    A <<
        1.0F, 0.0F, deltaTimeMs, 0.0F, 0.5F * deltaTimeMs * deltaTimeMs, 0.0F,
        0.0F, 1.0F, 0.0F, deltaTimeMs, 0.0F, 0.5F * deltaTimeMs * deltaTimeMs,
        0.0F, 0.0F, 1.0F, 0.0F, deltaTimeMs, 0.0F,
        0.0F, 0.0F, 0.0F, 1.0F, 0.0F, deltaTimeMs,
        0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F;


    //const auto matRFromGui{ kalmanFilterSetupGui.getMatRacc() };
    //const auto matQFromGui{ kalmanFilterSetupGui.getMatQacc() };
    kf::Matrix<DIM_Z, DIM_Z> matRAccPedestrian;
    kf::Matrix<DIM_X, DIM_X> matQAccPedestrian;

    const auto matR{ kalmanFilterSetupGui.getMatRacc() };
    const auto matQ{ kalmanFilterSetupGui.getMatQacc() };
    //kalmanFilterForPosition.predictLKF(A, matQ.value());

    kalmanFilterForPosition.predictLKF(A, matQ.value());

    kf::Matrix<DIM_Z, DIM_X> matH;
    kf::Vector<DIM_Z> vecZ;

    if (gpsBasedPosition)
    {
        const double velocity = gpsBasedPosition.value().velocity;
        const double xVelocity{ velocity * cos(filteredAzimuth) };
        const double yVelocity{ velocity * sin(filteredAzimuth) };
        const double xPosition{ gpsBasedPosition.value().xPosition };
        const double yPosition{ gpsBasedPosition.value().yPosition };
        

        //xpos ypos xvel  yvel  xacc yacc 
        matH <<
            1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z sensorów to zero
            0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z GPS to zero
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;//

        vecZ << xPosition, yPosition, 0.0F, 0.0F, 0.0F, 0.0F;


    }
    else
    {
        doIncrement = true;
        const double velocity{ rawMeasurement.getYvelocityMperS() };
        const double xVelocity{ velocity * cos(filteredAzimuth) };
        const double yVelocity{ velocity * sin(filteredAzimuth) };

    //            //xpos ypos xvel  yvel  xacc yacc 
        matH << 
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, //gdy brak danych z sensorów to zero
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
            0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,//gdy brak danych z GPS to zero
            0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,//
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,//
            0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F;//
        vecZ << 0.0F, 0.0F, xVelocity, yVelocity, 0.0F, 0.0F;
    }

    kalmanFilterForPosition.setMatH(matH);
    kalmanFilterForPosition.correctLKF(vecZ, matR.value());
    return doIncrement;
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
        0, 0.01F, 0,
        0, 0, 0.01F;

    double process_variance = 0.0000002F;
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
