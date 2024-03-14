#pragma once
#include "KalmanFilterSetupGui.h"
#include "AccelTransform.h"
#include "AppLogger.h"
#include "HaversineConverter.h"
#include "RawMeasurements.h"
#include "kalman_filter/kalman_filter.h"
#include <optional>

class KalmanFilters
{
public:
    KalmanFilters(KalmanFilterSetupGui& kalmanFilterSetupGui) : kalmanFilterSetupGui(kalmanFilterSetupGui){}

    bool makePositionFiltration(const MeasurementsController& rawMeasurement, const std::optional<GpsDistanceAngular> gpsBasedPosition, const TransformedAccel& transformedAccel,
        const double filteredAzimuth, const double Xacc, const double Yacc, uint32_t deltaTimeUint);
    void makeAzimuthFitration(const std::optional<GpsDistanceAngular> gpsBasedPosition, 
                              const double zAngleVelocityDegPerSec, const double azimuthFromMagn, const uint32_t deltaTime);
    void makeGyroFiltration(const double xAngleVelocityDegPerSec, const double yAngleVelocityDegPerSec,
                            const double zAngleVelocityDegPerSec, uint32_t deltaTimeMs);

    kf::KalmanFilter<DIM_X, DIM_Z>&                     getFilterForPosition() { return kalmanFilterForPosition; }
    kf::KalmanFilter<DIM_X_azimuth, DIM_Z_azimuth>&     getFilterForAzimuth() { return kalmanFilterForAzimuth; }
    kf::KalmanFilter<DIM_X_gyro, DIM_Z_gyro>&           getFilterForGyro() { return kalmanFilterForGyro; }

private:
    kf::KalmanFilter<DIM_X, DIM_Z>                  kalmanFilterForPosition;
    kf::KalmanFilter<DIM_X_azimuth, DIM_Z_azimuth>  kalmanFilterForAzimuth;
    kf::KalmanFilter<DIM_X_gyro, DIM_Z_gyro>        kalmanFilterForGyro;

    kf::Matrix<DIM_Z, DIM_X> matHforPosition;
    //kf::Matrix<DIM_Z, DIM_X> matHforAzimuth;
    //kf::Matrix<DIM_Z, DIM_X> matHforGyro;

    KalmanFilterSetupGui& kalmanFilterSetupGui;
};

