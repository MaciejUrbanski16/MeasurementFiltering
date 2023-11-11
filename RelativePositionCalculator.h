#pragma once
#include <cstdint>
#include <utility>
#include <optional>
#include <optional>
#include <Eigen/Dense>

#define M_PI 3.14159265358979323846

struct QuadraticEquationCoefficients
{
    double a;
    double b;
    double c;

    double x1;
    double x2;
    double y1;
    double y2;
};

class RelativePositionCalculator {
public:
    void calculateActualRelativePosition(const float velocity, const int timeIntervalMs, const float azimutDegree);
    void setPreviousRelativePosition(std::pair<double, double> previousRelativePosition);
    [[nodiscard]] std::pair<double, double> getCalculatedRelativePosition() const;
private:
    std::pair<double, double> calculatedRelativePosition{ 0.0, 0.0 };
    std::pair<double, double> previousRelativePosition{ 0.0, 0.0 };

    std::optional<std::pair<double, double>>
        calculateRelativePositionToActualPosition(const QuadraticEquationCoefficients& coefficients, const float d);

    void setNewCalculatedRelativePosition(const std::optional<std::pair<double, double>> relativePosition);

    std::optional<QuadraticEquationCoefficients> calculateEquationCoefficients(const double radius, const float degree);

    std::optional<std::pair<double, double>> calculateAcoefficient(const float degree);
    double invalidAzimutDegree{ 65535.0 };

    //Eigen::MatrixXd A, C, Q, R, P, K, P0;
};


