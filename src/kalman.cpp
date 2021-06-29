/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>

#include "kalman/kalman.h"

// -------------------------------- //
//        PRIVATE FUNCTIONS         //
// -------------------------------- //

/**
 * @brief Set the initial parameters of the estimator.
 * 
 */
void KalmanFilter::setInitState(const double &t, const Matrix &P)
{
    this->P = P;
    this->t0 = t;
    this->t = t;
    this->initialized = true;
}

// -------------------------------- //
//           CONSTRUCTORS           //
// -------------------------------- //

/**
 * @brief Create a Kalman filter with the specified matrices.
 *   A - System dynamics matrix
 *   C - Output matrix
 *   Q - Process noise covariance
 *   R - Measurement noise covariance
 *   P - Estimate error covariance
 */
KalmanFilter::KalmanFilter(
    double dt,
    const Matrix &A,
    const Matrix &C,
    const Matrix &Q,
    const Matrix &R,
    const Matrix &P)
    : A(A), C(C), Q(Q), R(R), P0(P),
      m(C.rows()), n(A.rows()), A_t(C.rows(), C.rows()), b_t(P.rows(), 1),
      dt(dt), initialized(false),
      x_hat(A.rows(), 1), x_hat_new(A.rows(), 1)
{
    I = Matrix::eye(n);
}

/**
 * @brief Create a blank Estimator.
 * 
 */
KalmanFilter::KalmanFilter() {}

// -------------------------------- //
//         PUBLIC FUNCTIONS         //
// -------------------------------- //

/**
 * @brief Initialize the filter with initial states as zero.
 * 
 */
void KalmanFilter::init(double t0, const Matrix &x0)
{
    x_hat = x0;
    setInitState(t0, P0);
}

/**
 * @brief Initialize the filter with a guess for initial states.
 * 
 */
void KalmanFilter::init()
{
    setInitState(0, P0);
}

/**
 * @brief Update the estimated state based on measured values. The
 * time step is assumed to remain constant.
 * 
 */
void KalmanFilter::update(const Matrix &y)
{

    if (!initialized)
        throw std::runtime_error("Filter is not initialized!");

    x_hat_new = A * x_hat;
    P = A * P * A.transpose() + Q;
    A_t = C * P * C.transpose();
    b_t = P * C;

    K = m_solver->solve(A_t, b_t);
    x_hat_new += K * (y - C.transpose() * x_hat_new);
    P = (I - K * C.transpose()) * P;
    x_hat = x_hat_new;

    t += dt;
}

/**
 * @brief Update the estimated state based on measured values,
 * using the given time step and dynamics matrix.
 * 
 */
void KalmanFilter::update(const Matrix &y, double dt, const Matrix A)
{
    this->A = A;
    this->dt = dt;
    update(y);
}

/**
 * @brief Return the current state and time.
 * 
 */
Matrix KalmanFilter::state() { return x_hat; };
double KalmanFilter::time() { return t; };
