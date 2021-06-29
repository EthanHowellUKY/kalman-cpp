
#ifndef KALMAN_H
#define KALMAN_H

/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory paper:
 *
 *     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include "LinearAlgebra/Matrix.h"
#include "LinearAlgebra/SolverFactory.h"

class KalmanFilter
{

public:
    // -------------------------------- //
    //           CONSTRUCTORS           //
    // -------------------------------- //
    KalmanFilter(
        double dt,
        const Matrix &A,
        const Matrix &C,
        const Matrix &Q,
        const Matrix &R,
        const Matrix &P);

    KalmanFilter();

    // -------------------------------- //
    //         PUBLIC FUNCTIONS         //
    // -------------------------------- //

    // ------------ GETTERS ----------- //
    Matrix state();
    double time();

    // ------------ SETTERS ----------- //
    void init();
    void init(double t0, const Matrix &x0);
    void update(const Matrix &y);
    void update(const Matrix &y, double dt, const Matrix A);

private:
    // -------------------------------- //
    //        PRIVATE FUNCTIONS         //
    // -------------------------------- //
    void setInitState(const double &t, const Matrix &P);

    // -------------------------------- //
    //        PRIVATE VARIABLES         //
    // -------------------------------- //

    // Matrices for computation
    Matrix A, C, Q, R, P, K, P0;
    Matrix A_t, b_t;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Matrix I;

    // Estimated states
    Matrix x_hat, x_hat_new;

    // Linear Systems solver
    Solver *m_solver = SolverFactory::create(SOLVER::LUDECOMP);
};

#endif // KALMAN_H
