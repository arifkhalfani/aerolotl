/*
 * kalman.h / kalman.c - minimal 2-state linear Kalman filter.
 * State: [altitude (m), vertical velocity (m/s)]
 * Control input: accelerometer reading (m/s^2)
 * Measurement: barometric altitude (m)
 *
 * Deliberately kept a plain linear KF, not an EKF - H and F are both
 * already linear for this state/sensor pair, so there's no Jacobian to
 * take yet. This is meant to be the first, checkable building block
 * before generalizing to the full state vector (orientation, biases)
 * where nonlinearity actually shows up.
 *
 * No FreeRTOS types, no HAL types - lives entirely in "Layer 3",
 * callable identically from the POSIX sim and from real firmware.
 */

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float h;   /* altitude estimate, m */
    float v;   /* velocity estimate, m/s */
    float P[2][2]; /* covariance matrix, row-major */
} kalman_state_t;

/* sigma_a: accelerometer noise std-dev (m/s^2), used to build Q internally */
void kalman_init(kalman_state_t *k, float h0, float v0, float P0_h, float P0_v);

/* Predict step: advance state using accelerometer reading `accel` over `dt` seconds. */
void kalman_predict(kalman_state_t *k, float accel, float dt, float sigma_a);

/* Update step: correct state using a barometer altitude reading `z_baro`. */
void kalman_update_baro(kalman_state_t *k, float z_baro, float sigma_baro);

/* Generic single-row EKF update for a scalar measurement whose Jacobian
   w.r.t. [h, v] is [H0, H1], and whose predicted value is `z_pred`
   (i.e. h(x^-), the nonlinear measurement function evaluated at the
   current estimate - NOT necessarily the state itself).
   This is the actual EKF update; kalman_update_baro above is the
   special case H0=1, H1=0, z_pred=k->h. */
void kalman_update_generic(kalman_state_t *k, float z, float z_pred,
                            float H0, float H1, float R);

/* Barometric pressure update: measurement is raw pressure (Pa), not
   altitude. Computes p(h) = p0 * exp(-h/scale_height) and its Jacobian
   internally, then calls kalman_update_generic. This is the physically
   correct way to fuse a pressure sensor - see derivation in chat. */
void kalman_update_baro_pressure(kalman_state_t *k, float p_measured_pa,
                                  float sigma_p_pa, float p0_pa, float scale_height_m);

#endif /* KALMAN_H */