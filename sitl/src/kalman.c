#include <math.h>
#include "kalman.h"

void kalman_init(kalman_state_t *k, float h0, float v0, float P0_h, float P0_v)
{
    k->h = h0;
    k->v = v0;
    k->P[0][0] = P0_h; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f; k->P[1][1] = P0_v;
}

void kalman_predict(kalman_state_t *k, float accel, float dt, float sigma_a)
{
    /* x_pred = F*x + B*accel, done component-wise (F, B are fixed 2x2/2x1) */
    float h_pred = k->h + k->v * dt + 0.5f * accel * dt * dt;
    float v_pred = k->v + accel * dt;

    /* P_pred = F*P*F^T + Q, expanded by hand for this specific F.
       F = [[1, dt],[0,1]] */
    float P00 = k->P[0][0], P01 = k->P[0][1];
    float P10 = k->P[1][0], P11 = k->P[1][1];

    float FP00 = P00 + dt * P10;
    float FP01 = P01 + dt * P11;
    float FP10 = P10;
    float FP11 = P11;

    float P00_new = FP00 + dt * FP01; /* (F P) F^T */
    float P01_new = FP01;
    float P10_new = FP10 + dt * FP11;
    float P11_new = FP11;

    /* Q: discretized white-noise-acceleration model, sigma_a^2 scaled */
    float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
    float q = sigma_a * sigma_a;
    float Q00 = q * dt4 / 4.0f;
    float Q01 = q * dt3 / 2.0f;
    float Q11 = q * dt2;

    k->h = h_pred;
    k->v = v_pred;
    k->P[0][0] = P00_new + Q00;
    k->P[0][1] = P01_new + Q01;
    k->P[1][0] = P10_new + Q01; /* symmetric */
    k->P[1][1] = P11_new + Q11;
}

void kalman_update_baro(kalman_state_t *k, float z_baro, float sigma_baro)
{
    /* Special case of kalman_update_generic: H = [1, 0], z_pred = k->h */
    kalman_update_generic(k, z_baro, k->h, 1.0f, 0.0f, sigma_baro * sigma_baro);
}

void kalman_update_generic(kalman_state_t *k, float z, float z_pred,
                            float H0, float H1, float R)
{
    float P00 = k->P[0][0], P01 = k->P[0][1];
    float P10 = k->P[1][0], P11 = k->P[1][1];

    /* S = H P H^T + R, fully expanded for a general 1x2 row H = [H0, H1] */
    float S = H0 * H0 * P00 + 2.0f * H0 * H1 * P01 + H1 * H1 * P11 + R;

    /* K = P H^T / S */
    float K0 = (H0 * P00 + H1 * P01) / S;
    float K1 = (H0 * P10 + H1 * P11) / S;

    float y = z - z_pred; /* innovation, in the sensor's own units */

    k->h += K0 * y;
    k->v += K1 * y;

    /* P = (I - K H) P, expanded */
    float new_P00 = (1.0f - K0 * H0) * P00 - K0 * H1 * P10;
    float new_P01 = (1.0f - K0 * H0) * P01 - K0 * H1 * P11;
    float new_P10 = -K1 * H0 * P00 + (1.0f - K1 * H1) * P10;
    float new_P11 = -K1 * H0 * P01 + (1.0f - K1 * H1) * P11;

    k->P[0][0] = new_P00;
    k->P[0][1] = new_P01;
    k->P[1][0] = new_P10;
    k->P[1][1] = new_P11;
}

void kalman_update_baro_pressure(kalman_state_t *k, float p_measured_pa,
                                  float sigma_p_pa, float p0_pa, float scale_height_m)
{
    /* Measurement function: p(h) = p0 * exp(-h / H_s), evaluated at the
       CURRENT predicted altitude - this is the "extended" part of EKF,
       re-linearizing every tick rather than using a fixed H. */
    float p_pred = p0_pa * expf(-k->h / scale_height_m);

    /* Jacobian dp/dh = -p_pred / H_s (falls out of differentiating the
       exponential - reuses p_pred, no separate computation needed).
       dp/dv = 0, barometer doesn't see velocity. */
    float H0 = -p_pred / scale_height_m;
    float H1 = 0.0f;

    kalman_update_generic(k, p_measured_pa, p_pred, H0, H1, sigma_p_pa * sigma_p_pa);
}