#include "../Header/kalman.hpp"

void KalmanPoint::init(POINT initializer)
{
    //position, x
    vec_x[0] = (float)initializer.x;
    //velocity, x
    vec_x[1] = 0.0f;
    //position, y
    vec_x[2] = (float)initializer.y;
    //velocity, y
    vec_x[3] = 0.0f;

    mat_p[0][0] = this->init_mat_p[0];
    mat_p[0][1] = 0.0f;
    mat_p[0][2] = 0.0f;
    mat_p[0][3] = 0.0f;
    mat_p[1][0] = 0.0f;
    mat_p[1][1] = this->init_mat_p[1];
    mat_p[1][2] = 0.0f;
    mat_p[1][3] = 0.0f;
    mat_p[2][0] = 0.0f;
    mat_p[2][1] = 0.0f;
    mat_p[2][2] = this->init_mat_p[2];
    mat_p[2][3] = 0.0f;
    mat_p[3][0] = 0.0f;
    mat_p[3][1] = 0.0f;
    mat_p[3][2] = 0.0f;
    mat_p[3][3] = this->init_mat_p[3];

    estimation.x = initializer.x;
    estimation.y = initializer.y;
}

void KalmanPoint::write_measurement(POINT measurement)
{
    float vec_x_predict[4];
    float mat_p_predict[4][4];
    float mat_k[4][2];
    float vec_z[2];

    float fxdt, hxdt, nxdt, pxdt;
    float inv_det;
    float _a, _k, ci;

    //step1: prediction
    vec_x_predict[0] = vec_x[0] + sec_per_frame * vec_x[1];
    vec_x_predict[1] = vec_x[1];
    vec_x_predict[2] = vec_x[2] + sec_per_frame * vec_x[3];
    vec_x_predict[3] = vec_x[3];
    fxdt = mat_p[1][1]*sec_per_frame;
    hxdt = mat_p[1][3]*sec_per_frame;
    nxdt = mat_p[3][1]*sec_per_frame;
    pxdt = mat_p[3][3]*sec_per_frame;
    mat_p_predict[0][0] = mat_p[0][0] + (mat_p[1][0]+mat_p[0][1])*sec_per_frame + fxdt*sec_per_frame;
    mat_p_predict[0][1] = mat_p[0][1] + fxdt;
    mat_p_predict[0][2] = mat_p[0][2] + (mat_p[0][3]+mat_p[1][2])*sec_per_frame + hxdt*sec_per_frame;
    mat_p_predict[0][3] = mat_p[0][3] + hxdt;
    mat_p_predict[1][0] = mat_p[1][0] + fxdt;
    mat_p_predict[1][1] = mat_p[1][1];
    mat_p_predict[1][2] = mat_p[1][2] + hxdt;
    mat_p_predict[1][3] = mat_p[1][3];
    mat_p_predict[2][0] = mat_p[2][0] + (mat_p[3][0]+mat_p[2][1])*sec_per_frame + nxdt*sec_per_frame;
    mat_p_predict[2][1] = mat_p[2][1] + nxdt;
    mat_p_predict[2][2] = mat_p[2][2] + (mat_p[3][2]+mat_p[2][3])*sec_per_frame + pxdt*sec_per_frame;
    mat_p_predict[2][3] = mat_p[2][3] + pxdt;
    mat_p_predict[3][0] = mat_p[3][0] + nxdt;
    mat_p_predict[3][1] = mat_p[3][1];
    mat_p_predict[3][2] = mat_p[3][2] + pxdt;
    mat_p_predict[3][3] = mat_p[3][3];
    mat_p_predict[0][0] += sys_noise_mat_q;
    mat_p_predict[1][1] += sys_noise_mat_q;
    mat_p_predict[2][2] += sys_noise_mat_q;
    mat_p_predict[3][3] += sys_noise_mat_q;

    //step2: get Kalman gain
    _a = mat_p_predict[0][0] + sensor_noise_mat_r;
    _k = mat_p_predict[2][2] + sensor_noise_mat_r;
    ci = mat_p_predict[0][2]*mat_p_predict[2][0];
    inv_det = 1/((_a*_k)-ci);
    mat_k[0][0] = inv_det * (mat_p_predict[0][0]*_k - ci);
    mat_k[0][1] = inv_det * (sensor_noise_mat_r * mat_p_predict[0][2]);
    mat_k[1][0] = inv_det * (mat_p_predict[1][0]*_k - mat_p_predict[1][2]*mat_p_predict[2][0]);
    mat_k[1][1] = inv_det * (mat_p_predict[1][2]*_a - mat_p_predict[1][0]*mat_p_predict[0][2]);
    mat_k[2][0] = inv_det * (sensor_noise_mat_r * mat_p_predict[2][0]);
    mat_k[2][1] = inv_det * (mat_p_predict[2][2]*_a - ci);
    mat_k[3][0] = inv_det * (mat_p_predict[3][0]*_k - mat_p_predict[3][2]*mat_p_predict[2][0]);
    mat_k[3][1] = inv_det * (mat_p_predict[3][2]*_a - mat_p_predict[3][0]*mat_p_predict[0][2]);

    //step3: get estimate value
    vec_z[0] = (float)measurement.x;
    vec_z[1] = (float)measurement.y;
    vec_z[0] -= vec_x_predict[0];
    vec_z[1] -= vec_x_predict[2];
    vec_x[0] = vec_x_predict[0] + mat_k[0][0]*vec_z[0] + mat_k[0][1]*vec_z[1];
    vec_x[1] = vec_x_predict[1] + mat_k[1][0]*vec_z[0] + mat_k[1][1]*vec_z[1];
    vec_x[2] = vec_x_predict[2] + mat_k[2][0]*vec_z[0] + mat_k[2][1]*vec_z[1];
    vec_x[3] = vec_x_predict[3] + mat_k[3][0]*vec_z[0] + mat_k[3][1]*vec_z[1];

    //update P_matrix
    mat_p[0][0] = mat_p_predict[0][0] - (mat_k[0][0]*mat_p_predict[0][0] + mat_k[0][1]*mat_p_predict[2][0]);
    mat_p[0][1] = mat_p_predict[0][1] - (mat_k[0][0]*mat_p_predict[0][1] + mat_k[0][1]*mat_p_predict[2][1]);
    mat_p[0][2] = mat_p_predict[0][2] - (mat_k[0][0]*mat_p_predict[0][2] + mat_k[0][1]*mat_p_predict[2][2]);
    mat_p[0][3] = mat_p_predict[0][3] - (mat_k[0][0]*mat_p_predict[0][3] + mat_k[0][1]*mat_p_predict[2][3]);
    mat_p[1][0] = mat_p_predict[1][0] - (mat_k[1][0]*mat_p_predict[0][0] + mat_k[1][1]*mat_p_predict[2][0]);
    mat_p[1][1] = mat_p_predict[1][1] - (mat_k[1][0]*mat_p_predict[0][1] + mat_k[1][1]*mat_p_predict[2][1]);
    mat_p[1][2] = mat_p_predict[1][2] - (mat_k[1][0]*mat_p_predict[0][2] + mat_k[1][1]*mat_p_predict[2][2]);
    mat_p[1][3] = mat_p_predict[1][3] - (mat_k[1][0]*mat_p_predict[0][3] + mat_k[1][1]*mat_p_predict[2][3]);
    mat_p[2][0] = mat_p_predict[2][0] - (mat_k[2][0]*mat_p_predict[0][0] + mat_k[2][1]*mat_p_predict[2][0]);
    mat_p[2][1] = mat_p_predict[2][1] - (mat_k[2][0]*mat_p_predict[0][1] + mat_k[2][1]*mat_p_predict[2][1]);
    mat_p[2][2] = mat_p_predict[2][2] - (mat_k[2][0]*mat_p_predict[0][2] + mat_k[2][1]*mat_p_predict[2][2]);
    mat_p[2][3] = mat_p_predict[2][3] - (mat_k[2][0]*mat_p_predict[0][3] + mat_k[2][1]*mat_p_predict[2][3]);
    mat_p[3][0] = mat_p_predict[3][0] - (mat_k[3][0]*mat_p_predict[0][0] + mat_k[3][1]*mat_p_predict[2][0]);
    mat_p[3][1] = mat_p_predict[3][1] - (mat_k[3][0]*mat_p_predict[0][1] + mat_k[3][1]*mat_p_predict[2][1]);
    mat_p[3][2] = mat_p_predict[3][2] - (mat_k[3][0]*mat_p_predict[0][2] + mat_k[3][1]*mat_p_predict[2][2]);
    mat_p[3][3] = mat_p_predict[3][3] - (mat_k[3][0]*mat_p_predict[0][3] + mat_k[3][1]*mat_p_predict[2][3]);

    estimation.x = (int)vec_x[0];
    estimation.y = (int)vec_x[2];
}
