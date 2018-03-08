#ifndef KALMAN_H_INCLUDED
#define KALMAN_H_INCLUDED

#include "../../Header/datatype.hpp"

class KalmanPoint
{
private:
    //constant
    float sec_per_frame;
    float sys_noise_mat_q;
    float sensor_noise_mat_r;
    float init_mat_p[4];

    //variable
    float vec_x[4];
    float mat_p[4][4];
    POINT estimation;

public:
    KalmanPoint() {}

    KalmanPoint(
   		    float spf,
		    float sysNoise,
		    float sensorNoise,
		    float imp0, float imp1, float imp2, float imp3)
                :sec_per_frame(spf),
                sys_noise_mat_q(sysNoise),
                sensor_noise_mat_r(sensorNoise)
    {
        init_mat_p[0] = imp0;
        init_mat_p[1] = imp1;
        init_mat_p[2] = imp2;
        init_mat_p[3] = imp3;
    }

    void init(POINT);

    void write_measurement(POINT);

    POINT read_estimation()
    {
        return estimation;
    }
};

class ExpMovingAvg
{
private:
    //constant
    float alpha;

    //variable
    float estimation;

public:
    ExpMovingAvg(){}

    ExpMovingAvg(float _alpha) : alpha(_alpha) {}

    void init(float value)
    {
        estimation = value;
    }

    void write_measurement(float value)
    {
        estimation = alpha * estimation + (1.0f - alpha) * value;
    }

    float read_estimation()
    {
        return estimation;
    }

};

class TimBox
{
private:
    KalmanPoint base;
    ExpMovingAvg width;
    ExpMovingAvg height;

public:
    TimBox() {}
    TimBox(
		    float spf, 
		    float sysNoise, 
		    float sensorNoise,
             	    float imp0, float imp1, float imp2, float imp3,
		    float alpha1, float alpha2)
    {
        base = KalmanPoint(spf, sysNoise, sensorNoise, imp0, imp1, imp2, imp3);
        width = ExpMovingAvg(alpha1);
        height = ExpMovingAvg(alpha2);
    }

    void init(RECT initializer)
    {
	base.init(initializer.orig);
	width.init((float)initializer.size.w);
	height.init((float)initializer.size.h);
    }

    void write_measurement(RECT box)
    {
	base.write_measurement(box.orig);
	width.write_measurement((float)box.size.w);
	height.write_measurement((float)box.size.h);
    }

    RECT read_estimation()
    {
        RECT box;
	box.orig = base.read_estimation();
	box.size.w = width.read_estimation();
	box.size.h = height.read_estimation();

        return box;
    }
};




#endif // KALMAN_H_INCLUDED
