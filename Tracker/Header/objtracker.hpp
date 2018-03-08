#ifndef OBJECTTRACKER_HPP_INCLUDED
#define OBJECTTRACKER_HPP_INCLUDED

#include "../../Header/datatype.hpp"
#include "kalman.hpp"
#include "historybuff.hpp"

#define OBJECTTRACKER_EXPMOVAVG_ALPHAW                    0.7f
#define OBJECTTRACKER_EXPMOVAVG_ALPHAH                    0.7f
#define OBJECTTRACKER_KALMAN_SEC_PER_FRAME                ((1.0f)/(30.0f))
#define OBJECTTRACKER_KALMAN_MATRIX_Q_VALUE               10.0f
#define OBJECTTRACKER_KALMAN_MATRIX_R_VALUE               100.0f
#define OBJECTTRACKER_KALMAN_MATRIX_P_VALUE1              1.0f
#define OBJECTTRACKER_KALMAN_MATRIX_P_VALUE2              1.0f
#define OBJECTTRACKER_KALMAN_MATRIX_P_VALUE3              1.0f
#define OBJECTTRACKER_KALMAN_MATRIX_P_VALUE4              1.0f

class ObjTracker
{
private:
	bool active;
	int param;
	TimBox tbox;

public:

	ObjTracker() : active(false), param(0)
	{
		tbox = TimBox(
				OBJECTTRACKER_KALMAN_SEC_PER_FRAME,
		      		OBJECTTRACKER_KALMAN_MATRIX_Q_VALUE,
                      		OBJECTTRACKER_KALMAN_MATRIX_R_VALUE,
				OBJECTTRACKER_KALMAN_MATRIX_P_VALUE1,
              		        OBJECTTRACKER_KALMAN_MATRIX_P_VALUE2,
		     		OBJECTTRACKER_KALMAN_MATRIX_P_VALUE3,
                     		OBJECTTRACKER_KALMAN_MATRIX_P_VALUE4,
		     		OBJECTTRACKER_EXPMOVAVG_ALPHAW,
                     		OBJECTTRACKER_EXPMOVAVG_ALPHAH
                      	);
    	}

    	void deactivate()
    	{
		fprintf(stdout ,":::::[Tracker] Deactivate Object Tracking :::::\n");
        	active = false;
    	}

    	void activate(Recog_Hist_Manager * phm)
    	{
		History retval;
		int _param;
	    	int i, hsize,ptr;
		
		fprintf(stdout ,":::::[Tracker] Activate Object Tracking :::::\n");
		hsize = phm->get_capacity();
		ptr = phm->get_ptr();
		this->tbox.init(phm->read_history(&ptr).box);

		for(_param = 0, i = 0; i < hsize; i++)
		{
			retval = phm->read_history(&ptr);
			this->tbox.write_measurement(retval.box);
			_param += retval.param;
		}
		
        	this->param = _param / hsize;
        	active = true;
    	}
	
	void write_box(RECT box)
	{
		tbox.write_measurement(box);
	}
	
	RECT read_box()
	{
		return tbox.read_estimation();
	}

	bool is_active()
	{
		return active;
	}

	int read_param()
	{
		return param;
	}
};

#endif // OBJECTTRACKER_HPP_INCLUDED
