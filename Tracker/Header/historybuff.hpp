#ifndef HISTORYBUFF_HPP_INCLUDED
#define HISTORYBUFF_HPP_INCLUDED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../../Header/datatype.hpp"

#define HISTORYBUFF_MAXSIZE	            3
#define HISTORYBUFF_BOUND_STDDEVX           2.0f
#define HISTORYBUFF_BOUND_STDDEVY           2.0f

struct History
{
	RECT box;
	int param;

	History() {}
	History(RECT b, int p) : box(b), param(p) {}
};

class Recog_Hist_Manager
{
private:
	bool flag;
	bool full;
	int ptr;
	int capacity;
	History* buff;

public:
	Recog_Hist_Manager() : flag(false), buff(NULL) {} 

	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] Recog_Hist_Manager(allocate) : already allocated.\n");
			exit(-1);
		}
		capacity = cap;
		buff = (History*)malloc(sizeof(History) * capacity);
		
		flag = true;
	}

	void clear()
	{
		if(buff)
		{
			free(buff);
			buff = NULL;
		}

		ptr = 0;
		capacity = 0;
		full = false;
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] Recog_Hist_Manager(init) : not allcated yet.\n");
			exit(-1);
		}

		memset(buff, 0x00, sizeof(History) * capacity);
		ptr = 0;
		full = false;
	}

	int get_capacity()
	{
		return capacity;
	}

	int get_ptr()
	{
		return ptr;
	}

	History read_history(int* cpyptr)
	{
		if(--(*cpyptr) < 0) *cpyptr = capacity-1;
		return buff[*cpyptr];
	}

	void write_recog(RECT b, int p)
	{
		buff[ptr].box = b;
		buff[ptr].param = p;

		if(ptr == capacity-1) full = true;
		if(++ptr >= capacity) ptr = 0;
	}

	bool observe_history_buff()
	{
        	int i, cpyptr;
	        float stddev_x, stddev_y;
       		float avg_x, avg_y;
        	float avg_sqx, avg_sqy;
        	float avgsq_x, avgsq_y;
        	RECT box;
        	POINT center;

		if(full == false) return false;

	        avg_x = 0.0f;
		avg_y = 0.0f;
            	avg_sqx = 0.0f;
            	avg_sqy = 0.0f;
            	cpyptr = ptr;

            	for(i = 0; i < capacity; i++)
            	{
			box = read_history(&cpyptr).box;
			center.x = ((box.orig.x << 1) + box.size.w) >> 1;
			center.y = ((box.orig.y << 1) + box.size.h) >> 1;

                	avg_x += (float)center.x;
                	avg_y += (float)center.y;
                	avg_sqx += (float)(center.x * center.x);
                	avg_sqy += (float)(center.y * center.y);
           	} 

		avg_x /= (float)capacity;
		avg_y /= (float)capacity;
		avg_sqx /= (float) capacity;
		avg_sqy /= (float) capacity;
		
		avgsq_x = avg_x * avg_x;
		avgsq_y = avg_y * avg_y;
		
		stddev_x = sqrt(avg_sqx - avgsq_x);
		stddev_y = sqrt(avg_sqy - avgsq_y);

      		if( 
				stddev_x < HISTORYBUFF_BOUND_STDDEVX
				&& stddev_y < HISTORYBUFF_BOUND_STDDEVX
										)
		{
			return true;
		}
	
		return false;
    }
};


#endif // HISTORYBUFF_HPP_INCLUDED
