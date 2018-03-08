#ifndef USERDTYPE_H
#define USERDTYPE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define DATATYPE_VPM_DATA_MAXSIZE		20000

typedef unsigned char uchar;

struct POINT
{
	int x;
	int y;

	POINT() :x(0), y(0) {}

	void write(int _x, int _y)
	{
		x = _x;
		y = _y;
	}
};

struct AREA
{
	int w;
	int h;

	AREA() : w(0), h(0) {}

	void write(int _w, int _h)
	{
		w = _w;
		h = _h;
	}
};

struct TUPLE
{
	int upper;
	int below;

	TUPLE() : upper(0), below(0) {}

	void write(int _upper, int _below)
	{
		upper = _upper;
		below = _below;
	}
};

struct RECT
{
	POINT	orig;
	AREA	size;
	
	RECT() {}

	RECT(int x, int y, int w, int h)
	{
		orig.write(x, y);
		orig.write(w, h);
	}

	void write(int x, int y, int w, int h)
	{
		orig.write(x, y);
		size.write(w, h);
	}

	int overlapw(RECT& rect)
	{
		int leftA = orig.x;
		int leftB = rect.orig.x;
		int rightA = leftA + size.w;
		int rightB = leftB + rect.size.w;

    		if(leftA != leftB && rightA != rightB)
		{
			if(leftA > leftB)
            			return (rightA > rightB) ? (rightB - leftA) : (rightA - leftA);

        		else
            			return (rightA > rightB) ? (rightB - leftB) : (rightA - leftB);
    		}

    		else if(leftA != leftB && rightA == rightB)
    		{
        		return (leftA < leftB) ? (rightB - leftB) : (rightA - leftA);
    		}

    		else if(leftA == leftB && rightA != rightB)
    		{
        		return (rightA < rightB) ? (rightA - leftA) : (rightB - leftB);
    		}

    		else
    		{
        		return (rightA - leftA);
    		}
	}

	int intervalh(RECT& rect)
	{
		int heightA = size.h;
		int heightB = rect.size.h;
		int topA = orig.y;
		int topB = rect.orig.y;
		int botA = topA + size.h;
		int botB = topB + rect.size.h;
   
		if(heightA && heightB)
    		{
		        if(topA > topB && botA > botB && topA > botB)
		            return topA - botB;

		        else if(topA < topB && botA < botB && topB > botA)
		            return topB - botA;

		        else
		            return 0;
		}

 		else if(heightA && !heightB)	
		{
        		if(topB < topA)
		            return topA - topB;

		        else if (topB > botA)
		            return topB - botA;

		        else
		            return 0;
		}
	
		else if(!heightA && heightB)
		{
        		if(topA < topB)
		            return topB - topA;
		        else if(topA > botB)
		            return topA - botB;
		        else
		            return 0;
		}

		else
		{
			return ((topA > topB) ? (topA - topB) : (topB - topA));
    		}
	}
};

struct IMAGE
{
	bool	flag;
	AREA	size;
	int	ch;
	uchar	*data;

	IMAGE() : flag(false), data(NULL) {}

	void allocate(int c, int w, int h)
	{
		if(flag)
		{
			fprintf(stderr, "[error] IMAGE(allocate) : already allocated.\n");
			exit(-1);
		}
		
		size.write(w, h);
		ch = c;
		data = (uchar*)malloc(ch * sizeof(uchar) * size.w * size.h);
		flag = true;
	}

	void clear()
	{
		if(data)
		{	
			free(data);
			data = NULL;
		}
		ch = 0;
		size.write(0,0);
		flag = false;
	}
};

struct VPIXEL
{
	int x;
	int y;
	int val;

	void write(int _x, int _y, int _v)
	{
		x = _x;
		y = _y;
		val = _v;
	}
};

struct VpixelManager 
{
	bool	flag;
	int	capacity;
	int	size;
	VPIXEL	*data;

	VpixelManager() : flag(false), data(NULL) {}
	
	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] VpixelManager(allocate) : already allocated.\n");
			exit(-1);
		}
		capacity = cap;
		data = (VPIXEL*)malloc(sizeof(VPIXEL) * capacity);
		size = capacity;
		flag = true;
	}

	void clear()
	{
		if(data)
		{
			free(data);
			data = NULL;
		}
		capacity = 0;
		size = 0;
		flag = false;
	}

	void init()
	{
		size = 0;
	}

	void insert(int x, int y, int val)
	{
		if(!flag)
		{
			fprintf(stderr, "[error] VpixelManager(insert) : not allocated yet.\n");
			exit(-1);
		}

		if(size >= capacity)
		{
			fprintf(stderr, "[error] VpixelManager(insert) : size out of range.\n");
			return;
		}

		data[size++].write(x, y, val);
	}
};

struct Histogram
{
	bool	flag;
	int	capacity;
	int	size;
	int	freq;
	int	*data;

	Histogram() : flag(false), data(NULL) {}
	
	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] Histogram(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = cap;
		data = (int*)malloc(sizeof(int) * capacity);
		size = capacity;
		flag = true;
	}

	void clear()
	{
		if(data)
		{
			free(data);
			data = NULL;
		}
		capacity = 0;
		size = 0;
		freq = 0;
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] Histogram(init) : not allocated yet.\n");
			exit(-1);
		}

		memset(data, 0x00, sizeof(int) * size);
		size = 0;
		freq = 0;
	}

	void process(AREA size, uchar* src, RECT pos)
	{
		int r, c, val;

		if(!flag)
		{
			fprintf(stderr, "[error] Histogram(process) : not allcated yet.\n");
			exit(-1);
		}
	
		this->init();

		this->size = pos.size.w;

		//calcuate histogram.
		for(r = pos.orig.y; r < pos.orig.y + pos.size.h; r++)
		{
			for(c = 0; c < pos.size.w; c++)
			{
				data[c] += src[(pos.orig.x + c) + r * size.w];
			}
		}

		//calculate frequency.
		for(c = 1; c < this->size; c++)
		{
			val = data[c] - data[c-1];
			freq += (val > 0) ? val : -val;
		}
	}
};


#endif
