#ifndef LPR_H_INCLUDED
#define LPR_H_INCLUDED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "timdebug.hpp"
#include "datatype.hpp"
#include "datastruct.hpp"
#include "blob.hpp"
#include "imgproc.hpp"
#include "../Tracker/Header/objtracker.hpp"
#include "../Tracker/Header/meanshift.hpp"

#ifdef ACTIVATE_OPENCV
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif

#define LPR_CM_DATA_MAXSIZE		25
#define LPR_CM_MAKE_SCALE		0.02f	

#define LPR_TM_DATA_MAXSIZE		20
#define LPR_TM_PROC_SCALE1		0.25f	
#define LPR_TM_PROC_SCALE2		0.04f
#define LPR_TM_SELECT_SCALE		0.5f
#define LPR_TM_SELECT_BOUND		0.5f

#define LPR_AN_CBUFF1_SIZE		3
#define LPR_AN_CBUFF2_SIZE		3
#define LPR_AN_CATCH_SCALE1		0.7f
#define LPR_AN_CATCH_SCALE2		0.5f

#define LPR_NE_DATA_MAXSIZE		20

#define LPR_EX_DATA_MAXSIZE		20
#define LPR_EX_TYPE2_SCALE1		0.5f
#define LPR_EX_TYPE2_SCALE2		0.6f

#define LPR_TRACKER_PADDSCALE_W		1.5f
#define LPR_TRACKER_PADDSCALE_H		0.5f
#define LPR_TRACKER_PARAM_SCALE		0.2f

extern uchar	_8bit_buff_1[52500];
extern uchar	_8bit_buff_2[52500];
extern uchar	_8bit_buff_3[52500];
extern int	_32bit_buff[52500];
extern int	_global_FrmOrigH;


class ClusterManager
{
private:
	bool		flag;
	int		capacity;
	DisjointSet	djt;

	void make(AREA, BlobManager*);
	int classify(AREA, BlobManager*);
	void sort();
public:
	int		nCluster;
	DBLinkedList	*cluster;

	ClusterManager() : flag(false), cluster(NULL) {}

	void allocate(int buffsize, int* buff, int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] ClusterManager(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = cap;
		cluster = new DBLinkedList[capacity];
		djt.regist(buffsize, buff);
		nCluster = capacity;
		flag = true;
	}

	void clear()
	{
		int i;

		if(cluster)
		{
			for(i = 0; i < capacity; i++)
			{
				cluster[i].clear();
			}
			delete(cluster);
			cluster = NULL;
		}
		capacity = 0;
		nCluster = 0;
		djt.clear();
		flag = false;
	}

	void init()
	{
		int i;

		if(!flag)
		{
			fprintf(stderr, "[error] ClusterManager(init) : not allocated yet.\n");
			exit(-1);
		}

		for(i = 0; i < nCluster; i++)
		{
			cluster[i].init();
		}
		nCluster = 0;
		djt.init();
	}

	int process(AREA, BlobManager*);
};

class TupleManager
{
private:
	bool	flag;
	int	capacity;
	Histogram hist;

	int select(AREA, BlobManager*, DBLinkedList*, DBLinkedList*, int, TUPLE*);
	int check(BlobManager*, ClusterManager*, TUPLE, int, int);
	void bestfit(AREA, uchar*, BlobManager*, RECT*);
public:
	int	nTuple;
	TUPLE	*tuple;
	
	TupleManager() : flag(false), tuple(NULL) {}
	
	void allocate(int hcap, int tcap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] TupleManager(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = tcap;
		tuple = (TUPLE*)malloc(sizeof(TUPLE) * capacity);
		hist.allocate(hcap);
		flag = true;
	}

	void clear()
	{
		if(tuple)
		{
			free(tuple);
			tuple = NULL;
		}
		capacity = 0;
		nTuple = 0;
		hist.clear();
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] TupleManager(init) : not allocated yet.\n");
			exit(-1);
		}

		nTuple = 0;
	}
	
	int process(AREA, uchar*, BlobManager*, ClusterManager*, RECT*);
};

class MvAvgFilter
{
private:
	bool	flag;

	int get_MvAvg_from_buff()
	{
		int i;
		int cpptr = buff.ptr;
		int avg = 0;

		for(i = 0; i < buff.capacity; i++)
		{
			avg += buff.read(&cpptr);
		}

		return (avg/buff.capacity);
	}
public:
	CircularBuff buff;
	
	MvAvgFilter() :flag(false) {}

	void allocate(int capacity)
	{
		if(flag)
		{
			fprintf(stderr, "[error] MvAvgFilter(allocate) : already allocated.\n");
			exit(-1);
		}

		if(capacity < 2)
		{
			fprintf(stderr, "[error] MvAvgFilter(allocate) : invalid capacity(%d).\n",capacity);
			exit(-1);
		}

		buff.allocate(capacity);
		flag = true;
	}

	void clear()
	{
		buff.clear();
		flag = false;
	}
	
	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] MvAvgFilter(init) : not allocated yet.\n");
			exit(-1);
		}
		buff.init();
	}

	//caution: not buff.init() in this function.
	void process(int* hist, int size)
	{
		int i, k;
		int offset;

		if(!flag)
		{
			fprintf(stderr, "[error] MvAvgFilter(process) : not allocated yet.\n");
			exit(-1);
		}

		offset = (this->buff.capacity) >> 1;
		
		for(k = 0; k < 3; k++)
		{
			for(i = 0; i < size; i++)
			{
				buff.write(hist[i]);
				if(i < this->buff.capacity - 1) continue;

				hist[i-offset] = get_MvAvg_from_buff();
			}

			memset(hist, 0x00, sizeof(int) * offset);
			memset(&hist[size - offset], 0x00, sizeof(int) * offset);
		}
	}
};

class ExtremevCatcher
{
private:
	bool flag;
public:
	CircularBuff buff;
	
	ExtremevCatcher() : flag(false) {}

	void allocate(int capacity)
	{
		if(flag)
		{
			fprintf(stderr, "[error] ExtremevCatcher(allocate) : already allocated.\n");
			exit(-1);
		}

		if(capacity != 3)
		{
			fprintf(stderr, "[error] ExtremevCatcher(allocate) : invalid capacity(%d).\n", capacity);
			exit(-1);
		}

		buff.allocate(capacity);
		flag = true;
	}

	void clear()
	{
		buff.clear();
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] ExtvCatcher(init) : not allocated yet.\n");
			exit(-1);
		}
		buff.init();
	}

	//caution : not buff.init() in this functin.
	int process()
	{
		int i;
		int cpptr = buff.ptr;
		int offset = (buff.capacity) >> 1;
		int Right = 0;
		int Left = 0;

		for(i = 0; i < offset; i++)
		{
			Right += buff.read(&cpptr);
		}

		if(--cpptr < 0) cpptr = buff.capacity - 1;

		for(i = 0; i < offset; i++)
		{
			Left += buff.read(&cpptr);
		}

		if(Left > 0 && Right < 0) return 0;

		return -1;
	}
};



class Analyst
{
private:
	bool			flag;

	int			capacity;
	int			size;
	int			*buff1;
	int			*buff2;
	int			*hist;

	MvAvgFilter		maf;
	ExtremevCatcher		evc;

	void makeDHist(AREA, uchar*, RECT);
	int catchExtV(RECT, int*, int*);
public:
	LinkedList	ll;

	Analyst() : flag(false), buff1(NULL), buff2(NULL), hist(NULL) {}

	void allocate(int hbcap, int mcap, int ecap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] Analyst(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = hbcap;
		buff1 = (int*)malloc(sizeof(int) * capacity);
		buff2 = (int*)malloc(sizeof(int) * capacity);
		hist = (int*)malloc(sizeof(int) * capacity);
		maf.allocate(mcap);
		evc.allocate(ecap);
		size = capacity;
		flag = true;
	}

	void clear()
	{
		if(buff1)
		{
			free(buff1);
			buff1 = NULL;
		}

		if(buff2)
		{
			free(buff2);
			buff2 = NULL;
		}

		if(hist)
		{
			free(hist);
			hist = NULL;
		}

		capacity = 0;
		size = 0;
		maf.clear();
		evc.clear();
		ll.clear();
		flag = false;
	}
	
	void init(int newsize)
	{
		if(!flag)
		{
			fprintf(stderr, "[error] Analyst(init) : not allocated yet.\n");
			exit(-1);
		}
		
		memset(buff1, 0x00, sizeof(int) * size);
		memset(buff2, 0x00, sizeof(int) * size);

		maf.init();
		evc.init();

		ll.init();

		size = newsize;
	}

	int process(AREA, uchar*, RECT, int*, int*);	
};

class NoiseEraser
{
private:
	bool		flag;
	int		tcapacity;

	int		lcapacity;
	int		mem_min_h;
	int		mem_max_h;
	LinkedList	*ll;
	uchar		*mark;
	void SimpleErase(AREA, BlobManager*);
	int DetailErase(AREA, BlobManager*);
	void sort();
public:
	int		nToken;
	RECT		*token;

	NoiseEraser():flag(false), ll(NULL), mark(NULL), token(NULL) {}

	void allocate(int lcap, int tcap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] NoiseEraser(allocate) : already allocated.\n");
			exit(-1);
		}

		tcapacity = tcap;
		token = (RECT*)malloc(sizeof(RECT) * tcapacity);

		lcapacity = lcap;
		ll = new LinkedList[lcapacity];

		mem_min_h = 0;
		mem_max_h = lcapacity-1;
		
		flag = true;
	}

	void clear()
	{
		int i;
		
		if(ll)
		{
			for(i = 0; i < lcapacity; i++)
			{
				ll[i].clear();
			}

			delete []ll;
			ll = NULL;
		}
		
		if(token)
		{
			free(token);
			token = NULL;
		}

		nToken = 0;
		tcapacity = 0;
		lcapacity = 0;
		flag = false;
	}

	//do not init ll in this function.
	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] NoiseEraser(init) : not allocated yet.\n");
			exit(-1);
		}
		
		nToken = 0;
	}

	int process(AREA, BlobManager*);
};

class Extractor
{
private:
	bool	flag;

	int	capacity;
	int	pcnt;
	int	mcnt;

	int	*pixel;
	int	*match;

	int type1(AREA, uchar*, NoiseEraser*, Analyst*,RECT);
	int type2(NoiseEraser*, Analyst*, RECT, int);
	void extraction(AREA, uchar*, int*, RECT*, RECT*);
public:
	
	RECT	token[7];

	Extractor() :flag(false), pixel(NULL), match(NULL) {}

	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] Extractor(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = cap;
		pixel = (int*)malloc(sizeof(int) * capacity);
		match = (int*)malloc(sizeof(int) * capacity);

		pcnt = capacity;
		mcnt = capacity;

		flag = true;
	}
	void clear()
	{
		if(pixel)
		{
			free(pixel);
			pixel = NULL;
		}

		if(match)
		{
			free(match);
			match = NULL;
		}

		capacity = 0;
		pcnt = 0;
		mcnt = 0;
		memset(token, 0x00, sizeof(RECT) * 7);
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] Extractor(init) : not allocated yet.\n");
			exit(-1);
		}
	
		memset(pixel, 0x00, sizeof(int) * pcnt);
		memset(match, 0xFF, sizeof(int) * mcnt);
		memset(token, 0x00, sizeof(int) * 7);
		pcnt = 0;
		mcnt = 0;
	}

	int process(AREA, uchar*, uchar*, NoiseEraser*, Analyst*, int, int, int*, RECT*, RECT*);
};

class LPRManager
{
private:
	bool		flag;

	VpixelManager	vm;
	BlobManager	bm;

	ClusterManager	cm;
	TupleManager	tm;
	Analyst		an;
	NoiseEraser	ne;
	Extractor	ex;

	int		param;
	RECT		pos;
	RECT		pos_tk[7];

	int determine_lppos_yaxis(AREA, uchar*, uchar*);
	int determine_lppos_xaxis(AREA, uchar*, uchar*, uchar*,  uchar*, int*);
	void move(IMAGE*, IMAGE*, IMAGE*, RECT, RECT*);
public:

	LPRManager() : flag(false) {}

	void allocate(
			AREA szbuff, int* buff,
			int vsize,
			int bsize,
			int csize,
			int tsize,
			int nsize,
			int esize,
			int asize1, int asize2
							)
					
	{
		if(flag)
		{
			fprintf(stderr, "[error] LPRManager(allocate) : already allcated.\n");
			exit(-1);
		}
		
		vm.allocate(vsize);
		bm.allocate(szbuff, buff, bsize);
		cm.allocate(szbuff.w * szbuff.h, buff, csize);
		tm.allocate(szbuff.w, tsize);
		an.allocate(szbuff.w, asize1, asize2); 
		ne.allocate(szbuff.h, nsize);
		ex.allocate(esize);
		flag = true;
	}

	void clear()
	{
		vm.clear();
		cm.clear();
		tm.clear();
		bm.clear();
		an.clear();
		ne.clear();
		ex.clear();

		param = 0;
		memset(pos_tk, 0x00, sizeof(RECT) * 7);
		pos.write(0,0,0,0);
		flag = false;
	}

	void init()
	{
		param = 0;
		memset(pos_tk, 0x00, sizeof(RECT) * 7);
		pos.write(0,0,0,0);
	}

	RECT read_pos()
	{
		return this->pos;
	}

	int read_param()
	{
		return this->param;
	}

	int recognize(IMAGE*, IMAGE*, IMAGE*);

	int _Tracker_recognize(IMAGE*, IMAGE*, IMAGE*, ObjTracker*);
};

#endif // LPR_H_INCLUDED
