#ifndef IMGPROC_H
#define IMGPROC_H

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "datatype.hpp"

#define IMGPROC_HAAR_BOUND_SCALE1	0.1f
#define IMGPROC_HAAR_BOUND_SCALE2	0.5f

#define IMGPROC_BINARIZE_WDW_W		5	
#define IMGPROC_BINARIZE_WDW_H		5
#define IMGPROC_BINARIZE_BIAS		5

#define IMGPROC_RESIZE_NEWPOS_W		22
#define IMGPROC_RESIZE_NEWPOS_H		28
#define IMGPROC_RESIZE_BUFF1W		50
#define IMGPROC_RESIZE_BUFF2W		(IMGPROC_RESIZE_NEWPOS_W)	

void imgproc_sobel(AREA, uchar*,  uchar*, uchar*);
int imgproc_sobel_local(AREA, uchar*, uchar*, RECT, RECT);
void imgproc_haar(AREA, uchar*, VpixelManager*);
void imgproc_binarize(AREA, uchar* , VpixelManager*, RECT, uchar*, int*);
int imgproc_resize(IMAGE*, IMAGE*, RECT, int);
#endif

