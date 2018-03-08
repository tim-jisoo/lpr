#include "../Header/meanshift.hpp"

inline float Euclidean_Distance(POINT _old, POINT _new)
{
	return sqrt((float)((_new.y - _old.y)*(_new.y - _old.y) + (_new.x - _old.x)*(_new.x - _old.x)));
}

float Kernel_Epanechnikov(float domain)
{
	const float c = 3.0f/4.0f;
	
	if(domain < 1.0f)
		return c * (1.0f - domain);
	else 
		return 0.0f;
}

int meanshift(AREA size_tracking_region, uchar* data_tracking_region, RECT prediction, RECT *measurement)
{
	const float wrate = MEANSHIFT_PADDING_SCALE_W;
	const float hrate = MEANSHIFT_PADDING_SCALE_H;
	const int direction = MEANSHIFT_PADDING_DIRECTION;
	int dx, dy, dw, dh;
	int x, y;
	float xnumerator;
	float ynumerator;
	float denominator;
	float sqr, sqnorm, kval, eval, term;
	POINT yold, ynew;
	RECT wdw, base;

	//Derive Box which set position of license plate in tracking region.
	base.size.w = prediction.size.w;
	base.size.h = prediction.size.h;
	base.orig.x = (size_tracking_region.w - prediction.size.w) >> 1;
	base.orig.y = (size_tracking_region.h - prediction.size.h) >> 1;

	//Make Window Box which is used for meanshift algorithm.
	dx = direction * (dw = (int)((float)base.size.w * wrate)) >> 1;
	dy = direction * (dh = (int)((float)base.size.h * hrate)) >> 1;
	if((wdw.orig.x = base.orig.x + dx) < 0){
		fprintf(stdout, "[notice] meanshift : (1) window out of boundary.\n");
		return -1;
	}
	if((wdw.orig.y = base.orig.y + dy) < 0){
		fprintf(stdout, "[notice] meanshift : (2) window out of boundary.\n");
		return -1;
	}
	if(wdw.orig.x + (wdw.size.w = base.size.w + dw) >= size_tracking_region.w){
		fprintf(stdout, "[notice] meanshift : (3) window out of boundary.\n");
		return -1;
	}
	if(wdw.orig.y + (wdw.size.h = base.size.h + dh) >= size_tracking_region.h){
		fprintf(stdout, "[notice] meanshift : (4) window out of boundary.\n");
		return -1;
	}

	
	ynew.x = ((wdw.orig.x << 1) + wdw.size.w) >> 1;
	ynew.y = ((wdw.orig.y << 1) + wdw.size.h) >> 1;
	sqr = (float)((wdw.size.w * wdw.size.w + wdw.size.h * wdw.size.h) >> 2);
	do{
		xnumerator = 0.0f;
		ynumerator = 0.0f;
		denominator = 0.0f;

		for(y = wdw.orig.y ; y < wdw.orig.y + wdw.size.h; y++)
		{
			for(x = wdw.orig.x ; x < wdw.orig.x + wdw.size.w ; x++)
			{
				sqnorm = (float)((ynew.y - y)*(ynew.y - y) + (ynew.x - x)*(ynew.x - x));
				kval = Kernel_Epanechnikov(sqnorm/sqr);
				eval = (float)data_tracking_region[x + y * size_tracking_region.w];
				term = kval * eval;

				denominator += term;
				xnumerator += (float)(x * term);
				ynumerator += (float)(y * term);
			}
		}

		yold = ynew;
		ynew.x = (int)(xnumerator / denominator);
		ynew.y = (int)(ynumerator / denominator);
		wdw.orig.x += (ynew.x - yold.x);
		wdw.orig.y += (ynew.y - yold.y);

		if( wdw.orig.x < 0 ||wdw.orig.y < 0
		||wdw.orig.x + wdw.size.w >= size_tracking_region.w
		||wdw.orig.y + wdw.size.h >= size_tracking_region.h){
			fprintf(stdout, "[notice] meanshift : (5) window out of range.\n");
			return -1;
		}
	}while(Euclidean_Distance(yold, ynew) > (float)2.0f);

	wdw.orig.x -= dx;
	wdw.orig.y -= dy;
	wdw.size.w -= dw;
	wdw.size.h -= dh;

	measurement->orig.x = prediction.orig.x - base.orig.x + wdw.orig.x;
	measurement->orig.y = prediction.orig.y - base.orig.y + wdw.orig.y;
	measurement->size.w = prediction.size.w;
	measurement->size.h = prediction.size.h;

	return 0;
}
