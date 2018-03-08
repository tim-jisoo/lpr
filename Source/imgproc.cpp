#include "../Header/imgproc.hpp"

void imgproc_sobel(AREA size, uchar* img, uchar* hedge, uchar* vedge)
{
	int r, c;
	int sum;
	int right,left;
	int top, bot;

    	r = (size.h - 1) * size.w;

	memset(hedge, 0x00, sizeof(uchar) * size.w);
	memset(&hedge[r], 0x00, sizeof(uchar) * size.w);

	memset(vedge, 0x00, sizeof(uchar) * size.w);
	memset(&vedge[r], 0x00, sizeof(uchar) * size.w);

	for(r = 1; r < size.h - 1; r++)
	{
        	top = r - 1;
		bot = r + 1;
     
		hedge[0 + r * size.w] = 0x00;
		vedge[0 + r * size.w] = 0x00;
		
		for(c = 1; c < size.w - 1; c++)
		{
            		right = c + 1;
    		        left = c - 1;

		        //calc vertical edge
            		sum  = img[right + top * size.w];
            		sum += img[right + r   * size.w] << 1;
		        sum += img[right + bot * size.w];
            		sum -= img[left  + top * size.w];
           		sum -= img[left  + r   * size.w] << 1;
           		sum -= img[left  + bot * size.w];
           		sum = ((sum > 0) ? sum : -sum) >> 3;
           		vedge[c + r * size.w] = sum;

      			//calc horizontal edge
          		sum  = img[right + bot * size.w];
          		sum += img[c     + bot * size.w] << 1;
	  		sum += img[left  + bot * size.w];
	  		sum -= img[right + top * size.w];
          		sum -= img[c     + top * size.w] << 1;
         		sum -= img[left  + top * size.w];
         		sum = ((sum > 0) ? sum : -sum) >> 3;
          		hedge[c + r * size.w] = sum;
	        }

		hedge[(size.w - 1) + r * size.w] = 0x00;
		vedge[(size.w - 1) + r * size.w] = 0x00;
	}
}

int imgproc_sobel_local(AREA size, uchar* src, uchar* dst, RECT tgt, RECT base)
{
	int r, c;
	int sum, param;
	int right, left, top, bot;

	param = 0;
    	r = (tgt.size.h - 1) * tgt.size.w;
	memset(dst, 0x00, sizeof(uchar) * tgt.size.w);
	memset(&dst[r], 0x00, sizeof(uchar) * tgt.size.w);

	for(r = 1; r < tgt.size.h - 1; r++)
	{
        	top = r - 1;
		bot = r + 1;
		dst[0 + r * tgt.size.w] = 0x00;
		for(c = 1; c < tgt.size.w - 1; c++)
		{
            		right = c + 1;
    		        left = c - 1;
			
			sum  = src[(tgt.orig.x + right) + (tgt.orig.y + top) * size.w];
			sum += src[(tgt.orig.x + right) + (tgt.orig.y + r  ) * size.w] << 1;
			sum += src[(tgt.orig.x + right) + (tgt.orig.y + bot) * size.w];
			sum -= src[(tgt.orig.x + left ) + (tgt.orig.y + top) * size.w];
			sum -= src[(tgt.orig.x + left ) + (tgt.orig.y + r  ) * size.w] << 1;
			sum -= src[(tgt.orig.x + left ) + (tgt.orig.y + bot) * size.w];
			sum = (sum > 0) ? sum : -sum;
			sum >>= 3;
			dst[c + r * tgt.size.w] = sum;

			if( 
				(tgt.orig.x + c < base.orig.x)
				||(tgt.orig.x + c >= base.orig.x + base.size.w)
				||(tgt.orig.y + r < base.orig.y)
				||(tgt.orig.y + r >= base.orig.y + base.size.h)
										) continue;
			param += sum;
	        }
		dst[(tgt.size.w - 1) + r * tgt.size.w] = 0x00;
	}
	
	return param;
}

void imgproc_haar(AREA size, uchar* hedge, VpixelManager* pVm)
{
	int r, c, k;
	int state, cnt, cond;
	int curr, before;
	int bound_under;
	int bound_upper;

	bound_under = (int)((float)size.w * IMGPROC_HAAR_BOUND_SCALE1);
	bound_upper = (int)((float)size.w * IMGPROC_HAAR_BOUND_SCALE2);

	pVm->init();

	//there is no any data when processing img boundary.
	//because sobel does not left result in case of img boundary.
	for(r = 1; r < size.h - 1; r++)
	{
        	state = 0;
		cnt = 0;

		for(c = 1; c < size.w - 1; c++)
		{
			curr   = hedge[c + r     * size.w];
			before = hedge[c + (r-1) * size.w];
            		cond = (curr > before) ? 1 : 0;

			if(!state)
			{
				if(!cond) continue;
				state = 1;
				cnt++;
			}
			else
			{
				if(cond)
				{
					cnt++;
					continue;
				}

				if( cnt >= bound_under && cnt <= bound_upper)
				{
					for(k = cnt; k >= 1; k--)
						pVm->insert(c-k, r, 0);
				}
				state = 0;
				cnt = 0;
			}
        	}
	}
}


static void integral(AREA size, uchar* src, int* buff32)
{
	int r,c;

	memset(buff32, 0x00, sizeof(int) * size.w * size.h);

	for(c = 0; c < size.w; c++)
		buff32[c] = (int)src[c];

	for(r = 1; r < size.h; r++)
    	{
        	for(c = 0; c < size.w; c++)
	        {
        		buff32[c + r * size.w]
			    = buff32[c + (r-1) * size.w] + (int)src[c + r * size.w];
	        }
    	}

	for(r = 0; r < size.h; r++)
   	{
        	for(c = 1; c < size.w; c++)
	        {
        		buff32[c + r * size.w] += buff32[(c-1) + r * size.w];
	        }
    	}
}

static int wdwsum(AREA size, int* buff32, int c, int r, AREA wdw)
{
	int retVal, right_top, left_bot;

	assert(c >=1 && r >= 1);

	c--;
	r--;

	right_top = c + wdw.w;
	left_bot = r + wdw.h;

	assert(right_top < size.w && left_bot < size.h);

	retVal  = buff32[right_top + left_bot * size.w];
	retVal += buff32[c         + r        * size.w];
	retVal -= buff32[c         + left_bot * size.w];
	retVal -= buff32[right_top + r        * size.w];
	
	assert(retVal >= 0);

	return retVal;
}

static void binarize(AREA size, uchar* src, VpixelManager* pVm, int* buff32, AREA wdw, int bias)
{
	int r, c;
	int center_r, center_c;
	int bound_r, bound_c;
	int offset_r, offset_c;
	int factor, val;
	
	bound_r = size.h - wdw.h;
	bound_c = size.w - wdw.w;
	offset_r = wdw.h >> 1;
	offset_c = wdw.w >> 1;
	factor  = wdw.w * wdw.h;

	integral(size, src, buff32);

	pVm->init();

	for(r = 1; r <= bound_r; r++)
	{
		center_r = r + offset_r;
		for(c = 1; c <= bound_c; c++)
		{
			center_c = c + offset_c;

			val = wdwsum(size, buff32, c, r, wdw) / factor;
			
			if(src[center_c + center_r * size.w] >= val - bias) continue;

			if(c < size.w - wdw.w && r < size.h - wdw.h)
			{
				pVm->insert(c, r, 0);
			}
		}
	}
}

static void attach_padding(AREA size, uchar* src, uchar* dst, RECT pos, AREA wdw)
{
	int r;
	int wdst = pos.size.w + wdw.w;
	int hdst = pos.size.h + wdw.h;
	int offset_w = wdw.w >> 1;
	int offset_h = wdw.h >> 1;

	memset(dst, 0x00, sizeof(uchar) * wdst * hdst);

	for(r = 0; r < pos.size.h ; r++)
	{
		memcpy(
			&dst[(offset_w) + (offset_h + r) * wdst],
			&src[(pos.orig.x) + (pos.orig.y + r) * size.w],
			sizeof(uchar) * pos.size.w
		      );
	}
}

void imgproc_binarize(AREA size, uchar* src, VpixelManager *pVm, RECT pos, uchar* buff8, int* buff32)
{
	int bias;
	AREA wdw;
	AREA size_with_padding;

	bias = IMGPROC_BINARIZE_BIAS;
	wdw.write(IMGPROC_BINARIZE_WDW_W, IMGPROC_BINARIZE_WDW_H);

	attach_padding(size, src, buff8, pos, wdw);

	size_with_padding.write(wdw.w + pos.size.w, wdw.h + pos.size.h);
	binarize(size_with_padding, buff8, pVm, buff32, wdw, bias);	
}

static int interpolation(int x0, int x1, int x2, int x3, float x)
{
        float a = (float)(0  - (x0 >> 1) + ((3*x1) >> 1) - ((3*x2) >> 1) + (x3 >> 1));
        float b = (float)(x0 - ((5*x1) >> 1) + (x2 << 1) - (x3 >> 1));
        float c = (float)(0  - (x0 >> 1) + (x2 >> 1));
        float d = (float)x1;

        float val = a*x*x*x + b*x*x + c*x + d;

        if(val < 0.0f)
            return 0;

        else if (val > 255.0f)
            return 255;

        else
            return (int)val;
}

int imgproc_resize(IMAGE* img, IMAGE* tk, RECT oldpos, int idx)
{
	RECT newpos; 
	const int width_buff1 = IMGPROC_RESIZE_BUFF1W;
	const int width_buff2 = IMGPROC_RESIZE_BUFF2W;
	
	uchar   buff1[28 * 50];
	uchar   buff2[28 * 22];

	int     r,c;
	float   ratio;
	float   fidxSample[28];
	int     idxSample[28];
	int v1, v2, v3, v4;

	newpos.write(0,0,IMGPROC_RESIZE_NEWPOS_W, IMGPROC_RESIZE_NEWPOS_H);
	if(oldpos.size.w > width_buff1)
	{
		fprintf(stdout, "[notice] imgproc.cpp(imgproc_resize) : token has too much width size.\n");
		return -1;
	}

	//[step1] regard to height.
	ratio = (float)oldpos.size.h / (float)newpos.size.h;
	
	//height scale up
    	if(ratio < 1.0f)
    	{
        	for(r = 0; r < newpos.size.h; r++)
        	{
			fidxSample[r] = ratio * (float)r;
			idxSample[r] = (int)(ratio * (float)r);
		}
		
		for(c = 0; c < oldpos.size.w; c++)
		{
            		v1 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[0]) * img->size.w];
		        v2 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[0]) * img->size.w];
			v3 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[1]) * img->size.w];
            		v4 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[2]) * img->size.w];
            		buff1[c + 0 * width_buff1] = interpolation(v1, v2, v3, v4, fidxSample[0] - (float)idxSample[0]);

			for(r = 1; r < newpos.size.h-2; r++)
			{
            			v1 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[r-1]) * img->size.w];
            			v2 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[r+0]) * img->size.w];
            			v3 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[r+1]) * img->size.w];
            			v4 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[r+2]) * img->size.w];
            			buff1[c + r * width_buff1] = interpolation(v1, v2, v3, v4, fidxSample[r] - (float)idxSample[r]);
			}
			v1 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-3]) * img->size.w];
			v2 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-2]) * img->size.w];
			v3 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-1]) * img->size.w];
			v4 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-1]) * img->size.w];
			buff1[c + (newpos.size.h-2) * width_buff1] = interpolation(v1, v2, v3, v4, fidxSample[newpos.size.h-2] - (float)idxSample[newpos.size.h-2]);

			v1 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-2]) * img->size.w];
			v2 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-1]) * img->size.w];
			v3 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-1]) * img->size.w];
			v4 = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[newpos.size.h-1]) * img->size.w];
			buff1[c + (newpos.size.h-1) * width_buff1] = interpolation(v1, v2, v3, v4, fidxSample[newpos.size.h-1] - (float)idxSample[newpos.size.h-1]);
		}
	}

    	//height scale down => sampling
    	else
    	{
		for(r = 0; r < newpos.size.h; r++)
		{
			idxSample[r] = (int)(ratio * (float)r);
		}

		for(c = 0; c < oldpos.size.w; c++)
		{
			for(r = 0; r < newpos.size.h; r++)
			{
				buff1[c + r * width_buff1] = img->data[(oldpos.orig.x + c) + (oldpos.orig.y + idxSample[r]) * img->size.w];
			}
		}
	}


	//[step2] regard to width.
	ratio = (float)oldpos.size.w / (float)newpos.size.w;

 	//width scale up
  	if(ratio < 1.0f)
	{
        	for(c = 0; c < newpos.size.w; c++)
        	{
        		fidxSample[c] = ratio * (float)c;
         		idxSample[c] = (int)(ratio * (float)c);
		}

		for(r = 0; r < newpos.size.h; r++)
        	{
			v1 = buff1[idxSample[0] + r * width_buff1];
			v2 = buff1[idxSample[0] + r * width_buff1];
			v3 = buff1[idxSample[1] + r * width_buff1];
			v4 = buff1[idxSample[2] + r * width_buff1];

			buff2[0 + r * width_buff2] = interpolation(v1, v2, v3, v4, fidxSample[0] - (float)idxSample[0]);

			for(c = 1; c < newpos.size.w - 2; c++)
			{
				v1 = buff1[idxSample[c-1] + r * width_buff1];
				v2 = buff1[idxSample[c+0] + r * width_buff1];
				v3 = buff1[idxSample[c+1] + r * width_buff1];
				v4 = buff1[idxSample[c+2] + r * width_buff1];

				buff2[c + r * width_buff2] = interpolation(v1, v2, v3, v4, fidxSample[c] - (float)idxSample[c]);
			}

			v1 = buff1[idxSample[newpos.size.w-3] + r * width_buff1];
			v2 = buff1[idxSample[newpos.size.w-2] + r * width_buff1];
			v3 = buff1[idxSample[newpos.size.w-1] + r * width_buff1];
			v4 = buff1[idxSample[newpos.size.w-1] + r * width_buff1];

			buff2[newpos.size.w-2 + r * width_buff2]
			 = interpolation(v1, v2, v3, v4, fidxSample[newpos.size.w-2] - (float)idxSample[newpos.size.w-2]);

			v1 = buff1[idxSample[newpos.size.w-2] + r * width_buff1];
			v2 = buff1[idxSample[newpos.size.w-1] + r * width_buff1];
			v3 = buff1[idxSample[newpos.size.w-1] + r * width_buff1];
			v4 = buff1[idxSample[newpos.size.w-1] + r * width_buff1];

			buff2[newpos.size.w-1 + r * width_buff2]
			 = interpolation(v1, v2, v3, v4, fidxSample[newpos.size.w-1] - (float)idxSample[newpos.size.w-1]);
		}
	}

	//width scale down => sampling
	else
	{
		for(c = 0; c < newpos.size.w; c++)
		{
			idxSample[c] = (int)(ratio * (float)c);
		}

		for(r = 0; r < newpos.size.h; r++)
		{
			for(c = 0; c < newpos.size.w; c++)
			{
				buff2[c + r * width_buff2] = buff1[idxSample[c] + r * width_buff1];
			}
		}
	}

    	//License plate has 7 tokens.
	newpos.orig.y = (tk->size.h    - newpos.size.h) /2;
	newpos.orig.x = (tk->size.w / 7) * idx + (tk->size.w / 7 - newpos.size.w)/2;

	for(r = 0; r < newpos.size.h; r++)
	{
		memcpy(
			&tk->data[(newpos.orig.x) + (newpos.orig.y + r) * tk->size.w],
			&buff2[r * width_buff2],
			sizeof(uchar) * newpos.size.w
		);
	}

	return 0;
}
