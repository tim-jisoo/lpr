#include "../Header/blob.hpp"

#ifdef ACTIVATE_OPENCV
void rainbow(char* name, AREA size, VpixelManager *pVm, DisjointSet* pdjt)
{
	int i, r, c, val;
	int root;
	const int nColor = 42;
	int color[nColor][3] = {
		{255,0,0}, {0,255,0}, {0,0,255},
		{0,255,255}, {255,0,255}, {255,255,0},
		{36,28,237}, {237,36,28}, {28,237,36},
		{39,127,255},{255,39,127}, {127,255,39},
		{76,177,34}, {34, 76,177}, {177,34,76},
		{162,0,232}, {232,162,0}, {0,232,162},
		{164,73,163}, {163,164,73}, {73,163,164},
		{201,174,255}, {255,201,174}, {174,255,201},
		{29,230,181}, {181,29,230}, {230,181,29},
		{232,162,0}, {0,232,162}, {162,0,232},
		{230,181,29}, {29,230,181}, {181,29,230},
		{231,191,200}, {200,231,191}, {191,200,231},
		{87,122,185}, {185,87,122}, {122,185,87},
		{45,200,23}, {23,45,200}, {200,23,45}
	};

	cv::Mat show(size.h, size.w, CV_8UC3);
	uchar* data = show.data;

	memset(data, 0x00, sizeof(uchar) * 3 * show.cols * show.rows);

	for(i = 0; i < pVm->size; i++)
	{
		r = pVm->data[i].y;
		c = pVm->data[i].x;
		val = pVm->data[i].val;

		root = pdjt->SimpleFind(val);

		data[3 * (c + r * size.w) + 0] = color[root % nColor][0];
		data[3 * (c + r * size.w) + 1] = color[root % nColor][1];
		data[3 * (c + r * size.w) + 2] = color[root % nColor][2];
	}

	cv::imshow(name, show);
}
#endif


int BlobManager::process(AREA size, VpixelManager *pVm)
{
	if(!flag)
	{
		fprintf(stderr, "[error] BlobManager(process) : not allocated yet.\n");
		exit(-1);
	}

	this->init();	

	if(this->make_label_img(size, pVm) < 0) return -1;

#ifdef ACTIVATE_OPENCV
	//rainbow("BLOB", size, pVm, &this->djt);
#endif

	this->derive_blob_info(size, pVm);

	return this->nBlob;
}

int BlobManager::make_label_img(AREA size, VpixelManager *pVm)
{
	int i;
	int xnew, ynew, xold, yold;
	int root1, root2;
	int nblob = 0, maxlabel = 0;
	int *line_above = (int*)malloc(sizeof(int) * size.w); 
	int *line_curr  = (int*)malloc(sizeof(int) * size.w);
	memset(line_above, 0x00, sizeof(int) * size.w);
	memset(line_curr, 0x00, sizeof(int) * size.w);
	
	/********************************************/
	/*[step1] make label space image and parent.*/
	/********************************************/

	xold = pVm->data[0].x;
	yold = pVm->data[0].y;
	pVm->data[0].val = ++maxlabel;
	line_curr[xold] = pVm->data[0].val;
	for(i = 1; i < pVm->size; i++)
	{
		xnew = pVm->data[i].x;
		ynew = pVm->data[i].y;
	
		//When entering new line.
		if(ynew - yold > 0)
		{
			//By entering new line, before token is forcibly fragmented.
			//In regard to past pixel.
			if(xold < size.w-1)
			{
				if(!line_above[xold] && line_above[xold+1])
				{
					root1 = this->djt.CollapsingFind(line_above[xold+1]);
					root2 = this->djt.CollapsingFind(line_curr[xold]);
					if(root1 != root2) this->djt.Union(root1, root2);
				}
			}

			if(ynew - yold > 1)
			{
				memset(line_above, 0x00, sizeof(int) * size.w);
			}
			else
			{
				memcpy(line_above, line_curr, sizeof(int) * size.w);
			}
			
			memset(line_curr, 0x00, sizeof(int) * size.w);


			//In regard to current pixel.
			pVm->data[i].val = ++maxlabel;
			line_curr[xnew] = pVm->data[i].val;
			if(xnew > 0)
			{
				//case1
				if(!line_above[xnew-1] && !line_above[xnew])
				{
					//No any instructions.
				}
				
				//case2
				else if(line_above[xnew-1] && !line_above[xnew])
				{
					root1 = this->djt.CollapsingFind(line_above[xnew-1]);
					this->djt.Union(root1, line_curr[xnew]);
				}

				//case3, 4
				else
				{
					root1 = this->djt.CollapsingFind(line_above[xnew]);
					this->djt.Union(root1, line_curr[xnew]);
				}
			}
			else
			{
				if(line_above[xnew])
				{
					root1 = this->djt.CollapsingFind(line_above[xnew]);
					this->djt.Union(root1, line_curr[xnew]);
				}
			}
		}

		//When processing same line.
		else
		{
			//When current pixel is next to the past pixel.
			if(xnew - xold == 1)
			{
				pVm->data[i].val = maxlabel;
				line_curr[xnew] = pVm->data[i].val;

				if(!line_above[xnew-1] && line_above[xnew])
				{
					root1 = this->djt.CollapsingFind(line_above[xnew]);
					root2 = this->djt.CollapsingFind(line_curr[xnew]);
					if(root1 != root2) this->djt.Union(root1, root2);
				}
			}

			//When current pixel is apart from the past pixel.
			else
			{
				pVm->data[i].val = ++maxlabel;	
				line_curr[xnew] = pVm->data[i].val;

				//In regard to past pixel.
				if(!line_above[xold] && line_above[xold+1])
				{
					root1 = this->djt.CollapsingFind(line_above[xold+1]);
					root2 = this->djt.CollapsingFind(line_curr[xold]);
					if(root1 != root2) this->djt.Union(root1, root2);
				}

				//In regard to current pixel.
				/*case 1*/
				if(!line_above[xnew-1] && !line_above[xnew])
				{
					//No any instructions.
				}

				/*case 2*/
				else if(line_above[xnew-1] && !line_above[xnew])
				{
					root1 = this->djt.CollapsingFind(line_above[xnew-1]);
					this->djt.Union(root1, line_curr[xnew]);
				}

				/*case 3,4 */
				else
				{
					root1 = this->djt.CollapsingFind(line_above[xnew]);
					this->djt.Union(root1, line_curr[xnew]);
				}

			}
		}
	
		xold = xnew;
		yold = ynew;
	}

	free(line_above);
	free(line_curr);

	/*********************************************************/
	//[step2] count total blob number.
	//idx 0 of djt.parent means background label of image.
	/*********************************************************/
	for(i = 1; i <= maxlabel; i++)
	{
		if(this->djt.parent[i] >= 0) continue;

		if(++nblob > this->capacity)
		{
			fprintf(stdout, "[notice] BlobManager(make_label_img) : nBlob full capacity.\n");
			return -1;
		}
	}

	if(!nblob)
	{
		fprintf(stdout, "[notice] BlobManager(make_label_img) : there is no any blobs.\n");
		return -1;
	}

	this->nBlob = nblob;

	return 0;
}

void BlobManager::derive_blob_info(AREA size, VpixelManager* pVm)
{
	HashNode* ptr;
	int i, idx, root, label;
	int xnew, xold, ynew, yold;
	int max_hash_entry;
	const int CASE_CONSECUTIVE_VPIXEL = 0;
	
	max_hash_entry = (3 * this->nBlob) >> 1;
	this->ht.init(max_hash_entry);

	xold = pVm->data[0].x;
	yold = pVm->data[0].y;
	label = pVm->data[0].val;
	root = this->djt.SimpleFind(label);
	this->ht.insert(root, xold, yold, &ptr);
	for(i = 1; i < pVm->size; i++)
	{
		xnew = pVm->data[i].x;
		ynew = pVm->data[i].y;

		//when processing consecutive pixel.
		if((ynew - yold == 0) && (xnew - xold == 1))
		{
			this->ht.insert(CASE_CONSECUTIVE_VPIXEL, xnew, ynew, &ptr);
		}

		//when entering new pixel token.
		else
		{
			label = pVm->data[i].val;
			root = this->djt.SimpleFind(label);
			this->ht.insert(root, xnew, ynew, &ptr);
		}
		
		xold = xnew;
		yold = ynew;
	}

	for(idx = 0, i = 0 ; i < this->ht.size; i++)
	{
		ptr = ht.table[i];
		while(ptr)
		{
			this->data[idx++].write(ptr->param.box, ptr->param.val);
			ptr = ptr->next;
		}
	}
}



