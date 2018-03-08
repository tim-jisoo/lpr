#include "../Header/lpr.hpp"

#ifdef ACTIVATE_OPENCV
cv::Mat imgSobel;
cv::Mat imgHaar;
cv::Mat imgLp;
cv::Mat imgLpB;
cv::Mat imgLpN;
cv::Mat imgHist;
cv::Mat buffLp;
cv::Mat	buffTk;
#endif

void ClusterManager::make(AREA size, BlobManager* pBm)
{
	int a, b, nblob;
	int rootA, rootB;
	int len_ovl_w, dist_itv_h;
	int criteria;
	BLOB blobA, blobB;

	criteria = (int)((float)_global_FrmOrigH * LPR_CM_MAKE_SCALE);

	nblob = pBm->nBlob;

	//Mathmatical Combination can be implementd by setting b = a + 1.
	for(a = 0; a < nblob-1; a++)
	{
		for(b = a+1; b < nblob; b++)
		{
			rootA = this->djt.CollapsingFind(a);
			rootB = this->djt.CollapsingFind(b);
			if(rootA == rootB) continue;
			
			blobA = pBm->data[a];
			blobB = pBm->data[b];
			len_ovl_w = blobA.box.overlapw(blobB.box);
			if(len_ovl_w <= 0) continue;

			dist_itv_h = blobA.box.intervalh(blobB.box);
			if(dist_itv_h > criteria) continue;

			this->djt.Union(rootA, rootB);
		}
	}

}

int ClusterManager::classify(AREA size, BlobManager* pBm)
{
	int a, b;
	int nblob = pBm->nBlob;

	for(a = 0; a < nblob; a++)
	{
		//first, choose root label which value is minus.
		if(this->djt.parent[a] >= 0) continue;

		if(this->nCluster >= this->capacity)
		{
			fprintf(stdout, "[notice] ClusterManager(classify) : nCluster full capacity.\n");
			return -1;
		}

		for(b = 0; b < nblob; b++)
		{
			//insert root's element to linked list. 
			if(this->djt.SimpleFind(b) != a) continue;

			//1st parameter => b: blob number
			//2nd parameter => pBm->data[b].box.orig.y: ascending sort param
			this->cluster[this->nCluster].insert2(b, (pBm->data[b]).box.orig.y);
		}
	
		this->nCluster++;
	}

	if(this->nCluster < 2)
	{
		fprintf(stdout, "[notice] ClusterManager(classify) : not enough clusters.\n");
		return -1;
	}
	
	return 0;
}

void ClusterManager::sort()
{
	int a, b;
	int idx;
	DBLinkedList temp;
	
	for(a = 0; a < nCluster-1; a++)
	{
		idx = a;

		for(b = a+1; b < nCluster; b++)
		{
			if(cluster[idx].head->val2 > cluster[b].head->val2)
				idx = b;
		}

		temp = cluster[idx];
		cluster[idx] = cluster[a];
		cluster[a] = temp;
	}
}

int ClusterManager::process(AREA size, BlobManager* pBm)
{
	this->init();

	this->make(size, pBm);

	if(this->classify(size, pBm) < 0) return -1;

	this->sort();

	return 0;
}




int TupleManager::select(AREA size, BlobManager* pBm, DBLinkedList* upll, DBLinkedList* dwll, int h, TUPLE* retv)
{
	int w, min_l, max_r;
	int ovl,itv, criteria;
	float rate, rate_w, rate_h, max_rate;
	RECT up, dw;
	DNODE *upiter, *dwiter;

	criteria = (int)((float)size.w * LPR_TM_SELECT_SCALE);
	max_rate = 0.0f;
	
	upiter = upll->tail;
	while(upiter)
	{
		up = pBm->data[upiter->val1].box;
		if(up.size.w > criteria)
		{
			upiter = upiter -> left;
			continue;
		}

		dwiter = dwll->head;
		while(dwiter)
		{
			dw = pBm->data[dwiter->val1].box;
			if(dw.size.w > criteria)
			{
				dwiter = dwiter -> right;
				continue;
			}

			if((ovl = up.overlapw(dw)) <= 0)
			{
				dwiter = dwiter -> right;
				continue;
			}

			//concerned with width
			min_l = (up.orig.x < dw.orig.x) ? up.orig.x : dw.orig.x;
			max_r = (up.orig.x + up.size.w > dw.orig.x + dw.size.w) ?
					(up.orig.x + up.size.w) : (dw.orig.x + dw.size.w);
			w = max_r - min_l;
			rate_w = (float)ovl/(float)w;

			//concerned with height
			itv = up.intervalh(dw);
			rate_h = (float)h/(float)itv;

			if((rate = rate_w * rate_h) > max_rate)
			{
				max_rate = rate;
				retv->write(upiter->val1, dwiter->val1);
			}

			dwiter = dwiter -> right;
		}
		upiter = upiter -> left;
	}

	if(max_rate < LPR_TM_SELECT_BOUND) return -1;
	return 0;
}


int TupleManager::check(BlobManager* pBm, ClusterManager* pCm, TUPLE tuple, int a, int b)
{
	int i;
	RECT up, dw, tgt;
	DNODE* iter;


	up = pBm->data[tuple.upper].box;
	dw = pBm->data[tuple.below].box;

	for(i = a+1; i <= b-1; i++)
	{
		iter = pCm->cluster[i].head;

		while(iter)
		{
			tgt = pBm->data[iter->val1].box;

			if(up.overlapw(tgt) > 0)
				return -1;

			if(dw.overlapw(tgt) > 0)
				return -1;

			iter = iter -> right;
		}
	}

	return 0;
}

void TupleManager::bestfit(AREA size, uchar* vedge, BlobManager* pBm, RECT* pos)
{
	int i;

	int max_freq_val = 0;
	
	RECT best_fit_pos;
	RECT up,dw, temp;

	for(i = 0; i < this->nTuple ; i++)
	{
		up = pBm->data[this->tuple[i].upper].box;	
		dw = pBm->data[this->tuple[i].below].box;

		temp.orig.x = (up.orig.x < dw.orig.x) ? up.orig.x : dw.orig.x;
		temp.orig.y = up.orig.y + up.size.h;
		temp.size.h = dw.orig.y - (temp.orig.y) + 1;
		temp.size.w = ((up.orig.x + up.size.w) > (dw.orig.x + dw.size.w)) ? 
			(up.orig.x + up.size.w - temp.orig.x) : (dw.orig.x + dw.size.w - temp.orig.x);

		this->hist.process(size, vedge, temp);
		
		if(this->hist.freq > max_freq_val)
		{
			max_freq_val = this->hist.freq;
			best_fit_pos = temp;
		}
	}

	*pos = best_fit_pos;
}


int TupleManager::process(AREA size, uchar* vEdge, BlobManager* pBm, ClusterManager* pCm, RECT* pos)
{
	int a, b, itv;
	int criteria1, criteria2;
	DNODE *upper, *below;
	TUPLE _tuple;
	
	criteria1 = (int)((float)_global_FrmOrigH * LPR_TM_PROC_SCALE1);
	criteria2 = (int)((float)_global_FrmOrigH * LPR_TM_PROC_SCALE2);

	this->init();
	
	for(a = 0; a < pCm->nCluster-1; a++)
	{
		upper = pCm->cluster[a].tail;		
		
		for(b = a + 1; b < pCm->nCluster; b++)
		{
			below = pCm->cluster[b].head;

			//[step1]
			//choose two clusters, Both cluster have appropriate horizontal interval..
			itv = pBm->data[upper->val1].box.intervalh(pBm->data[below->val1].box);
			if(itv >= criteria1 || itv <= criteria2)
				continue;

			//[step2]
			//select edge tuple which is likely to be strong horizontal edge of lp.
			if(this->select(size, pBm, &(pCm->cluster[a]), &(pCm->cluster[b]), itv, &_tuple) < 0)
				continue;	
			
			//[step3]
			//In between two strong horizontal lines, There should be no any horizontal line.
			if(this->check(pBm, pCm,  _tuple, a, b) < 0)
				continue;
			
			if(this->nTuple >= this->capacity)
			{
				fprintf(stdout, "[notice] TupleManager(process) : nTuple full capacity.\n");
				return -1;
			}
			this->tuple[this->nTuple++] = _tuple;
		}
	}

	if(!(this->nTuple))
	{
		fprintf(stdout, "[notice] TupleManager(process) : no any tuples.\n");
		return -1;
	}

	this->bestfit(size, vEdge, pBm, pos);

	return 0;
}

void Analyst::makeDHist(AREA size, uchar* hedge, RECT pos)
{
	int r, c;

	//buff1 for creating horizontal edge histogram.
	//create histogram
	for(r = pos.orig.y; r < pos.orig.y + pos.size.h; r++)
	{
		for(c = 0; c < pos.size.w; c++)
		{
			this->buff1[c] += hedge[(pos.orig.x + c) + r * size.w];
		}
	}
	
	//in fact, hist pointer saves derivative of hedge histogram.
	for(this->hist[0] = 0, c = 1; c < this->size; c++)
	{
		this->hist[c] = this->buff1[c] - this->buff1[c-1];
	}

	//and erase noise of hist.
	this->maf.process(this->hist, this->size);

#ifdef ACTIVATE_OPENCV
	int		max_v;
	int		scale_v;
	const int	h_img = 50;

	imgHist = cv::Mat(h_img, pos.size.w, CV_8UC3);
	memset(imgHist.data, 0x00, 3 * sizeof(uchar) * imgHist.cols * imgHist.rows);
	for(max_v = 0, c = 0; c < pos.size.w; c++)
	{
		if(max_v >= this->buff1[c]) continue;
		max_v = this->buff1[c];
	}

	for(c = 0; c < pos.size.w; c++)
	{
		scale_v = (int)((float)this->buff1[c] / (float)max_v * (float)h_img);
		cv::line(
				imgHist, 
				cv::Point(c, imgHist.rows),
				cv::Point(c, imgHist.rows - scale_v),
				cv::Scalar(0xFF, 0xFF, 0xFF), 1
			);
	}
#endif
}

int Analyst::catchExtV(RECT pos, int* retv1, int* retv2)
{
	int c, i;
	int before, curr;
	int criteria, dist;
	int bundle, cnt;
	int max_hist_val, idx_max_hist;
	const int Range2CalcDist = 1;
	NODE* iter;

	//buff2 for vote table.
	for(curr = -1, c = 0; c < this->size; c++)
	{
		this->evc.buff.write(hist[c]);

		//when evc buffer is completely filled, execute below code.
		if(c < this->evc.buff.capacity - 1)
			continue;

		//when meeting extreme value of histogram, execute below code.
		if(this->evc.process() < 0)
			continue;

		before = curr;
		curr = c;

		//insert first domain val of extreme val into linked list.
		if(before < 0)
		{
			this->ll.insert(curr);
			continue;
		}
	
		//insert domain val into linked list.
		if((dist = curr - before) > 1)
		{
			this->ll.insert(curr);

			//vote.
			for(i = -Range2CalcDist; i <= Range2CalcDist; i++)
			{
				if(dist + i < 0 || dist + i >=pos.size.w)
					continue;
				buff2[dist+i]++;
			}
		}
	}

	//if the minimum condition is met
	//remove adjacent nodes and some noise.
	if(this->ll.nNode >= 7)
	{
		//investigate vote table to know distance between nodes.
		//idx_max_hist equals to distance between nodes.
		max_hist_val = 0;
		idx_max_hist = 0;
		for(c = 0; c < this->size; c++)
		{
			if(max_hist_val < buff2[c])
			{
				max_hist_val = buff2[c];
				idx_max_hist = c;
			}
		}
		*retv1 = idx_max_hist;

		//first, remove adjacent nodes.
		criteria = (int)((float)idx_max_hist * LPR_AN_CATCH_SCALE1);	
		curr = this->ll.head->val;
		iter = this->ll.head->next;
		while(iter)
		{
			before = curr;
			curr = iter->val;

			if(curr - before < criteria)
				this->ll.remove(curr);

			iter = iter->next;
		}

		//secondly, remove noise.
		cnt  = 1;
		bundle = 1;
		criteria = (int)((float)idx_max_hist * LPR_AN_CATCH_SCALE2);
		curr = this->ll.head->val;
		iter = this->ll.head->next;
		while(iter)
		{
			before = curr;
			curr = iter->val;

			if((curr - before - idx_max_hist) > criteria)
			{
				//iter points tail node.
				if(iter->next == NULL)
				{
					this->ll.remove(curr);
				}	
				else
				{
					if(cnt > 1)
						bundle++;
					else
						this->ll.remove(before);

					cnt = 1;
				}
			}
			else
			{
				cnt++;
			}
	
			iter = iter->next;
		}
		*retv2 = bundle;

#ifdef ACTIVATE_OPENCV
	iter = this->ll.head;
	while(iter)
	{
		cv::line(
				imgHist,
				cv::Point(iter->val, 0),
				cv::Point(iter->val, imgHist.rows),
				cv::Scalar(0x00, 0xFF, 0x00), 1

			);
		iter = iter->next;
	}
	cv::imshow("histogram", imgHist);
	cv::moveWindow("histogram", 700, 300);
#endif

		return 0;
	}
	else
	{
		fprintf(stdout, "[notice] Analyst(catchExtV) : not enough node %d\n", this->ll.nNode);
		*retv1 = 0;
		*retv2 = 0;

		return -1;
	}

}


int Analyst::process(AREA size, uchar* hedge, RECT pos, int* retv1, int* retv2)
{
	this->init(pos.size.w);

	this->makeDHist(size, hedge, pos);

	if(this->catchExtV(pos, retv1, retv2) < 0)
		return -1;
	
	return 0;
}

void NoiseEraser::SimpleErase(AREA size, BlobManager* pBm)
{
	int i;
	int width, height;

	for(i = 0; i < pBm->nBlob; i++)
	{
		width = pBm->data[i].box.size.w;
		height = pBm->data[i].box.size.h;

		if(height < width)
		{
			pBm->data[i].val = -1;
			continue;
		}

		//blob which has big width will be major element when calculating average width.
		if(width > size.w/7)
		{
			pBm->data[i].val = -1;
			continue;
		}

		//blob which has very small height will be major element when calculating average heihgt.
		if(height < size.h/3)
		{
			pBm->data[i].val = -1;
			continue;
		}
	}
}

int NoiseEraser::DetailErase(AREA size, BlobManager* pBm)
{
	NODE* iter;
	int i, k;
	int height, base;
	int max_vote, idx_vote;
	const int range_vote = 2;

	for(i = this->mem_min_h; i <= this->mem_max_h; i++)
		this->ll[i].init();

	this->mem_max_h = 0;
	this->mem_min_h = lcapacity - 1;
	idx_vote = 0;
	max_vote = 0;
	for(i = 0; i < pBm->nBlob; i++)
	{
		if(pBm->data[i].val < 0) continue;

		height = pBm->data[i].box.size.h;

		for(k = height - range_vote ; k <= height + range_vote; k++)
		{
			if(k < 0 || k >= lcapacity)
				continue;

			this->ll[k].insert(i);						
			if(max_vote < this->ll[k].nNode)
			{
				max_vote = this->ll[k].nNode;
				idx_vote = k;
			}

			if(this->mem_max_h < k) this->mem_max_h = k;
			if(this->mem_min_h > k) this->mem_min_h = k;
		}
	}

	this->mark = (uchar*)malloc(sizeof(uchar) * pBm->nBlob);
	memset(this->mark, 0x00, sizeof(uchar) * pBm->nBlob);

	iter = ll[idx_vote].head;
	while(iter)
	{
		mark[iter->val] = 1;
		iter = iter->next;
	}
	
	for(i = this->mem_min_h; i <= this->mem_max_h ; i++)
		this->ll[i].init();

	this->mem_max_h = 0;
	this->mem_min_h = lcapacity-1;
	max_vote = 0;
	idx_vote = 0;
	for(max_vote = 0, idx_vote = 0, i = 0; i < pBm->nBlob; i++)
	{
		if(pBm->data[i].val < 0) continue;

		base = pBm->data[i].box.orig.y;

		for(k = base - range_vote ; k <= base + range_vote; k++)
		{
			if(k < 0 || k >= lcapacity)
				continue;

			this->ll[k].insert(i);						

			if(max_vote < this->ll[k].nNode)
			{
				max_vote = this->ll[k].nNode;
				idx_vote = k;
			}

			if(this->mem_max_h < k) this->mem_max_h = k;
			if(this->mem_min_h > k) this->mem_min_h = k;
		}
	}

	iter = ll[idx_vote].head;
	while(iter)
	{
		//mark for implementing AND operation.
		if(!mark[iter->val])
		{
			iter = iter->next;
			continue;
		}
		
		if(this->nToken >= this->tcapacity)
		{
			free(this->mark);
			fprintf(stdout, "[notice] NoiseEraser(DetailErase) : nToken full capacity.\n");
			return -1;
		}
		
		this->token[this->nToken++] = pBm->data[iter->val].box;
		iter = iter->next;
	}

	free(this->mark);

	if(this->nToken < 6)
	{
		fprintf(stdout, "[notice] NoiseEraser(DetailErase) : no any token.\n");
		return -1;
	}

	return 0;
}


void NoiseEraser::sort()
{
	int i, j, idx;
	RECT temp;

	for(i = 0; i < nToken-1; i++)
	{
		idx = i;
		for(j = i + 1; j < nToken; j++)
		{
			if(this->token[idx].orig.x > this->token[j].orig.x)
				idx = j;
		}
		
		temp = this->token[idx];
		this->token[idx] = this->token[i];
		this->token[i] = temp;
	}
}


int NoiseEraser::process(AREA size, BlobManager* pBm)
{
	this->init();

	this->SimpleErase(size, pBm);

	if(this->DetailErase(size, pBm) < 0) return -1;

	this->sort();

	return 0;
}


int Extractor::type1(AREA size, uchar* img, NoiseEraser* pNe, Analyst* pAn, RECT pos)
{
	int i, k, r, c;
	int bound_c, bound_r;
	int ncase, sum, max, start;
	NODE* iter;

	//[step1] make pixel-value-summation table
	this->pcnt = pNe->nToken;
	for(i = 0; i < this->pcnt; i++)
	{
		bound_r = pNe->token[i].orig.y + pNe->token[i].size.h;
		bound_c = pNe->token[i].orig.x + pNe->token[i].size.w;

		for(r = pNe->token[i].orig.y; r < bound_r; r++)
		{
			for(c = pNe->token[i].orig.x; c < bound_c; c++)
			{
				this->pixel[i] += (int)(0xFF - img[c + r * size.w]);
			}
		}
	}

	//[step2] make matching information.
	this->mcnt = pAn->ll.nNode;
	iter = pAn->ll.head;
	for(start = 0, i = 0; i < this->mcnt; i++)
	{
		for(k = start; k < this->pcnt; k++)
		{
			if(
				iter->val >= pNe->token[k].orig.x
				&& iter->val <= pNe->token[k].orig.x + pNe->token[k].size.w
				)
			{
				this->match[i] = k;
				start = k+1;
				break;
			}
		}
		iter = iter->next;
	}

	//[step3] find starting token which has maxima summation value.
	ncase = pAn->ll.nNode - 7;
	for(max = 0, start = 0, i = 0; i <= ncase; i++)
	{
		sum = 0;

		for(k = i; k < i+7; k++)
		{
			if(this->match[k] < 0) continue;
			sum += this->pixel[this->match[k]];
		}

		if(max < sum)
		{
			max = sum;
			start = i;
		}
	}

	//[step4] determine final token position.
	for(i = 0; i < 2; i++) //token 0, 1
	{
		if(this->match[start + i] < 0)
		{
			fprintf(stdout, "[notice] Extractor(type1) : Extraction %d Failure.\n", i);
			return -1;
		}

		this->token[i] = pNe->token[match[start + i]];
		this->token[i].orig.x += pos.orig.x;
		this->token[i].orig.y += pos.orig.y;
	}

	for(i = 3; i < 7; i++) //token 3, 4, 5, 6
	{
		if(this->match[start + i] < 0)
		{
			fprintf(stdout, "[notice] Extractor(type1) : Extraction %d Failure.\n", i);
			return -1;
		}
		
		this->token[i] = pNe->token[match[start+i]];
		this->token[i].orig.x += pos.orig.x;
		this->token[i].orig.y += pos.orig.y;
	}

	//token 2
	this->token[2].orig.x = this->token[1].orig.x + this->token[1].size.w;
	this->token[2].size.w = this->token[3].orig.x - this->token[2].orig.x;
	this->token[2].orig.y = (this->token[1].orig.y + this->token[3].orig.y) >> 1;
	this->token[2].size.h = (this->token[1].size.h + this->token[3].size.h) >> 1;

	return 0;
}

int Extractor::type2(NoiseEraser* pNe, Analyst* pAn, RECT pos, int distance)
{
	int i, k, start;
	int tpoint, before, curr;
	int criteria;
	NODE* iter;
	
	//[step1] make matching information.
	this->mcnt = pAn->ll.nNode;
	this->pcnt = pNe->nToken;
	iter = pAn->ll.head;
	start = 0;

	for(i = 0; i < this->mcnt; i++)
	{
		for(k = start; k < this->pcnt; k++)
		{
			if(
				iter->val >= pNe->token[k].orig.x
				&& iter->val <= pNe->token[k].orig.x + pNe->token[k].size.w
				)
			{
				this->match[i] = k;
				start = k+1;
				break;
			}
		}
		iter = iter->next;
	}


	//[step2] Based on distnace, Find turning point which set apart each bundles.
	tpoint = 0;
	curr = pAn->ll.head->val;
	iter = pAn->ll.head->next;
	criteria = (int)((float)distance * LPR_EX_TYPE2_SCALE1);
	while(iter)
	{
		before = curr;
		curr = iter->val;
		
		if((curr - before - distance) > criteria)
			break;

		tpoint++;
		iter = iter->next;
	}

	if(tpoint < 2)
	{
		fprintf(stdout, "[notice] Extractor(type2) : tpoint error. \n");
		return -1;
	}

	//[step3] determine final token position.
	for(i = 0; i < 2; i++) //token 0, 1
	{
		if(this->match[tpoint-2+i] < 0)
		{
			fprintf(stdout, "[notice] Extractor(type2) : Extraction %d Failure.\n", i);
			return -1;
		}

		this->token[i] = pNe->token[match[tpoint-2+i]];
		this->token[i].orig.x += pos.orig.x;
		this->token[i].orig.y += pos.orig.y;
	}

	for(i = 3; i < 7; i++) //token 3, 4, 5, 6
	{
		if(this->match[tpoint-2+i] < 0)
		{
			fprintf(stdout, "[notice] Extractor(type2) : Extraction %d Failure.\n", i);
			return -1;
		}
		
		this->token[i] = pNe->token[match[tpoint-2+i]];
		this->token[i].orig.x += pos.orig.x;
		this->token[i].orig.y += pos.orig.y;
	}

	//token 2
	this->token[2].orig.x = this->token[1].orig.x + this->token[1].size.w;
	this->token[2].size.w = (int)((float)(this->token[3].orig.x - this->token[2].orig.x) * LPR_EX_TYPE2_SCALE2);
	this->token[2].orig.y = (this->token[1].orig.y < this->token[3].orig.y) ? (this->token[1].orig.y) : (this->token[3].orig.y);
	this->token[2].size.h = (this->token[1].orig.y + this->token[1].size.h > this->token[3].orig.y + this->token[3].size.h) ?
					(this->token[1].orig.y + this->token[1].size.h - this->token[2].orig.y) : (this->token[3].orig.y + this->token[3].size.h - this->token[2].orig.y);

	return 0;
}

void Extractor::extraction(AREA size, uchar* vedge, int* param, RECT* pos, RECT* pos_tk)
{
	int r, c;
	int x1, x2;
	int parameter = 0;
	const int padding = 2;

	x1 = this->token[0].orig.x - padding;
	if(x1 < 0) x1 = 0;

	x2 = this->token[6].orig.x + this->token[6].size.w + padding;
	if(x2 >= size.w) x2 = size.w - 1;

	pos->orig.x = x1;
	pos->size.w = x2 - x1; 

	memcpy(pos_tk, this->token, sizeof(RECT) * 7);

	for(r = pos->orig.y ; r < pos->orig.y + pos->size.h; r++)
	{
		for(c = pos->orig.x ; c < pos->orig.x + pos->size.w; c++)
			parameter += vedge[c + r * size.w];
	}

	*param = parameter;
}

int Extractor::process(AREA size, uchar *img, uchar *vedge, NoiseEraser* pNe, Analyst* pAn, int bundle, int distance, int* param, RECT* pos, RECT* pos_tk)
{
	this->init();

	switch(bundle)
	{
	case 1:
		if(this->type1(size, img, pNe, pAn, *pos) < 0)
			return -1;
		break;

	case 2:
		if(this->type2(pNe, pAn, *pos, distance) < 0)
			return -1;
		break;

	default:
		fprintf(stdout, "[notice] Extractor(process) : bundle exception occur.\n");
		return -1;
	}

	this->extraction(size, vedge, param, pos, pos_tk);
	return 0;
}

int LPRManager::determine_lppos_yaxis(AREA size, uchar* hedge, uchar* vedge)
{
	imgproc_haar(size, hedge, &this->vm);

#ifdef ACTIVATE_OPENCV
	imgHaar = cv::Mat(size.h, size.w, CV_8UC1);
	memset(imgHaar.data, 0x00, sizeof(uchar) * imgHaar.cols * imgHaar.rows);
	for(int i = 0; i < this->vm.size; i++)
	{
		imgHaar.data[this->vm.data[i].x + this->vm.data[i].y * imgHaar.cols] = 0xFF;
	}
	cv::imshow("haar-horizon", imgHaar);
	cv::moveWindow	("haar-horizon", 370,0);
#endif

	if(bm.process(size, &this->vm) < 2)
	{
		fprintf(stdout, "[notice] LPRManager : determine_llpos_yaxis - not enough blobs \n");
		return -1;
	}

	if(cm.process(size, &this->bm) < 0)
		return -1;

	if(tm.process(size, vedge, &this->bm, &this->cm, &this->pos) < 0)
		return -1;

#ifdef ACTIVATE_OPENCV
	cv::cvtColor(imgSobel, imgSobel, CV_GRAY2BGR);
	for(int i = 0; i < this->tm.nTuple; i++)
	{
		RECT up = this->bm.data[tm.tuple[i].upper].box;
		RECT down = this->bm.data[tm.tuple[i].below].box;

		cv::line(
				imgSobel,
				cv::Point(up.orig.x, up.orig.y + up.size.h) ,
				cv::Point(up.orig.x + up.size.w, up.orig.y + up.size.h),
				cv::Scalar(0x00, 0xFF, 0xFF),1
			);

		cv::line(
				imgSobel,
				cv::Point(down.orig.x, down.orig.y) ,
				cv::Point(down.orig.x + down.size.w, down.orig.y),
				cv::Scalar(0x00, 0xFF, 0xFF),1
			);
	}
	cv::imshow("sobel-vertical-tuple", imgSobel);
	cv::moveWindow("sobel-vertical-tuple",370,230);
#endif

	return 0;
}

int LPRManager::determine_lppos_xaxis(AREA size, uchar* img, uchar* hedge, uchar* vedge, uchar* buff8, int* buff32)
{
	int distance, bundle;

	if(an.process(size, hedge, this->pos, &distance, &bundle) < 0)
		return -1;

	imgproc_binarize(size, img, &this->vm, this->pos, buff8, buff32);

#ifdef ACTIVATE_OPENCV
	imgLpB = cv::Mat(this->pos.size.h, this->pos.size.w, CV_8UC1);
	memset(imgLpB.data, 0x00, sizeof(uchar) * imgLpB.cols * imgLpB.rows);
	for(int i = 0; i < this->vm.size; i++)
	{
		imgLpB.data[this->vm.data[i].x + this->vm.data[i].y * imgLpB.cols] = 0xFF;
	}
	cv::imshow("lp-binary", imgLpB);
	cv::moveWindow("lp-binary", 700, 100);
#endif

	if(bm.process(this->pos.size, &this->vm) < 7)
	{
		fprintf(stdout, "[notice] LPRManager : determine_llpos_xaxis -  not enough blobs\n");
		return -1;
	}

#ifdef ACTIVATE_OPENCV
	imgLpN = cv::Mat(this->pos.size.h, this->pos.size.w, CV_8UC1);
	memcpy(imgLpN.data, imgLp.data, sizeof(uchar) * imgLp.cols * imgLp.rows);
	cv::cvtColor(imgLpN, imgLpN, CV_GRAY2BGR);
	for(int i = 0; i < this->bm.nBlob; i++)
	{
		int x1 = this->bm.data[i].box.orig.x;
		int y1 = this->bm.data[i].box.orig.y;
		int x2 = x1 + this->bm.data[i].box.size.w;
		int y2 = y1 + this->bm.data[i].box.size.h;

		cv::rectangle(
				imgLpN,
				cv::Point(x1, y1),
				cv::Point(x2, y2),
				cv::Scalar(0xFF, 0x00, 0xFF), 1
			     );
	}
	cv::imshow("lp-before-noise", imgLpN);
	cv::moveWindow("lp-before-noise", 700, 170);
#endif


	if(ne.process(this->pos.size, &this->bm) < 0)
		return -1;

#ifdef ACTIVATE_OPENCV
	imgLpN = cv::Mat(this->pos.size.h, this->pos.size.w, CV_8UC1);
	memcpy(imgLpN.data, imgLp.data, sizeof(uchar) * imgLp.cols * imgLp.rows);
	cv::cvtColor(imgLpN, imgLpN, CV_GRAY2BGR);

	for(int i = 0; i < this->ne.nToken; i++)
	{
		int x1 = this->ne.token[i].orig.x;
		int y1 = this->ne.token[i].orig.y;
		int x2 = this->ne.token[i].orig.x + this->ne.token[i].size.w;
		int y2 = this->ne.token[i].orig.y + this->ne.token[i].size.h;
		cv::rectangle(
			imgLpN,
			cv::Point(x1, y1),
			cv::Point(x2, y2),
			cv::Scalar(0xFF, 0x00, 0xFF), 1
			);

	}
	cv::imshow("lp-before-noisef", imgLpN);
	cv::moveWindow("lp-before-noisef",700, 230);
#endif

	if(ex.process(size, img, vedge, &this->ne, &this->an, bundle, distance, &this->param, &this->pos, this->pos_tk) < 0)
		return -1;

	return 0;
}

void LPRManager::move(IMAGE* img, IMAGE* lp, IMAGE* tk, RECT box, RECT* tks)
{
	int i, r;

	//write license plate to ui buffer.
	memset(lp->data, 0x00, sizeof(uchar) * lp->size.w * lp->size.h);
	for(r = 0; r < box.size.h; r++)
	{
		memcpy(
			&lp->data[r*lp->size.w],
			&img->data[(box.orig.x) + (box.orig.y + r) * img->size.w],
			sizeof(uchar) * box.size.w
		      );
	}

	//write tokens to ui buffer.
	memset(tk->data, 0x00, sizeof(uchar) * tk->size.w * tk->size.h);
	for(i = 0; i < 7; i++)
	{
		if(imgproc_resize(img, tk, this->ex.token[i], i) < 0)
			break;;
	}
}

int LPRManager::recognize(IMAGE* pImage, IMAGE* pLp, IMAGE* pTokens)
{

	this->init();

	imgproc_sobel(pImage->size, pImage->data, _8bit_buff_1, _8bit_buff_2);	

#ifdef ACTIVATE_OPENCV
	imgSobel = cv::Mat(pImage->size.h, pImage->size.w, CV_8UC1);
	memcpy(imgSobel.data, _8bit_buff_1, sizeof(uchar) * imgSobel.cols * imgSobel.rows);
	cv::imshow("sobel-horizon", imgSobel);
	cv::moveWindow("sobel-horizon",0,0);

	memcpy(imgSobel.data, _8bit_buff_2, sizeof(uchar) * imgSobel.cols * imgSobel.rows);
	cv::imshow("sobel-vertical", imgSobel);
	cv::moveWindow("sobel-vertical",0,230);
#endif

	if(this->determine_lppos_yaxis(pImage->size, _8bit_buff_1, _8bit_buff_2) < 0)
		return -1;

#ifdef ACTIVATE_OPENCV
	imgLp = cv::Mat(this->pos.size.h, this->pos.size.w, CV_8UC1);
	for(int r = 0; r < imgLp.rows; r++)
	{
		memcpy(
			&imgLp.data[r * imgLp.cols],
			&pImage->data[(this->pos.orig.x) + (this->pos.orig.y + r) * pImage->size.w],
			sizeof(uchar) * this->pos.size.w
		      );
	}
	cv::imshow("lp-before", imgLp);
	cv::moveWindow("lp-before",700, 0);
	//cv::waitKey();
#endif

	if(this->determine_lppos_xaxis(pImage->size, pImage->data, _8bit_buff_1, _8bit_buff_2, _8bit_buff_3, _32bit_buff) < 0)
		return -1;

#ifdef ACTIVATE_OPENCV
	imgLp = cv::Mat(this->pos.size.h, this->pos.size.w, CV_8UC1);
	for(int r = 0; r < imgLp.rows; r++)
	{
		memcpy(
			&imgLp.data[r * imgLp.cols],
			&pImage->data[(this->pos.orig.x) + (this->pos.orig.y + r) * pImage->size.w],
			sizeof(uchar) * this->pos.size.w
		      );
	}
	cv::imshow("lp-after", imgLp);
	cv::moveWindow("lp-after", 700, 400);
#endif
	
	this->move(pImage, pLp, pTokens, this->pos, this->pos_tk);

#ifdef ACTIVATE_OPENCV
	buffLp = cv::Mat(pLp->size.h, pLp->size.w, CV_8UC1);
	memcpy(buffLp.data, pLp->data, sizeof(uchar) * buffLp.cols * buffLp.rows); 
	cv::imshow("final1", buffLp);
	cv::moveWindow("final1", 800,0);

	buffTk = cv::Mat(pTokens->size.h, pTokens->size.w, CV_8UC1);
	memcpy(buffTk.data, pTokens->data, sizeof(uchar) * buffTk.cols * buffTk.rows);
	cv::imshow("final2", buffTk);
	cv::moveWindow("final2", 800,100);
#endif
	return 0;
}

int LPRManager::_Tracker_recognize(IMAGE* pImage, IMAGE* pLp, IMAGE* pTokens, ObjTracker* pTracker)
{
	const float wratio = LPR_TRACKER_PADDSCALE_W;
	const float hratio = LPR_TRACKER_PADDSCALE_H;
	int param_ref, param_eval;
	int param_diff, param_criteria;
	int dx, dy, dh, dw;
	RECT measurement;
	RECT prediction;
	RECT TrackingRegion;

	prediction = pTracker->read_box();

	//step1 : make Tracking Region
	dx = (dw = (int)((float)prediction.size.w * wratio)) >> 1;
	dy = (dh = (int)((float)prediction.size.h * hratio)) >> 1;
	TrackingRegion.orig.x = prediction.orig.x - dx;
	if(TrackingRegion.orig.x < 0)
		TrackingRegion.orig.x = 0;
	TrackingRegion.orig.y = prediction.orig.y - dy;
	if(TrackingRegion.orig.y < 0)
		TrackingRegion.orig.y = 0;
	TrackingRegion.size.w = prediction.size.w + dw;
	if(TrackingRegion.orig.x + TrackingRegion.size.w > pImage->size.w)
		TrackingRegion.size.w = pImage->size.w - TrackingRegion.orig.x;
	TrackingRegion.size.h = prediction.size.h + dh;
	if(TrackingRegion.orig.y + TrackingRegion.size.h > pImage->size.h)
		TrackingRegion.size.h = pImage->size.h - TrackingRegion.orig.y;

	//step2 : get sobel data from prediction box.
	param_ref = pTracker->read_param();
	param_eval = imgproc_sobel_local(pImage->size, pImage->data, _8bit_buff_1, TrackingRegion, prediction);
	param_diff = (param_ref > param_eval) ? (param_ref - param_eval) : (param_eval - param_ref);
	param_criteria = (int)((float)param_ref * LPR_TRACKER_PARAM_SCALE);
	if(param_diff > param_criteria)
	{
		fprintf(stdout , "[notice] _Tracker_recognize : inappropriate param_eval %d/%d\n", param_eval, param_ref);
		return -1;
	}

	if(meanshift(TrackingRegion.size, _8bit_buff_1, prediction, &measurement) < 0)
		return -1;

	pTracker->write_box(measurement);

	this->pos = measurement;
	
	//******************************************************************************//
	//write license plate to ui buffer.
	memset(pLp->data, 0x00, sizeof(uchar) * pLp->size.w * pLp->size.h);
	for(int r = 0; r < measurement.size.h; r++)
	{
		memcpy(
			&pLp->data[r*pLp->size.w],
			&pImage->data[(measurement.orig.x) + (measurement.orig.y + r) * pImage->size.w],
			sizeof(uchar) * measurement.size.w
		      );
	}

	memset(pTokens->data, 0x00, sizeof(uchar) * pTokens->size.w * pTokens->size.h);
	//****************************************************************************//
	
	return 0;
}
