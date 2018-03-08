#ifndef BLOB_H_INCLUDED
#define BLOB_H_INCLUDED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "timdebug.hpp"
#include "datatype.hpp"
#include "datastruct.hpp"

#ifdef ACTIVATE_OPENCV
#include "opencv/cv.hpp"
#endif

#define	BLOB_DATA_MAXSIZE	200


struct BLOB
{
	RECT		box;
	int		val;

	void write(RECT bx, int v)
	{
		box = bx;
		val = v;
	}
};


class BlobManager
{
private:
	bool		flag;
	int		capacity;
	
	HashTable	ht;
	DisjointSet	djt;
	
	int make_label_img(AREA, VpixelManager*);
	void derive_blob_info(AREA, VpixelManager*);
public:
	int		nBlob;
	BLOB		*data;

	BlobManager() : flag(false), data(NULL){}

	void allocate(AREA buffsz, int* buff, int cap)
	{
		int max_hash_entry;

		if(flag)
		{
			fprintf(stderr, "[error] BlobManager(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = cap;
		data = (BLOB*)malloc(sizeof(BLOB) * capacity);
		djt.regist(buffsz.w * buffsz.h, buff);
		max_hash_entry = (3 * capacity) >> 1;
		ht.allocate(max_hash_entry);
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
		nBlob = 0;
		djt.clear();
		ht.clear();
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] BlobManager(init) : not allocated yet.\n");
			exit(-1);
		}

		nBlob = 0;
		djt.init();
	}

	int process(AREA, VpixelManager*);
};


#endif // BLOB_H_INCLUDED
