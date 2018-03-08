#ifndef DATASTRUCT_H_INCLUDED
#define DATASTRUCT_H_INCLUDED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datatype.hpp"

struct NODE
{
	int val;
	NODE * next;

	NODE(int v) : val(v), next(NULL) {}
};

struct LinkedList
{
	int	nNode;
	NODE	*head;

	LinkedList() : nNode(0), head(NULL) {}

	void init()
	{
		NODE* temp;

		while(head)
		{
			temp = head;
			head = head->next;
			delete(temp);
		}

		nNode = 0;
		head = NULL;
	}

	void clear()
	{
		this->init();
	}

	void insert(int val)
	{
		NODE	*iter;
		NODE	*newbie = new NODE(val);

		if(!head)
		{
			head = newbie;
		}
		else
		{
			iter = head;
			while(iter->next) iter = iter -> next;
			iter->next = newbie;
		}
		nNode++;
	}

	void remove(int val)
	{
		NODE	*follower = NULL;
		NODE	*iter = head;
		
		while(iter)
		{
			if(iter->val == val)
			{
				if(iter == head)
				{
					head = head -> next;
				}
				else
				{
					follower -> next = iter -> next;
				}

				nNode--;
				delete(iter);
				break;
			}

			follower = iter;
			iter = iter->next;
		}
	}
};

struct DNODE
{
	int	val1;
	int	val2;
	DNODE	*left;
	DNODE	*right;

	DNODE(int v) : val1(v), val2(0), left(NULL), right(NULL) {}
	DNODE(int v1, int v2) : val1(v1), val2(v2), left(NULL), right(NULL){}
};

struct DBLinkedList
{
	int	nNode;
	DNODE	*head;
	DNODE	*tail;

	DBLinkedList() : nNode(0), head(NULL), tail(NULL) {}

	void init()
	{
		DNODE *temp;

		while(head)
		{
			temp = head;
			head = head->right;
			delete(temp);
		}

		nNode = 0;
		head = NULL;
		tail = NULL;
	}

	void clear()
	{
		this->init();
	}

	void insert(int val)
	{
		DNODE *newbie = new DNODE(val);

		if(!head)
		{
			head = newbie;
			tail = newbie;
		}
		else
		{
			newbie->left = tail;
			tail->right = newbie;
			tail = newbie;
		}
		nNode++;
	}

	//insert with sorting.
	//val2 is parameter of ascending sort.
	void insert2(int val1, int val2)
	{
		DNODE	*iter;
		DNODE	*follower;
		DNODE	*newbie = new DNODE(val1, val2);
		
		//empty linked list.
		if(head == NULL)
		{
			head = newbie;
			tail = newbie;
		}

		//non-empty linked list.
		else
		{
			follower = NULL;
			iter = head;

			while(iter)
			{
				//when newbie's param is smaller than iterator.
				//insert a newbie on iterator's left side.
				if(newbie->val2 < iter->val2)
				{
					if(iter == head)
					{
						newbie->right = iter;
						iter->left = newbie;
						head = newbie;
					}
					else
					{
						newbie->right = iter;
						newbie->left = follower;

						iter->left = newbie;
						follower->right = newbie;
					}
					
					nNode++;
					return;
				}

				follower = iter;
				iter = iter -> right;
			}
			
			//when break out the while loop, iterator points null.
			//which means that newbie has most big parameter.
			//so newbie will be inserted to tail node.
			newbie->left = tail;
			tail->right = newbie;
			tail = newbie;
		}

		nNode++;
		return;
	}

	void remove(int val)
	{
		DNODE	*iter = head;

		while(iter)
		{
			//when find target node.
			if(iter->val1 == val)
			{
				//when target is head node.
				if(iter == head)
				{
					//when dll only has one node.
					if(nNode == 1)
					{
						head = NULL;
						tail = NULL;
					}

					//when dll has more than one node.
					else
					{
						head = iter -> right;
						head -> left = NULL;
					}
				}

				//when target is not head node.
				//also which means that dll has more than one node.
				else
				{
					//when iter points tail node.
					if(iter == tail)
					{
						tail = iter->left;
						tail->right = NULL;
					}

					//when iter points middle node.
					else
					{
						iter->left->right = iter->right;
						iter->right->left = iter->left;
					}

				}
				nNode--;
				delete(iter);
				break;
			}

			iter = iter->right;
		}
	}
};

struct Param
{
	/*user defined method*/
	int	root;
	int	val;
	RECT	box;

	Param() {}
	Param(int r, int x, int y)
		: root(r), val(1)
	{
		box.write(x, y, 1, 1);
	}
};


struct HashNode
{
	Param		param;
	HashNode	*next;

	HashNode(Param p) :next(NULL)
	{
		param = p;
	}
};

class HashTable
{
private:
	bool		flag;
	int		capacity;

public:
	int		size;
	HashNode	**table;

	HashTable() : flag(false), table(NULL) {}

	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] HashTable(allocate) : already allocated.\n");
			exit(-1);
		}

		capacity = cap;
		table = (HashNode**)malloc(sizeof(HashNode*) * capacity);
		memset(table, 0x00, sizeof(HashNode*) * capacity);
		size = 0;
		flag = true;
	}

	void clear()
	{
		int i;
		HashNode* temp;

		if(table)
		{
			for(i = 0; i < capacity; i++)
			{
				while(table[i])
				{
					temp = table[i];
					table[i] = table[i] -> next;
					delete (temp);				
				}
			}
			free(table);
			table = NULL;
		}
		capacity = 0;
		size = 0;
		flag = false;
	}

	void init(int newsize)
	{
		int i;
		HashNode *temp;

		if(!flag)
		{
			fprintf(stderr, "[error] HashTable(init) : not allocated yet.\n");
			exit(-1);
		}
		
		for(i = 0; i < size; i++)
		{
			while(table[i])
			{
				temp = table[i];
				table[i] = table[i]->next;
				delete (temp);
			}
		}

		memset(table, 0x00, sizeof(HashNode*) * size);
		size = newsize;
	};

	/*user defined function*/
	//% => hash function.
	void insert(int root, int x, int y, HashNode** dptr)
	{
		int idx, mem;
		HashNode *iter, *follower;


		/* below branch will be executed when processing consecutive pixel*/
		if(!root)
		{
			iter = *dptr;
			if(iter->param.box.orig.x + iter->param.box.size.w < x)
				iter->param.box.size.w = x - iter->param.box.orig.x;
			iter->param.val++;
			return;
		}


		/* below code will be executed when entering new pixel token */
		//1. go to hash entry.
		idx = root % size;
		iter = table[idx];
		follower = NULL;

		//2. search specific hash node.
		while(iter)
		{
			if(iter->param.root == root) break;
			follower = iter;
			iter = iter -> next;
		}
		
		//3. When specific hash node exists.
		if(iter)
		{
			if(iter->param.box.orig.x > x)
			{
				mem = iter->param.box.orig.x + iter->param.box.size.w;
				iter->param.box.orig.x = x;
				iter->param.box.size.w = mem - iter->param.box.orig.x;
			}
			
			if(iter->param.box.orig.y + iter->param.box.size.h < y)
				iter->param.box.size.h = y - iter->param.box.orig.y;

			iter->param.val++;
		}

		//4. When specific hash node does not exist.
		else
		{	
			iter = new HashNode(Param(root, x, y));

			//When this entry has no hasn node.
			//follower is initated with NULL.
			if(!follower) table[idx] = iter;

			//follower is tail node.
			else follower->next = iter;
		}
		
		*dptr = iter;
	}

};

struct CircularBuff
{
	//ptr points cell which can be filled with val.
	bool	flag;
	int	capacity;
	int	ptr;
	int	*buff;

	CircularBuff(): flag(false), buff(NULL){}

	void allocate(int cap)
	{
		if(flag)
		{
			fprintf(stderr, "[error] CircularBuff(allocate) : already allocated.\n");
			exit(-1);
		}
		capacity = cap;
		buff = (int*)malloc(sizeof(int) * capacity);
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
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] CircularBuff(init) : not allocated yet.\n");
			exit(-1);
		}
		
		memset(buff, 0x00, sizeof(int) * capacity);
		ptr = 0;
	}

	void write(int val)
	{
		if(!flag)
		{
			fprintf(stderr, "[error] CircularBuff(write) : not allocated yet.\n");
			exit(-1);
		}

		buff[ptr] = val;
		if(++ptr >= capacity) ptr = 0;
	}

	int read(int* cpptr)
	{
		if(!flag)
		{
			fprintf(stderr, "[error] CircularBuff(read) : not allocated yet.\n");
			exit(-1);
		}

		if(--(*cpptr) < 0) *cpptr = capacity - 1;
		return buff[*cpptr];
	}
};

struct DisjointSet
{
	int flag;
	int capacity;
	int size;
	int *parent;

	DisjointSet() : flag(false), parent(NULL){}

	void regist(int cap, int* ptr)
	{
		if(flag)
		{
			fprintf(stderr, "[error] DisjointSet(regist) : already registered.\n");
			exit(-1);
		}
		
		capacity = cap;
		parent = ptr;
		flag = true;
	}

	void clear()
	{
		capacity = 0;
		size = 0;
		parent = NULL;
		flag = false;
	}

	void init()
	{
		if(!flag)
		{
			fprintf(stderr, "[error] DisjointSet(init) : not registered yet.\n");
			exit(-1);
		}

		//parent points shared memory.
		//which means that parent buff will be corrupted by another DisjointSet object.
		//in conclusion, always parent should be initiated entirely.
		memset(parent, 0xFF, sizeof(int) * capacity);
		size = 0;
	}

	void Union(int i, int j)
	{
		int count;
		
		if(!flag)
		{
			fprintf(stderr, "[error] DisjointSet(Union) : not registered yet.\n");
			exit(-1);
		}

		if(i >= capacity || j >= capacity)
		{
			fprintf(stderr, "[error] DisjointSet(Union) : parent idx out of range.\n");
			exit(-1);
		}

		count = parent[i] + parent[j];
		if (parent[i] > parent[j])
		{
			parent[i] = j;
			parent[j] = count;
		}
		else
		{
			parent[j] = i;
			parent[i] = count;
		}
	}

	int CollapsingFind(int i)
	{
		int r,s;
		
		if(!flag)
		{
			fprintf(stderr, "[error] DisjointSet(CollapsingFind) : not registered yet.\n");
			exit(-1);
		}

		if(i >= capacity)
		{
			fprintf(stderr, "[error] DisjointSet(CollapsingFind : parent idx out of range.\n");
			exit(-1);
		}

		//find root index (r) (root has minus content)
		for (r = i; parent[r] >= 0; r = parent[r]);
		if (i == r) return r;

		//collapsing
		while ((s = parent[i]) != r) { parent[i] = r; i = s; }
		return r;
	}

	int SimpleFind(int i)
	{
		if(!flag)
		{
			fprintf(stderr, "[error] DisjointSet(SimpleFind) : not registered yet.\n");
			exit(-1);
		}

		if(i >= capacity)
		{
			fprintf(stderr, "[error] DisjointSet(SimpleFind) : parent idx out of range.\n");
			exit(-1);
		}

		//find root index (r)
		while (parent[i] >= 0) i = parent[i];
		return i;
	}
};

#endif // DATASTRUCT_H_INCLUDED
