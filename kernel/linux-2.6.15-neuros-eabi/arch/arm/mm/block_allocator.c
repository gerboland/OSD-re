/******************************************************************************
block_alloc.c-> High performance block memory allocator for io and drivers.

12/23/2004 Nathan J. Crawford.

These routines allocate fixed-size blocks of memory for IO and drivers.  This 
version will return NULL if out of memory.  It will not crash if the user
attempts to free the same memory multiple times.  Nor will it crash if the 
user attempts to free a NULL pointer.
   
******************************************************************************/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/block_allocator.h>
/************************************************************
This is the number of QB_BLOCK_SIZE buffers that will fit
in the memory allocated.  Since get_free_pages requests order
7 (2 ** 7, or 128) pages, this value must be adjusted so
that it matches the amount of memory allocates.  For example,
128 pages @ 4k/page = 512k of RAM.  512k/2048 = 256 entries: 
************************************************************/
#define QB_LIST_COUNT 256 
/*
typedef union qb_buffer{
	char data[QB_BLOCK_SIZE];
	union qb_buffer * next;
	}qb_buffer;
*/
qb_buffer * qb_free_pointer;
qb_buffer * qb_list;
char * qb_alloc_list;
char * qb_version = "Quick Block Allocator version 1.0 by Nathan Crawford";

void qb_init(){
	int i;
	qb_alloc_list = kmalloc(QB_LIST_COUNT,GFP_ATOMIC);
	if (!qb_alloc_list){
		printk("%s: Unable to initialize allocator list\n",__FUNCTION__);
		return;
		}
	//qb_list = kmalloc(QB_LIST_COUNT * QB_BLOCK_SIZE,GFP_ATOMIC);
	qb_list = __get_free_pages(GFP_ATOMIC,7);// 512k of memory
	if (!qb_list){
		printk("%s: Unable to initialize block list!\n",__FUNCTION__);
		return;
		}
	else	{
		printk("%s",qb_version);
		}
	for (i = QB_LIST_COUNT - 2; i >= 0; --i){
		qb_list[i].next = &(qb_list[i+1]);
		qb_alloc_list[i] = 'f';
		
		}
	qb_free_pointer = qb_list;
	printk(" : loaded\n");
	}

void * qb_alloc(){
	unsigned int index;
	void * tmp = qb_free_pointer;
	if (qb_free_pointer){
		index = 
		(((unsigned int)qb_free_pointer - 
		(unsigned int)qb_list) / QB_BLOCK_SIZE);
		qb_alloc_list[index] = 'a';// mark entry as allocated.
		qb_free_pointer = qb_free_pointer->next;
		}
	return tmp;
	}			
	
void qb_free(void * fr){
	qb_buffer * freed = (qb_buffer*) fr;
	unsigned int index = 
	(((unsigned int)fr - 
	(unsigned int)qb_list) / QB_BLOCK_SIZE);
	if (freed == NULL)
		return;
	if (index >= QB_LIST_COUNT)
		return; // attempt to free memory not allocated by us.
	if (qb_alloc_list[index] != 'a')
		return; // attempt to free a free node.
	freed->next = qb_free_pointer;
	qb_free_pointer = freed;
	qb_alloc_list[index] = 'f'; // mark entry as free.
	}	