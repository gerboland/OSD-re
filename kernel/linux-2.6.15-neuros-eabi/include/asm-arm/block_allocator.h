/******************************************************************************
block_allocator.h -> high performance memory block allocation for io and 
drivers.

12/23/2004 Nathan J. Crawford

******************************************************************************/
#define QB_BLOCK_SIZE 2048

typedef union qb_buffer{
	char data[QB_BLOCK_SIZE];
	union qb_buffer * next;
	}qb_buffer;

extern void qb_init();
extern void * qb_alloc();		
extern void qb_free(void * fr);
