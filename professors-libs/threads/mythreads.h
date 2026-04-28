#ifndef _THREADS_H
#define _THREADS_H



void * Server(void *);
void * Client(void *);
extern int Send_Flag;
extern int Global_Counter;

#endif
