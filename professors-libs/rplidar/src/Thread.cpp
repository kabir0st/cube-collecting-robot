/*
 *	Thread.cpp
 *
 *  Created on: 12 march 2017
 *      Author: hvesch
 */

#include "Thread.hpp"

namespace libcpp
{

Thread::Thread() 
{

}


void * Thread::getArguments()  
{ 
	return _arguments; 
}


void Thread::setArguments(void *args) 
{
	_arguments = args; 
}


int Thread::start(void *args)
{
	setArguments(args);

	_running = true;

	int threadCreateResult = pthread_create(&_theThread, NULL, Thread::threadEntryPoint, this);

	return threadCreateResult;
}


void Thread::stop()
{
	_running = false;
}


void Thread::join()
{
	pthread_join(_theThread, NULL);
}


void Thread::run(void *arguments)
{
	setup();
	execute(arguments);
}


void *Thread::threadEntryPoint(void *pthis)
{
	Thread *pt = (Thread *) pthis;

	pt->run(pt->getArguments());

	return 0;
}

}