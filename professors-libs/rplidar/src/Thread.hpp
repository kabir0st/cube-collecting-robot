/*
 *	Thread.hpp
 *
 *  Created on: 12 march 2017
 *      Author: hvesch
 */

#ifndef _THREAD_HPP
#define _THREAD_HPP

#include <thread>

namespace libcpp
{

class Thread
{

	private:
		pthread_t _theThread;
		void *_arguments;

	protected:
		bool _running;

		void run (void *args);

		static void *threadEntryPoint(void *args);

		virtual void setup() { }
		virtual void execute(void *args) { }

		void * getArguments();
		void setArguments(void *args);

	public:
		Thread();
		int start(void *args);
		void stop();
		void join();

};

}

#endif