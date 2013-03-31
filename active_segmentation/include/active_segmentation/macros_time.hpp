#ifndef DIRINI_MACROS_HPP_
#define DIRINI_MACROS_HPP_

#include <sys/time.h>
#include <iostream>


#define INIT_PROFILING struct timeval profiling_start_time, profiling_end_time; gettimeofday(&profiling_start_time, NULL);
#define RESET gettimeofday(&profiling_start_time, NULL);

#define MEASURE(text) 	gettimeofday(&profiling_end_time, NULL); std::cout << "time for " << text << " " << ((profiling_end_time.tv_sec - profiling_start_time.tv_sec) * 1000000u + profiling_end_time.tv_usec - profiling_start_time.tv_usec) /1000000. << " s" << std::endl; gettimeofday(&profiling_start_time, NULL);



#endif
