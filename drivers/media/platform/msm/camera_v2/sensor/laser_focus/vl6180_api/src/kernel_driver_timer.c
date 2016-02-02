#include <linux/timer.h>

/* Display current time */
struct timeval get_current_time(void){
	struct timeval now;
	
	do_gettimeofday(&now);

	/*
		tv_sec: second
		tv_usec: microseconds 
	*/
	//printk("Current UTC: %lu (%lu)\n", now.tv_sec, now.tv_usec);

	return now;
}
