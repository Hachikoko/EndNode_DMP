#ifndef _FLITER_H_
#define _FLITER_H_

#include "MPU9250.h"
#include "link_queue.h"

#define FLITER_RAW_N 12
#define FLITER_N (FLITER_RAW_N - 2)

typedef struct splited_raw_data{
	short ax[FLITER_N];
	short ay[FLITER_N];
	short az[FLITER_N];
	short gx[FLITER_N];
	short gy[FLITER_N];
	short gz[FLITER_N];
	short mx[FLITER_N];
	short my[FLITER_N];
	short mz[FLITER_N];
}Splited_Raw_Data;

typedef struct fliter_result{
	int ax;
	int ay;
	int az;
	int gx;
	int gy;
	int gz;
	int mx;
	int my;
	int mz;
}Fliter_Result;

void fill_data(Splited_Raw_Data* ptr_split_raw_data,Link_Queue* prt_link_raw_data);
void sort_split_data(Splited_Raw_Data* ptr_split_raw_data);
Fliter_Result* middle_average_filter(Link_Queue* prt_link_raw_data,Fliter_Result* ptr_fliter_result);


#endif //fliter.h
