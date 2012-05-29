//filter-avg.c
//Averaging filter, using a circular buffer.

#include "filter-avg.h"
#include<stdlib.h>

//Creates a filter and returns a point.
//Caller should check for NULL returns.
void filterAvgCreate(filterAvgInt_t* filt, unsigned int length){
	filt->data = calloc(length, sizeof(int)); //Initialize data to 0
	filt->windowLen = length;
	filt->index = 0;
}

//Add a value to the circular buffer, incrementing index
void filterAvgUpdate(filterAvgInt_t* filt, int newval){
	filt->data[filt->index] = newval;
	filt->index = (filt->index + 1) % filt->windowLen;
}

//Calculate and return average value;
//TODO: more efficient calculation? DSP? delta?
int filterAvgCalc(filterAvgInt_t* filt){
	int i;
	long acc = 0;
	for(i = 0; i < filt->windowLen; i++){
		acc += filt->data[i];
	}
	return acc/(filt->windowLen); //Integer division
}
