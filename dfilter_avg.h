//dfilter_avg.h
#include <dsp.h>

typedef struct {
	unsigned int windowLen;
	unsigned int index;
	int* data;
        long accum;
} filterAvgInt_t;

void filterAvgCreate(filterAvgInt_t*, unsigned int);
void filterAvgUpdate(filterAvgInt_t*, int);
int filterAvgCalc(filterAvgInt_t*);
