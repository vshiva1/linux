#ifndef __PERF_COUNTS_H
#define __PERF_COUNTS_H

#include "xyarray.h"

/* Not Available (NA) value. Any operation with a NA equals a NA. */
#define PERF_COUNTS_NA ((u64)~0ULL)

struct perf_counts_values {
	union {
		struct {
			u64 val;
			u64 ena;
			u64 run;
		};
		u64 values[3];
	};
};

static inline void
perf_counts_values__make_na(struct perf_counts_values *values)
{
	values->val = PERF_COUNTS_NA;
	values->ena = PERF_COUNTS_NA;
	values->run = PERF_COUNTS_NA;
}

static inline bool
perf_counts_values__is_na(struct perf_counts_values *values)
{
	return values->val == PERF_COUNTS_NA ||
	       values->ena == PERF_COUNTS_NA ||
	       values->run == PERF_COUNTS_NA;
}

struct perf_counts {
	s8			  scaled;
	struct perf_counts_values aggr;
	struct xyarray		  *values;
};


static inline struct perf_counts_values*
perf_counts(struct perf_counts *counts, int cpu, int thread)
{
	return xyarray__entry(counts->values, cpu, thread);
}

struct perf_counts *perf_counts__new(int ncpus, int nthreads);
void perf_counts__delete(struct perf_counts *counts);

void perf_evsel__reset_counts(struct perf_evsel *evsel);
int perf_evsel__alloc_counts(struct perf_evsel *evsel, int ncpus, int nthreads);
void perf_evsel__free_counts(struct perf_evsel *evsel);

#endif /* __PERF_COUNTS_H */
