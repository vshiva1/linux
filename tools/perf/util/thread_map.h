#ifndef __PERF_THREAD_MAP_H
#define __PERF_THREAD_MAP_H

#include <sys/types.h>
#include <stdio.h>
#include "util.h"

struct thread_map {
	int nr;
	int p_nr;
	pid_t map[];
};

struct thread_map *thread_map__new_dummy(void);
struct thread_map *thread_map__new_by_pid(pid_t pid);
struct thread_map *thread_map__new_by_tid(pid_t tid);
struct thread_map *thread_map__new_by_uid(uid_t uid);
struct thread_map *thread_map__new(pid_t pid, pid_t tid, uid_t uid);

struct thread_map *thread_map__new_str(const char *pid,
		const char *tid, uid_t uid);

void thread_map__delete(struct thread_map *threads);
int thread_map__build_aggr_pid_map(const char *pid_str,
				    struct proc_map **map,
				    struct thread_map *threads);
int thread_map_aggr_tindex(int tindex, struct proc_map *pmap);

size_t thread_map__fprintf(struct thread_map *threads, FILE *fp);

static inline int thread_map__nr(struct thread_map *threads)
{
	return threads ? threads->nr : 1;
}

#endif	/* __PERF_THREAD_MAP_H */
