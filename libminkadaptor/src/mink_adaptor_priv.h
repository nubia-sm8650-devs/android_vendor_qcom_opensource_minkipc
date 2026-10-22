// Copyright (c) 2025, Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _MINK_ADAPTOR_PRIV_H_
#define _MINK_ADAPTOR_PRIV_H_

#include <qcomtee_object_types.h>
#include "object.h"

/* Suppress verbose and debug logs by default */
#ifdef MSGV
#undef MSGV
#define MSGV(...)
#endif

#ifdef MSGD
#undef MSGD
#define MSGD(...)
#endif

#define container_of(ptr, type, member) \
	((type *)((void *)(ptr) - __builtin_offsetof(type, member)))

#define DEFAULT_CBOBJ_THREAD_CNT 4

#define MAX_OBJ_ARG_COUNT                                               \
	(ObjectCounts_maxBI + ObjectCounts_maxBO + ObjectCounts_maxOI + \
	 ObjectCounts_maxOO)

#define FOR_ARGS(ndxvar, counts, section)                         \
	for (size_t ndxvar = ObjectCounts_index##section(counts); \
	     ndxvar < (ObjectCounts_index##section(counts) +      \
		       ObjectCounts_num##section(counts));        \
	     ++ndxvar)

struct qcomtee_callback_obj {
	struct qcomtee_object object;
	Object mink_obj;
};

#define CALLBACKOBJ(o) container_of((o), struct qcomtee_callback_obj, object)

#endif // _MINK_ADAPTOR_PRIV_H_
