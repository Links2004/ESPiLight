/*
	Copyright (C) 2013 - 2016 CurlyMo

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef _MEM_H_
#define _MEM_H_

#define OUT_OF_MEMORY fprintf(stderr, "out of memory in %s #%d\n", __FILE__, __LINE__),exit(EXIT_FAILURE);

#ifdef __GNUC__
	#define atomic_inc(ptr) __sync_add_and_fetch(&(ptr), 1)
	#define atomic_dec(ptr) __sync_add_and_fetch(&(ptr), -1)
#elif defined (_WIN32)
	#define atomic_inc(ptr) InterlockedIncrement(&(ptr))
	#define atomic_dec(ptr) InterlockedDecrement(&(ptr))
#else
	#error "Need some more porting work here"
#endif

#if !defined(ESP8266)
void *__malloc(unsigned long, const char *, int);
void *__realloc(void *, unsigned long, const char *, int);
void *__calloc(unsigned long a, unsigned long b, const char *, int);
void __free(void *, const char *, int);
/*
 * Windows already has a _strdup, libpcap uses __strdup
 */
char *___strdup(char *, const char *, int);
#endif

#if defined(DEBUG) && !defined(__mips__) && !defined(__aarch64__) && !defined(ESP8266)
	#define MALLOC(a) __malloc(a, __FILE__, __LINE__)
	#define REALLOC(a, b) __realloc(a, b, __FILE__, __LINE__)
	#define CALLOC(a, b) __calloc(a, b, __FILE__, __LINE__)
	#define STRDUP(a) ___strdup(a, __FILE__, __LINE__)
	#define FREE(a) __free((void *)(a), __FILE__, __LINE__),(a)=NULL
#else
	#define MALLOC malloc
	#define REALLOC realloc
	#define CALLOC calloc
	#define STRDUP strdup
	#define FREE(a) free((void *)(a)),(a)=NULL
#endif

#endif
