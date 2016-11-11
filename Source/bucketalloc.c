/*
** SGI FREE SOFTWARE LICENSE B (Version 2.0, Sept. 18, 2008) 
** Copyright (C) [dates of first publication] Silicon Graphics, Inc.
** All Rights Reserved.
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
** of the Software, and to permit persons to whom the Software is furnished to do so,
** subject to the following conditions:
** 
** The above copyright notice including the dates of first publication and either this
** permission notice or a reference to http://oss.sgi.com/projects/FreeB/ shall be
** included in all copies or substantial portions of the Software. 
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
** INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
** PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL SILICON GRAPHICS, INC.
** BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
** TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
** OR OTHER DEALINGS IN THE SOFTWARE.
** 
** Except as contained in this notice, the name of Silicon Graphics, Inc. shall not
** be used in advertising or otherwise to promote the sale, use or other dealings in
** this Software without prior written authorization from Silicon Graphics, Inc.
*/
/*
** Author: Mikko Mononen, July 2009.
*/

#include <stdio.h>
#include <stdlib.h>
#include "../Include/tesselator.h"

typedef struct BucketAlloc BucketAlloc;
typedef struct Bucket Bucket;

struct Bucket
{
    Bucket *prev;
	Bucket *next;
};

struct BucketAlloc
{
	void *freelist;
	Bucket *buckets;
    void *initMark;
    size_t bucketByteSize;
	unsigned int itemSize;
	unsigned int bucketSize;
	TESSalloc* alloc;
    
    BucketAlloc *next;
};

static char bAutomaticCleanup = 1;
void tessDisableAutomaticCleanup()
{
    bAutomaticCleanup = 0;
}

static int CreateBucket( struct BucketAlloc* ba )
{
    Bucket* bucket;
    unsigned char* head;
    if (ba->buckets && ba->buckets->prev)
    {
        bucket = ba->buckets->prev;
    }
    else if (ba->buckets && !ba->initMark)
    {
        bucket = ba->buckets;
    }
    else
    {
        // Allocate memory for the bucket
        bucket = (Bucket*)ba->alloc->memalloc( ba->alloc->userData, ba->bucketByteSize );
        if ( !bucket )
            return 0;

        // Add the bucket into the list of buckets.
        if (ba->buckets) ba->buckets->prev = bucket;
        bucket->prev = 0;
        bucket->next = ba->buckets;
    }
    
	ba->buckets = bucket;
	head = (unsigned char*)bucket + sizeof(Bucket);
    ba->freelist = head;
    ba->initMark = head;

	return 1;
}

static void *NextFreeItem( struct BucketAlloc *ba )
{
	return *(void**)ba->freelist;
}

struct BucketAlloc* createBucketAlloc( TESSalloc* alloc, const char* name,
									  unsigned int itemSize, unsigned int bucketSize )
{
    BucketAlloc *ba;
    BucketAlloc *prevBa;
    if ( itemSize < sizeof(void*) )
    {
        itemSize = sizeof(void*);
    }
    
    for (ba = (BucketAlloc*)alloc->freeAllocs, prevBa = 0;
         ba != 0;
         ba = ba->next)
    {
        if (ba->itemSize == itemSize && ba->itemSize == itemSize && ba->bucketSize == bucketSize)
            break;
        prevBa = ba;
    }

    if (ba)
    {
        if (prevBa)
        {
            prevBa->next = ba->next;
        }
        else
        {
            alloc->freeAllocs = ba->next;
        }
        ba->next = 0;
    }
    else
    {
        ba = (BucketAlloc*)alloc->memalloc( alloc->userData, sizeof(BucketAlloc) );
        if (!ba)
        {
            return 0;
        }

        ba->alloc = alloc;
        ba->itemSize = itemSize;
        ba->bucketSize = bucketSize;
        ba->bucketByteSize = sizeof(Bucket) + ba->itemSize * ba->bucketSize;
        ba->freelist = 0;
        ba->buckets = 0;
        ba->initMark = 0;
        ba->next = 0;
    }
    
	return ba;
}

void* bucketAlloc( struct BucketAlloc *ba )
{
	void *it;

	// If running out of memory, allocate new bucket and update the freelist.
	if ( !ba->freelist )
	{
		if ( !CreateBucket( ba ) )
			return 0;
	}

	// Pop item from in front of the free list.
	it = ba->freelist;
    if (it == ba->initMark)
    {
        ba->initMark = (char*)ba->initMark + ba->itemSize;
        if (ba->initMark == (char*)ba->buckets + ba->bucketByteSize )
        {
            ba->freelist = NULL;
        }
        else
        {
            ba->freelist = ba->initMark;
        }
    }
    else
    {
        ba->freelist = NextFreeItem( ba );
    }
	return it;
}

void bucketFree( struct BucketAlloc *ba, void *ptr )
{
	// Add the node in front of the free list.
	*(void**)ptr = ba->freelist;
	ba->freelist = ptr;
}

void deleteBucketAlloc( struct BucketAlloc *ba )
{
    if (ba->buckets)
    {
        while (ba->buckets->next)
        {
            ba->buckets = ba->buckets->next;
        }
    }
	ba->freelist = 0;
    ba->initMark = 0;
    
    ba->next = (BucketAlloc*)ba->alloc->freeAllocs;
    ba->alloc->freeAllocs = ba;
    
    if (bAutomaticCleanup)
    {
        tessCleanupAlloc(ba->alloc);
    }
}

void tessCleanupAlloc( struct TESSalloc* alloc )
{
    BucketAlloc *ba;
    BucketAlloc *nextBa;
    Bucket *bucket;
    Bucket *next;
    for (ba = (BucketAlloc*)alloc->freeAllocs;
         ba != 0;
         ba = nextBa)
    {
        bucket = ba->buckets;
        while (bucket)
        {
            next = bucket->next;
            alloc->memfree(alloc->userData, bucket);
            bucket = next;
        }
        nextBa = ba->next;
        alloc->memfree(alloc->userData, ba);
    }
    alloc->freeAllocs = 0;
}
