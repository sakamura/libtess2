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
** Author: Michel Donais, November 2016.
*/

#pragma once

#include <vector>
#include <deque>
#include <cassert>

namespace Tess
{
    // This system is a compatible subset of Boost's object_pool.
	class BucketAllocImpl
	{
		typedef std::vector<void*> Bucket;
		std::size_t objectSize;
		unsigned int bucketSize;
		void** firstFree;
		std::deque<Bucket> buckets;
		
		std::size_t allocated, freed;

	public:
		~BucketAllocImpl();
		void* malloc();
		void free(void*);
	protected:
		BucketAllocImpl(std::size_t objectSize, unsigned int bucketSize);
	private:
		void newBucket();
	};
	
	template<std::size_t objectSizeTmpl, unsigned int bucketSizeTmpl = 512>
	class BucketAllocSizeImpl : public BucketAllocImpl
	{
	public:
		BucketAllocSizeImpl() : BucketAllocImpl(objectSizeTmpl, bucketSizeTmpl) {}
	};

	template<typename T, unsigned int bucketSizeTmpl = 512>
	class BucketAlloc : public BucketAllocSizeImpl< (sizeof(T) + sizeof(void*) - 1) / sizeof(void*), bucketSizeTmpl>
	{
    public:
        template <typename... Args>
        T* construct(Args&&... args)
        {
            void* ptr = this->malloc();
            return new (ptr) T(std::forward<Args>(args)...);
        }
        void destroy(T* t)
        {
            t->~T();
            free( (void*)t );
        }
	};
}

#include "bucketalloc.inl"
