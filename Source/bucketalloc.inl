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
** Original Author: Mikko Mononen, July 2009.
*/

#include <cassert>
#include "bucketalloc.h"

namespace Tess
{
    template<std::size_t objectSize, unsigned int bucketSize>
	BucketAllocImpl<objectSize, bucketSize>::BucketAllocImpl(const BucketPoolSharedPtr& _bucketPool) :
		firstFree(nullptr),
		allocated(0),
		freed(0),
        bucketPool(_bucketPool)
	{
        buckets.reserve(16);
	}
    template<std::size_t objectSize, unsigned int bucketSize>
	BucketAllocImpl<objectSize, bucketSize>::~BucketAllocImpl()
	{
        if (bucketPool)
        {
            auto& bp = *bucketPool;
            std::lock_guard<decltype(bp.mutex)> guard(bp.mutex);
            
            bp.unusedBuckets.insert(bp.unusedBuckets.end(),
                                    std::make_move_iterator(buckets.begin()),
                                    std::make_move_iterator(buckets.end()));

        }
	}
    template<std::size_t objectSize, unsigned int bucketSize>
	void* BucketAllocImpl<objectSize, bucketSize>::malloc()
	{
		if (!firstFree)
		{
			if (buckets.empty())
			{
				newBucket();
			}
			else
			{
				Bucket& back(buckets.back());
				if (back.max_size == back.size)
				{
					newBucket();
				}
			}
			{
				Bucket& back(buckets.back());
				std::size_t origSize = back.size;
				back.size += objectSize;
				++allocated;
				return (void*)&back.data[origSize];
			}
		}
		else
		{
			void* result = firstFree;
			firstFree = *(void***)result;
			++allocated;
			return result;
		}
	}
    template<std::size_t objectSize, unsigned int bucketSize>
	void BucketAllocImpl<objectSize, bucketSize>::free(void* ptr)
	{
		*(void**)(ptr) = (void*)firstFree;
		firstFree = (void**)ptr;
		++freed;
	}
    template<std::size_t objectSize, unsigned int bucketSize>
	void BucketAllocImpl<objectSize, bucketSize>::newBucket()
	{
        if (bucketPool && !bucketPool->unusedBuckets.empty())
        {
            auto& bp = *bucketPool;
            std::lock_guard<decltype(bp.mutex)> guard(bp.mutex);
            
            if (!bp.unusedBuckets.empty())
            {
                buckets.insert(buckets.end(),
                               std::make_move_iterator(bp.unusedBuckets.rbegin()),
                               std::make_move_iterator(bp.unusedBuckets.rbegin()+1));
                buckets.back().size = 0;
                bp.unusedBuckets.pop_back();
                return;
            }
        }
        
        buckets.emplace_back(buckets.size(), true);         // Create data by using the special constructor that creates it
	}
}
