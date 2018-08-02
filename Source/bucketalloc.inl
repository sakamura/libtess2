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

#include <cassert>
#include "bucketalloc.h"

namespace Tess
{
	struct Header
	{
		BucketAllocImpl* bucket;
	};

	BucketAllocImpl::BucketAllocImpl(std::size_t _objectSize, unsigned int _bucketSize) :
		objectSize(sizeof(Header) + _objectSize),
		bucketSize(_bucketSize),
		firstFree(nullptr),
		allocated(0),
		freed(0)
	{
	}
	BucketAllocImpl::~BucketAllocImpl()
	{
	}
	void* BucketAllocImpl::malloc()
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
				if (back.capacity() == back.size())
				{
					newBucket();
				}
			}
			{
				Bucket& back(buckets.back());
				std::size_t origSize = back.size();
				back.resize(origSize+objectSize);
				back[origSize] = (void*)this;
				++allocated;
				return (void*)&back[origSize + 1];
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
	void BucketAllocImpl::free(void* ptr)
	{
		*(void**)(ptr) = (void*)firstFree;
		firstFree = (void**)ptr;
		++freed;
	}
	void BucketAllocImpl::newBucket()
	{
		buckets.emplace_back();
		buckets.back().reserve(bucketSize * objectSize);
	}

}
