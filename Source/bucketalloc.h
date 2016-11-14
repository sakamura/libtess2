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

#ifndef MEMALLOC_H
#define MEMALLOC_H

#include <vector>
#include <deque>
#include <cassert>

namespace Tess
{
    class BucketAllocImpl
    {
        typedef std::vector<void*> Bucket;
        std::size_t objectSize;
        unsigned int bucketSize;
        void** firstFree;
        std::deque<Bucket> buckets;
        
        std::size_t allocated, freed;   // For now nothing is freed. If needed, will be garbage collected.

    public:
        ~BucketAllocImpl();
        void* alloc();
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

        static BucketAllocImpl& get(std::size_t count = 0)
        {
            if (count)
            {
                assert((count + sizeof(void*) - 1) / sizeof(void*) <= objectSizeTmpl);
            }
            static BucketAllocSizeImpl bucket;
            return bucket;
        }
    };

    template<typename T, unsigned int bucketSizeTmpl = 512>
    class BucketAlloc : public BucketAllocSizeImpl< (sizeof(T) + sizeof(void*) - 1) / sizeof(void*), bucketSizeTmpl>
    {
    };
}

#endif
