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
 ** Original Author: Michel Donais, July 2018.
 */

#pragma once

namespace Tess
{
    // This pool can be instantiated in the options, and it will be used by the base allocators.
    // Since these are abused throughout the tesselation process, the goal is to keep the unused pools around
    // so they can be easily claimed by tesselators instead of asking the system for a new large chunk of memory.
    template <typename _Allocators>
    struct AllocatorPool
    {
        using Allocators = _Allocators;
        
        using DictNodeShared = typename decltype(Allocators::dictNodeAlloc)::BucketPoolSharedPtr;
        using VertexShared = typename decltype(Allocators::vertexAlloc)::BucketPoolSharedPtr;
        using FaceShared = typename decltype(Allocators::faceAlloc)::BucketPoolSharedPtr;
        using EdgePairShared = typename decltype(Allocators::edgePairAlloc)::BucketPoolSharedPtr;
        using ActiveRegionShared = typename decltype(Allocators::activeRegionAlloc)::BucketPoolSharedPtr;
        using EdgeStackNodeShared = typename decltype(Allocators::edgeStackNodeAlloc)::BucketPoolSharedPtr;
        
        DictNodeShared dictNodePool;
        VertexShared vertexPool;
        FaceShared facePool;
        EdgePairShared edgePairPool;
        ActiveRegionShared activeRegionPool;
        EdgeStackNodeShared edgeStackNodePool;
        
        AllocatorPool() :
            dictNodePool(new typename decltype(dictNodePool)::element_type),
            vertexPool(new typename decltype(vertexPool)::element_type),
            facePool(new typename decltype(facePool)::element_type),
            edgePairPool(new typename decltype(edgePairPool)::element_type),
            activeRegionPool(new typename decltype(activeRegionPool)::element_type),
            edgeStackNodePool(new typename decltype(edgeStackNodePool)::element_type)
        {}
        
        void free()
        {
            dictNodePool.free();
            vertexPool.free();
            facePool.free();
            edgePairPool.free();
            activeRegionPool.free();
            edgeStackNodePool.free();
        }
    };

    // These are the elements worthwhile to be put in a bucket allocator.
    // The BucketAlloc system keeps the buckets open for the duration of the Tesselator object.
    // Once closed down, all the allocators are freed.
    // If you have access to Boost, you can use a object_pool instead, which will bring it as a 1:1
    // replacement, with the only difference being the second optional template argument is actually
    // the first constructor argument, and the default value for BucketAlloc being 512, and 32 for Boost.
    // (Please note that as of current version of Boost, their object_pool is really inefficient in our
    // particular use-case, so you might not be happy of the performance)
    template <typename _Options>
    struct BaseAllocators
    {
        using Options = _Options;
        using Vertex = VertexT<Options, BaseAllocators>;
        using Face = FaceT<Options, BaseAllocators>;
        using EdgePair = EdgePairT<Options, BaseAllocators>;
        using ActiveRegion = ActiveRegionT<Options, BaseAllocators>;
        using EdgeStackNode = typename EdgeStackT<Options, BaseAllocators>::Node;
        using AllocatorPool = AllocatorPool<BaseAllocators>;
        
        BucketAlloc<DictNode, 8192> dictNodeAlloc;
        BucketAlloc<Vertex, 4096> vertexAlloc;
        BucketAlloc<Face> faceAlloc;
        BucketAlloc<EdgePair> edgePairAlloc;
        BucketAlloc<ActiveRegion> activeRegionAlloc;
        BucketAlloc<EdgeStackNode, 2048> edgeStackNodeAlloc;

        BaseAllocators(Options& options) :
        dictNodeAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->dictNodePool : typename AllocatorPool::DictNodeShared() ),
        vertexAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->vertexPool : typename AllocatorPool::VertexShared() ),
            faceAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->facePool : typename AllocatorPool::FaceShared() ),
            edgePairAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->edgePairPool : typename AllocatorPool::EdgePairShared() ),
            activeRegionAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->activeRegionPool : typename AllocatorPool::ActiveRegionShared() ),
            edgeStackNodeAlloc(options.template getAllocatorPool<AllocatorPool>() ? options.template getAllocatorPool<AllocatorPool>()->edgeStackNodePool : typename AllocatorPool::EdgeStackNodeShared() )
        {}
    };
}
