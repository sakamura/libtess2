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
 ** Author: Michel Donais, July 2018.
 */

#pragma once

namespace Tess
{
    // These are the elements worthwhile to be put in a bucket allocator.
    // The BucketAlloc system keeps the buckets open for the duration of the Tesselator object.
    // Once closed down, all the allocators are freed.
    // If you have access to Boost, you can use a object_pool instead, which will bring it as a 1:1
    // replacement, with the only difference being the second optional template argument is actually
    // the first constructor argument, and the default value for BucketAlloc being 512, and 32 for Boost.
    template <typename Options>
    struct BaseAllocators
    {
        using Vertex = VertexT<Options, BaseAllocators>;
        using Face = FaceT<Options, BaseAllocators>;
        using EdgePair = EdgePairT<Options, BaseAllocators>;
        using ActiveRegion = ActiveRegionT<Options, BaseAllocators>;
        using EdgeStackNode = typename EdgeStackT<Options, BaseAllocators>::Node;
        
        BucketAlloc<DictNode> dictNodeAlloc;
        BucketAlloc<Vertex> vertexAlloc;
        BucketAlloc<Face> faceAlloc;
        BucketAlloc<EdgePair> edgePairAlloc;
        BucketAlloc<ActiveRegion> activeRegionAlloc;
        BucketAlloc<EdgeStackNode, 2048> edgeStackNodeAlloc;
    };
}
