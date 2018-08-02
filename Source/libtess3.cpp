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


#include "tess.h"

extern const char* const LIBTESS_VERSION = "libTess3 1.0.0";

/*
extern Tess::Tesselator<> Test(Tess::BaseOptions());

#include <boost/pool/object_pool.hpp>

template <typename Options>
struct BoostAllocators
{
    using Vertex = Tess::VertexT<Options, BoostAllocators>;
    using Face = Tess::FaceT<Options, BoostAllocators>;
    using EdgePair = Tess::EdgePairT<Options, BoostAllocators>;
    using ActiveRegion = Tess::ActiveRegionT<Options, BoostAllocators>;
    using EdgeStackNode = typename Tess::EdgeStackT<Options, BoostAllocators>::Node;
    
    boost::object_pool<Tess::DictNode> dictNodeAlloc;
    boost::object_pool<Vertex> vertexAlloc;
    boost::object_pool<Face> faceAlloc;
    boost::object_pool<EdgePair> edgePairAlloc;
    boost::object_pool<ActiveRegion> activeRegionAlloc;
    boost::object_pool<EdgeStackNode> edgeStackNodeAlloc;
    
    BoostAllocators() :
        dictNodeAlloc(512),
        vertexAlloc(512),
        faceAlloc(512),
        edgePairAlloc(512),
        activeRegionAlloc(512),
        edgeStackNodeAlloc(2048)
    {
    }
};
extern Tess::Tesselator<Tess::BaseOptions, BoostAllocators<Tess::BaseOptions> > Test2(Tess::BaseOptions());
*/
