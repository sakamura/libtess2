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
** Author: Eric Veach, July 1994.
*/

#pragma once

#include "mesh.h"
#include <cmath>

namespace Tess
{
    template <typename Options, typename Allocators>
	inline bool vertAreEqual(const VertexT<Options, Allocators>* u, const VertexT<Options, Allocators>* v) { return ((u)->s == (v)->s && (u)->t == (v)->t); }

    template <typename Options, typename Allocators>
    inline bool vertAreLessOrEqual(const VertexT<Options, Allocators>* u, const VertexT<Options, Allocators>* v) { return (((u)->s < (v)->s) || ((u)->s == (v)->s && (u)->t <= (v)->t)); }
	
    template <typename Options, typename Allocators>
    inline bool transVertAreLessOrEqual(const VertexT<Options, Allocators>* u, const VertexT<Options, Allocators>* v) { return (((u)->t < (v)->t) || ((u)->t == (v)->t && (u)->s <= (v)->s)); }

    template <typename Options, typename Allocators>
    inline bool edgeGoesLeft(const HalfEdgeT<Options, Allocators>* e) { return vertAreLessOrEqual( e->Dst(), e->Org() ); }
	
    template <typename Options, typename Allocators>
    inline bool edgeGoesRight(const HalfEdgeT<Options, Allocators>* e) { return vertAreLessOrEqual( e->Org(), e->Dst() ); }
	
    template <typename Options, typename Allocators>
    inline bool edgeIsInternal(const HalfEdgeT<Options, Allocators>* e) { return e->Rface() && e->Rface()->inside; }
	
    template <typename Options, typename Allocators>
    inline bool vertAreCCW( const VertexT<Options, Allocators> *u, const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *w ) { return (u->s*(v->t - w->t) + v->s*(w->t - u->t) + w->s*(u->t - v->t)) >= 0; }
	
	
    template <typename Options, typename Allocators>
    typename Options::Coord edgeEval( const VertexT<Options, Allocators> *u, const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *w ); // Returns the signed distance from uw to v.
	
    template <typename Options, typename Allocators>
    typename Options::Coord edgeSign( const VertexT<Options, Allocators> *u, const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *w ); // Returns a number whose sign matches edgeEval(u,v,w) but which is cheaper to evaluate.
	
    template <typename Options, typename Allocators>
    typename Options::Coord transEdgeEval( const VertexT<Options, Allocators> *u, const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *w ); // Transposed version of edgeEval
    
    template <typename Options, typename Allocators>
    typename Options::Coord transEdgeSign( const VertexT<Options, Allocators> *u, const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *w ); // Transposed version of edgeSign
	
    template <typename Options, typename Allocators>
    void edgeIntersect( const VertexT<Options, Allocators> *o1, const VertexT<Options, Allocators> *d1, const VertexT<Options, Allocators> *o2, const VertexT<Options, Allocators> *d2, VertexT<Options, Allocators> *v ); // Given edges (o1,d1) and (o2,d2), compute their point of intersection.
	
    template <typename Options, typename Allocators>
    bool edgeIsLocallyDelaunay( const HalfEdgeT<Options, Allocators> *e );
}

#include "geom.inl"
