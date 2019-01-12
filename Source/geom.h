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
** Original Author: Eric Veach, July 1994.
*/

#pragma once

#include "mesh.h"
#include <cmath>

namespace Tess
{
    template <typename U, typename V>
	inline bool vertAreEqual(const U* u, const V* v) { return (u->getS() == v->getS() && u->getT() == v->getT()); }

    template <typename U, typename V>
    inline bool vertAreLessOrEqual(const U* u, const V* v) { return ((u->getS() < v->getS()) || (u->getS() == v->getS() && u->getT() <= v->getT())); }
	
    template <typename U, typename V>
    inline bool transVertAreLessOrEqual(const U* u, const V* v) { return ((u->getT() < v->getT()) || (u->getT() == v->getT() && u->getS() <= v->getS())); }

    template <typename Options, typename Allocators>
    inline bool edgeGoesLeft(const HalfEdgeT<Options, Allocators>* e) { return vertAreLessOrEqual( e->Dst(), e->Org() ); }
	
    template <typename Options, typename Allocators>
    inline bool edgeGoesRight(const HalfEdgeT<Options, Allocators>* e) { return vertAreLessOrEqual( e->Org(), e->Dst() ); }
	
    template <typename Options, typename Allocators>
    inline bool edgeIsInternal(const HalfEdgeT<Options, Allocators>* e) { return e->Rface() && e->Rface()->inside; }
	
    template <typename U, typename V, typename W>
    inline bool vertAreCCW( const U *u, const V *v, const W *w ) { return (u->getS()*(v->getT() - w->getT()) + v->getS()*(w->getT() - u->getT()) + w->getS()*(u->getT() - v->getT())) >= 0; }
	
	
    template <typename U, typename V, typename W>
    typename U::value_type edgeEval( const U *u, const V *v, const W *w ); // Returns the signed distance from uw to v.
	
    template <typename U, typename V, typename W>
    typename U::value_type edgeSign( const U *u, const V *v, const W *w ); // Returns a number whose sign matches edgeEval(u,v,w) but which is cheaper to evaluate.
	
    template <typename U, typename V, typename W>
    typename U::value_type transEdgeEval( const U *u, const V *v, const W *w ); // Transposed version of edgeEval
    
    template <typename U, typename V, typename W>
    typename U::value_type transEdgeSign( const U *u, const V *v, const W *w ); // Transposed version of edgeSign
	
    template <typename O1, typename D1, typename O2, typename D2, typename V>
    void edgeIntersect( const O1 *o1, const D1 *d1, const O2 *o2, const D2 *d2, V *v ); // Given edges (o1,d1) and (o2,d2), compute their point of intersection.
	
    template <typename Options, typename Allocators>
    bool edgeIsLocallyDelaunay( const HalfEdgeT<Options, Allocators> *e );
}

#include "geom.inl"
