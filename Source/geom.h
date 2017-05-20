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

#ifndef GEOM_H
#define GEOM_H

#include "mesh.h"
#include <cmath>

namespace Tess
{
    inline bool vertAreEqual(const Vertex* u, const Vertex* v) { return ((u)->s == (v)->s && (u)->t == (v)->t); }
    inline bool vertAreLessOrEqual(const Vertex* u, const Vertex* v) { return (((u)->s < (v)->s) || ((u)->s == (v)->s && (u)->t <= (v)->t)); }
    inline bool transVertAreLessOrEqual(const Vertex* u, const Vertex* v) { return (((u)->t < (v)->t) || ((u)->t == (v)->t && (u)->s <= (v)->s)); }
    inline bool edgeGoesLeft(const HalfEdge* e) { return vertAreLessOrEqual( e->Dst(), e->Org() ); }
    inline bool edgeGoesRight(const HalfEdge* e) { return vertAreLessOrEqual( e->Org(), e->Dst() ); }
    inline bool edgeIsInternal(const HalfEdge* e) { return e->Rface() && e->Rface()->inside; }
    inline bool vertAreCCW( const Vertex *u, const Vertex *v, const Vertex *w ) { return (u->s*(v->t - w->t) + v->s*(w->t - u->t) + w->s*(u->t - v->t)) >= 0; }
    
    float edgeEval( const Vertex *u, const Vertex *v, const Vertex *w ); // Returns the signed distance from uw to v.
    float edgeSign( const Vertex *u, const Vertex *v, const Vertex *w ); // Returns a number whose sign matches edgeEval(u,v,w) but which is cheaper to evaluate.
    float transEdgeEval( const Vertex *u, const Vertex *v, const Vertex *w ); // Transposed version of edgeEval
    float transEdgeSign( const Vertex *u, const Vertex *v, const Vertex *w ); // Transposed version of edgeSign
    void edgeIntersect( const Vertex *o1, const Vertex *d1, const Vertex *o2, const Vertex *d2, Vertex *v ); // Given edges (o1,d1) and (o2,d2), compute their point of intersection.
    bool edgeIsLocallyDelaunay( const HalfEdge *e );
}

#endif
