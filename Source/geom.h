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
    inline bool vertAreEqual(const TESSvertex* u, const TESSvertex* v) { return ((u)->s == (v)->s && (u)->t == (v)->t); }
    inline bool vertAreLessOrEqual(const TESSvertex* u, const TESSvertex* v) { return (((u)->s < (v)->s) || ((u)->s == (v)->s && (u)->t <= (v)->t)); }
    inline bool transVertAreLessOrEqual(const TESSvertex* u, const TESSvertex* v) { return (((u)->t < (v)->t) || ((u)->t == (v)->t && (u)->s <= (v)->s)); }
    inline bool edgeGoesLeft(const TESShalfEdge* e) { return vertAreLessOrEqual( (e)->Dst, (e)->Org ); }
    inline bool edgeGoesRight(const TESShalfEdge* e) { return vertAreLessOrEqual( (e)->Org, (e)->Dst ); }
    inline bool edgeIsInternal(const TESShalfEdge* e) { return e->Rface && e->Rface->inside; }
    inline bool vertAreCCW( const TESSvertex *u, const TESSvertex *v, const TESSvertex *w ) { return (u->s*(v->t - w->t) + v->s*(w->t - u->t) + w->s*(u->t - v->t)) >= 0; }
    
    float edgeEval( const TESSvertex *u, const TESSvertex *v, const TESSvertex *w ); // Returns the signed distance from uw to v.
    float edgeSign( const TESSvertex *u, const TESSvertex *v, const TESSvertex *w ); // Returns a number whose sign matches edgeEval(u,v,w) but which is cheaper to evaluate.
    float transEdgeEval( const TESSvertex *u, const TESSvertex *v, const TESSvertex *w ); // Transposed version of edgeEval
    float transEdgeSign( const TESSvertex *u, const TESSvertex *v, const TESSvertex *w ); // Transposed version of edgeSign
    void edgeIntersect( const TESSvertex *o1, const TESSvertex *d1, const TESSvertex *o2, const TESSvertex *d2, TESSvertex *v ); // Given edges (o1,d1) and (o2,d2), compute their point of intersection.
    bool edgeIsLocallyDelaunay( const TESShalfEdge *e );
}

#endif
