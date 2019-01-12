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

//#include "tesos.h"
#include <assert.h>
#include "mesh.h"
#include "geom.h"
#include <math.h>
#include <stdlib.h>

namespace Tess
{
    template <typename U, typename V, typename W>
    typename U::value_type edgeEval( const U *u, const V *v, const W *w )
	{
		/* Given three vertices u,v,w such that vertAreLessOrEqual(u,v) && vertAreLessOrEqual(v,w),
		 * evaluates the t-coord of the edge uw at the s-coord of the vertex v.
		 * Returns v->getT() - (uw)(v->getS()), ie. the signed distance from uw to v.
		 * If uw is vertical (and thus passes thru v), the result is zero.
		 *
		 * The calculation is extremely accurate and stable, even when v
		 * is very close to u or w.	 In particular if we set v->getT() = 0 and
		 * let r be the negated result (this evaluates (uw)(v->getS())), then
		 * r is guaranteed to satisfy MIN(u->getT(),w->getT()) <= r <= MAX(u->getT(),w->getT()).
		 */
        typename U::value_type gapL, gapR;
		
		assert( vertAreLessOrEqual( u, v ) && vertAreLessOrEqual( v, w ));
		
		gapL = v->getS() - u->getS();
		gapR = w->getS() - v->getS();
		
		if( gapL + gapR > 0 ) {
			if( gapL < gapR ) {
				return (v->getT() - u->getT()) + (u->getT() - w->getT()) * (gapL / (gapL + gapR));
			} else {
				return (v->getT() - w->getT()) + (w->getT() - u->getT()) * (gapR / (gapL + gapR));
			}
		}
		/* vertical line */
		return 0;
	}
	
    template <typename U, typename V, typename W>
    typename U::value_type edgeSign( const U *u, const V *v, const W *w )
	{
		/* Returns a number whose sign matches edgeEval(u,v,w) but which
		 * is cheaper to evaluate.	Returns > 0, == 0 , or < 0
		 * as v is above, on, or below the edge uw.
		 */
        typename U::value_type gapL, gapR;
		
		assert( vertAreLessOrEqual( u, v ) && vertAreLessOrEqual( v, w ));
		
		gapL = v->getS() - u->getS();
		gapR = w->getS() - v->getS();
		
		if( gapL + gapR > 0 ) {
			return (v->getT() - w->getT()) * gapL + (v->getT() - u->getT()) * gapR;
		}
		/* vertical line */
		return 0;
	}
	
	
	/***********************************************************************
	 * Define versions of edgeSign, edgeEval with s and t transposed.
	 */
	
    template <typename U, typename V, typename W>
    typename U::value_type transEdgeEval( const U *u, const V *v, const W *w )
	{
		/* Given three vertices u,v,w such that transVertAreLessOrEqual(u,v) && transVertAreLessOrEqual(v,w),
		 * evaluates the t-coord of the edge uw at the s-coord of the vertex v.
		 * Returns v->getS() - (uw)(v->getT()), ie. the signed distance from uw to v.
		 * If uw is vertical (and thus passes thru v), the result is zero.
		 *
		 * The calculation is extremely accurate and stable, even when v
		 * is very close to u or w.	 In particular if we set v->getS() = 0 and
		 * let r be the negated result (this evaluates (uw)(v->getT())), then
		 * r is guaranteed to satisfy MIN(u->getS(),w->getS()) <= r <= MAX(u->getS(),w->getS()).
		 */
        typename U::value_type gapL, gapR;
		
		assert( transVertAreLessOrEqual( u, v ) && transVertAreLessOrEqual( v, w ));
		
		gapL = v->getT() - u->getT();
		gapR = w->getT() - v->getT();
		
		if( gapL + gapR > 0 ) {
			if( gapL < gapR ) {
				return (v->getS() - u->getS()) + (u->getS() - w->getS()) * (gapL / (gapL + gapR));
			} else {
				return (v->getS() - w->getS()) + (w->getS() - u->getS()) * (gapR / (gapL + gapR));
			}
		}
		/* vertical line */
		return 0;
	}
	
    template <typename U, typename V, typename W>
    typename U::value_type transEdgeSign( const U *u, const V *v, const W *w )
	{
		/* Returns a number whose sign matches transEdgeEval(u,v,w) but which
		 * is cheaper to evaluate.	Returns > 0, == 0 , or < 0
		 * as v is above, on, or below the edge uw.
		 */
        typename U::value_type gapL, gapR;
		
		assert( transVertAreLessOrEqual( u, v ) && transVertAreLessOrEqual( v, w ));
		
		gapL = v->getT() - u->getT();
		gapR = w->getT() - v->getT();
		
		if( gapL + gapR > 0 ) {
			return (v->getS() - w->getS()) * gapL + (v->getS() - u->getS()) * gapR;
		}
		/* vertical line */
		return 0;
	}

    template <typename T>
    inline T interpolate(T a, T x, T b, T y)
	/* Given parameters a,x,b,y returns the value (b*x+a*y)/(a+b),
	 * or (x+y)/2 if a==b==0.  It requires that a,b >= 0, and enforces
	 * this in the rare case that one argument is slightly negative.
	 * The implementation is extremely stable numerically.
	 * In particular it guarantees that the result r satisfies
	 * MIN(x,y) <= r <= MAX(x,y), and the results are very accurate
	 * even when a and b differ greatly in magnitude.
	 */
	{
		if (a < 0.f) a = 0.f;
		if (b < 0.f) b = 0.f;
		if (a > b)
		{
			return (y + (x-y) * (b/(a+b)));
		}
		if (b != 0)
		{
			return (x + (y-x) * (a/(a+b)));
		}
		
		return ((x+y) / 2);
	}

    template <typename O1, typename D1, typename O2, typename D2, typename V>
	void edgeIntersect( const O1 *o1, const D1 *d1,
						  const O2 *o2, const D2 *d2,
						  V *v )
	/* Given edges (o1,d1) and (o2,d2), compute their point of intersection.
	 * The computed point is guaranteed to lie in the intersection of the
	 * bounding rectangles defined by each edge.
	 */
	{
        typename V::value_type z1, z2;
		
		/* This is certainly not the most efficient way to find the intersection
		 * of two line segments, but it is very numerically stable.
		 *
		 * Strategy: find the two middle vertices in the vertAreLessOrEqual ordering,
		 * and interpolate the intersection s-value from these.	 Then repeat
		 * using the transVertAreLessOrEqual ordering to find the intersection t-value.
		 */
		
		if( ! vertAreLessOrEqual( o1, d1 )) { std::swap( o1, d1 ); }
		if( ! vertAreLessOrEqual( o2, d2 )) { std::swap( o2, d2 ); }
		if( ! vertAreLessOrEqual( o1, o2 )) { std::swap( o1, o2 ); std::swap( d1, d2 ); }
		
		if( ! vertAreLessOrEqual( o2, d1 )) {
			/* Technically, no intersection -- do our best */
			v->s = (o2->getS() + d1->getS()) / 2;
		} else if( vertAreLessOrEqual( d1, d2 )) {
			/* Interpolate between o2 and d1 */
			z1 = edgeEval( o1, o2, d1 );
			z2 = edgeEval( o2, d1, d2 );
			if( z1+z2 < 0 ) { z1 = -z1; z2 = -z2; }
			v->s = interpolate( z1, o2->getS(), z2, d1->getS() );
		} else {
			/* Interpolate between o2 and d2 */
			z1 = edgeSign( o1, o2, d1 );
			z2 = -edgeSign( o1, d2, d1 );
			if( z1+z2 < 0 ) { z1 = -z1; z2 = -z2; }
			v->s = interpolate( z1, o2->getS(), z2, d2->getS() );
		}
		
		/* Now repeat the process for t */
		
		if( ! transVertAreLessOrEqual( o1, d1 )) { std::swap( o1, d1 ); }
		if( ! transVertAreLessOrEqual( o2, d2 )) { std::swap( o2, d2 ); }
		if( ! transVertAreLessOrEqual( o1, o2 )) { std::swap( o1, o2 ); std::swap( d1, d2 ); }
		
		if( ! transVertAreLessOrEqual( o2, d1 )) {
			/* Technically, no intersection -- do our best */
			v->t = (o2->getT() + d1->getT()) / 2;
		} else if( transVertAreLessOrEqual( d1, d2 )) {
			/* Interpolate between o2 and d1 */
			z1 = transEdgeEval( o1, o2, d1 );
			z2 = transEdgeEval( o2, d1, d2 );
			if( z1+z2 < 0 ) { z1 = -z1; z2 = -z2; }
			v->t = interpolate( z1, o2->getT(), z2, d1->getT() );
		} else {
			/* Interpolate between o2 and d2 */
			z1 = transEdgeSign( o1, o2, d1 );
			z2 = -transEdgeSign( o1, d2, d1 );
			if( z1+z2 < 0 ) { z1 = -z1; z2 = -z2; }
			v->t = interpolate( z1, o2->getT(), z2, d2->getT() );
		}
	}
	
    template <typename Options, typename Allocators>
	static inline typename Options::Coord inCircle( const VertexT<Options, Allocators> *v, const VertexT<Options, Allocators> *v0, const VertexT<Options, Allocators> *v1, const VertexT<Options, Allocators> *v2 ) {
		typename Options::Coord adx, ady, bdx, bdy, cdx, cdy;
		typename Options::Coord abdet, bcdet, cadet;
		typename Options::Coord alift, blift, clift;

		adx = v0->getS() - v->getS();
		ady = v0->getT() - v->getT();
		bdx = v1->getS() - v->getS();
		bdy = v1->getT() - v->getT();
		cdx = v2->getS() - v->getS();
		cdy = v2->getT() - v->getT();

		abdet = adx * bdy - bdx * ady;
		bcdet = bdx * cdy - cdx * bdy;
		cadet = cdx * ady - adx * cdy;

		alift = adx * adx + ady * ady;
		blift = bdx * bdx + bdy * bdy;
		clift = cdx * cdx + cdy * cdy;

		return alift * bcdet + blift * cadet + clift * abdet;
	}
	
	/*
		Returns 1 is edge is locally delaunay
	 */
    template <typename Options, typename Allocators>
	bool edgeIsLocallyDelaunay( const HalfEdgeT<Options, Allocators> *e )
	{
		return inCircle(e->Sym()->Lnext()->Org(), e->Lnext()->Org(), e->Sym()->Lnext()->Lnext()->Org(), e->Sym()->Org()) < 0;
	}
}
