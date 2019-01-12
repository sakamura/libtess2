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

#include <cstddef>
#include <cassert>

#include "mesh.h"
#include "geom.h"
#include "tess.h"
#include "dict.h"
#include "priorityq.h"
#include "bucketalloc.h"
#include "sweep.h"

namespace Tess
{
	/*
	 * Invariants for the Edge Dictionary.
	 * - each pair of adjacent edges e2=Succ(e1) satisfies EdgeLeq(e1,e2)
	 *	 at any valid location of the sweep event
	 * - if EdgeLeq(e2,e1) as well (at any valid sweep event), then e1 and e2
	 *	 share a common endpoint
	 * - for each e, e->Dst has been processed, but not e->Org
	 * - each edge e satisfies vertAreLessOrEqual(e->Dst,event) && vertAreLessOrEqual(event,e->Org)
	 *	 where "event" is the current sweep line event.
	 * - no edge e has zero length
	 *
	 * Invariants for the Mesh (the processed portion).
	 * - the portion of the mesh left of the sweep line is a planar graph,
	 *	 ie. there is *some* way to embed it in the plane
	 * - no processed edge has zero length
	 * - no two processed vertices have identical coordinates
	 * - each "inside" region is monotone, ie. can be broken into two chains
	 *	 of monotonically increasing vertices according to vertAreLessOrEqual(v1,v2)
	 *	 - a non-invariant: these chains may intersect (very slightly)
	 *
	 * Invariants for the Sweep.
	 * - if none of the edges incident to the event vertex have an activeRegion
	 *	 (ie. none of these edges are in the edge dictionary), then the vertex
	 *	 has only right-going edges.
	 * - if an edge is marked "fixUpperEdge" (it is a temporary edge introduced
	 *	 by ConnectRightVertex), then it is the only right-going edge from
	 *	 its associated vertex.	 (This says that these edges exist only
	 *	 when it is necessary.)
	 */
	
#define MAX(x,y)	((x) >= (y) ? (x) : (y))
#define MIN(x,y)	((x) <= (y) ? (x) : (y))
		
    template <typename Options, typename Allocators>
	bool Tesselator<Options, Allocators>::_edgeLeq( ActiveRegion *reg1, ActiveRegion *reg2 ) const
	/*
	 * Both edges must be directed from right to left (this is the canonical
	 * direction for the upper edge of each region).
	 *
	 * The strategy is to evaluate a "t" value for each edge at the
	 * current sweep line position, given by tess->event.  The calculations
	 * are designed to be very stable, but of course they are not perfect.
	 *
	 * Special case: if both edge destinations are at the sweep event,
	 * we sort the edges by slope (they would otherwise compare equally).
	 */
	{
		HalfEdge *e1, *e2;
		typename Options::Coord t1, t2;
		
		e1 = reg1->eUp;
		e2 = reg2->eUp;
		
		if( e1->Dst() == event ) {
			if( e2->Dst() == event ) {
				/* Two edges right of the sweep line which meet at the sweep event.
				 * Sort them by slope.
				 */
				if( vertAreLessOrEqual( e1->Org(), e2->Org() )) {
					return edgeSign( e2->Dst(), e1->Org(), e2->Org() ) <= 0;
				}
				return edgeSign( e1->Dst(), e2->Org(), e1->Org() ) >= 0;
			}
			return edgeSign( e2->Dst(), event, e2->Org() ) <= 0;
		}
		if( e2->Dst() == event ) {
			return edgeSign( e1->Dst(), event, e1->Org() ) >= 0;
		}
		
		/* General case - compute signed distance *from* e1, e2 to event */
		t1 = edgeEval( e1->Dst(), event, e1->Org() );
		t2 = edgeEval( e2->Dst(), event, e2->Org() );
		return (t1 >= t2);
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::deleteRegion( ActiveRegion *reg )
	{
		if( reg->fixUpperEdge ) {
			/* It was created with zero winding number, so it better be
			 * deleted with zero winding number (ie. it better not get merged
			 * with a real edge).
			 */
			assert( reg->eUp->winding() == 0 );
		}
		reg->eUp->resetActiveRegion();
		dict->deleteNode( reg->nodeUp );
		allocators.activeRegionAlloc.destroy(reg);
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::fixUpperEdge( ActiveRegion *reg, HalfEdge *newEdge )
	/*
	 * Replace an upper edge which needs fixing (see ConnectRightVertex).
	 */
	{
		assert( reg->fixUpperEdge );
		mesh->remove( reg->eUp );
		reg->fixUpperEdge = false;
		reg->eUp = newEdge;
		newEdge->setActiveRegion(reg);
	}
	
    template <typename Options, typename Allocators>
	ActiveRegionT<Options, Allocators>* Tesselator<Options, Allocators>::topLeftRegion( ActiveRegion *reg )
	{
		const Vertex *org = reg->eUp->Org();
		HalfEdge *edge;
		
		/* Find the region above the uppermost edge with the same origin */
		do {
			reg = reg->regionAbove( );
		} while( reg->eUp->Org() == org );
		
		/* If the edge above was a temporary edge introduced by ConnectRightVertex,
		 * now is the time to fix it.
		 */
		if( reg->fixUpperEdge ) {
			edge = mesh->connect( reg->regionBelow()->eUp->Sym(), reg->eUp->Lnext() );
			fixUpperEdge( reg, edge );
			reg = reg->regionAbove();
		}
		return reg;
	}
	
    template <typename Options, typename Allocators>
	ActiveRegionT<Options, Allocators>* Tesselator<Options, Allocators>::topRightRegion( ActiveRegion *reg )
	{
		const Vertex *dst = reg->eUp->Dst();
		
		/* Find the region above the uppermost edge with the same destination */
		do {
			reg = reg->regionAbove();
		} while( reg->eUp->Dst() == dst );
		return reg;
	}
	
    template <typename Options, typename Allocators>
	ActiveRegionT<Options, Allocators>* Tesselator<Options, Allocators>::addRegionBelow( ActiveRegion *regAbove, HalfEdge *eNewUp )
	/*
	 * Add a new active region to the sweep line, *somewhere* below "regAbove"
	 * (according to where the new edge belongs in the sweep-line dictionary).
	 * The upper edge of the new region will be "eNewUp".
	 * Winding number and "inside" flag are not updated.
	 */
	{
		ActiveRegion *regNew = allocators.activeRegionAlloc.construct();
		regNew->eUp = eNewUp;
		regNew->nodeUp = dict->insertBefore( regAbove->nodeUp, regNew );
		regNew->fixUpperEdge = false;
		regNew->sentinel = false;
		regNew->dirty = false;
		
		eNewUp->setActiveRegion(regNew);
		return regNew;
	}
	
    template <typename Options, typename Allocators>
	bool Tesselator<Options, Allocators>::isWindingInside( int n ) const
	{
		switch( options.windingRule() ) {
			case TESS_WINDING_ODD:
				return (n & 1);
			case TESS_WINDING_NONZERO:
				return (n != 0);
			case TESS_WINDING_POSITIVE:
				return (n > 0);
			case TESS_WINDING_NEGATIVE:
				return (n < 0);
			case TESS_WINDING_ABS_GEQ_TWO:
				return (n >= 2) || (n <= -2);
			default:
				assert(false);
				return false;
		}
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::computeWinding( ActiveRegion *reg )
	{
		reg->windingNumber = reg->regionAbove()->windingNumber + reg->eUp->winding();
		reg->inside = isWindingInside( reg->windingNumber );
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::finishRegion( ActiveRegion *reg )
	/*
	 * Delete a region from the sweep line.	 This happens when the upper
	 * and lower chains of a region meet (at a vertex on the sweep line).
	 * The "inside" flag is copied to the appropriate mesh face (we could
	 * not do this before -- since the structure of the mesh is always
	 * changing, this face may not have even existed until now).
	 */
	{
		HalfEdge *newUp = reg->eUp;
		Face *f = newUp->Lface();
		
		f->inside = reg->inside;
		f->anEdge = newUp;	 /* optimization for tessMeshTessellateMonoRegion() */
		deleteRegion( reg );
	}
	
	
    template <typename Options, typename Allocators>
	HalfEdgeT<Options, Allocators>* Tesselator<Options, Allocators>::finishLeftRegions( ActiveRegion *regFirst, ActiveRegion *regLast )
	/*
	 * We are given a vertex with one or more left-going edges.	 All affected
	 * edges should be in the edge dictionary.	Starting at regFirst->eUp,
	 * we walk down deleting all regions where both edges have the same
	 * origin vOrg.	 At the same time we copy the "inside" flag from the
	 * active region to the face, since at this point each face will belong
	 * to at most one region (this was not necessarily true until this point
	 * in the sweep).  The walk stops at the region above regLast; if regLast
	 * is nullptr we walk as far as possible.  At the same time we relink the
	 * mesh if necessary, so that the ordering of edges around vOrg is the
	 * same as in the dictionary.
	 */
	{
		ActiveRegion *reg, *regPrev;
		HalfEdge *edge, *ePrev;
		
		regPrev = regFirst;
		ePrev = regFirst->eUp;
		while( regPrev != regLast ) {
			regPrev->fixUpperEdge = false;	/* placement was OK */
			reg = regPrev->regionBelow( );
			edge = reg->eUp;
			if( edge->Org() != ePrev->Org() ) {
				if( ! reg->fixUpperEdge ) {
					/* Remove the last left-going edge.	 Even though there are no further
					 * edges in the dictionary with this origin, there may be further
					 * such edges in the mesh (if we are adding left edges to a vertex
					 * that has already been processed).  Thus it is important to call
					 * FinishRegion rather than just DeleteRegion.
					 */
					finishRegion( regPrev );
					break;
				}
				/* If the edge below was a temporary edge introduced by
				 * ConnectRightVertex, now is the time to fix it.
				 */
				edge = mesh->connect( ePrev->Lprev(), edge->Sym() );
				fixUpperEdge( reg, edge );
			}
			
			/* Relink edges so that ePrev->Onext == edge */
			if( ePrev->Onext() != edge ) {
				mesh->splice( edge->Oprev(), edge );
				mesh->splice( ePrev, edge );
			}
			finishRegion( regPrev );	/* may change reg->eUp */
			ePrev = reg->eUp;
			regPrev = reg;
		}
		return ePrev;
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::addRightEdges( ActiveRegion *regUp, HalfEdge *eFirst, HalfEdge *eLast, HalfEdge *eTopLeft, bool cleanUp )
	/*
	 * Purpose: insert right-going edges into the edge dictionary, and update
	 * winding numbers and mesh connectivity appropriately.	 All right-going
	 * edges share a common origin vOrg.  Edges are inserted CCW starting at
	 * eFirst; the last edge inserted is eLast->Oprev.	If vOrg has any
	 * left-going edges already processed, then eTopLeft must be the edge
	 * such that an imaginary upward vertical segment from vOrg would be
	 * contained between eTopLeft->Oprev and eTopLeft; otherwise eTopLeft
	 * should be nullptr.
	 */
	{
		ActiveRegion *reg, *regPrev;
		HalfEdge *edge, *ePrev;
		int firstTime = true;
		
		/* Insert the new right-going edges in the dictionary */
		edge = eFirst;
		do {
			assert( vertAreLessOrEqual( edge->Org(), edge->Dst() ));
			addRegionBelow( regUp, edge->Sym() );
			edge = edge->Onext();
		} while ( edge != eLast );
		
		/* Walk *all* right-going edges from edge->Org, in the dictionary order,
		 * updating the winding numbers of each region, and re-linking the mesh
		 * edges to match the dictionary ordering (if necessary).
		 */
		if( eTopLeft == nullptr ) {
			eTopLeft = regUp->regionBelow()->eUp->Rprev();
		}
		regPrev = regUp;
		ePrev = eTopLeft;
		for( ;; ) {
			reg = regPrev->regionBelow();
			edge = reg->eUp->Sym();
			if( edge->Org() != ePrev->Org() ) break;
			
			if( edge->Onext() != ePrev ) {
				/* Unlink edge from its current position, and relink below ePrev */
				mesh->splice( edge->Oprev(), edge );
				mesh->splice( ePrev->Oprev(), edge );
			}
			/* Compute the winding number and "inside" flag for the new regions */
			reg->windingNumber = regPrev->windingNumber - edge->winding();
			reg->inside = isWindingInside( reg->windingNumber );
			
			/* Check for two outgoing edges with same slope -- process these
			 * before any intersection tests (see example in tessComputeInterior).
			 */
			regPrev->dirty = true;
			if( ! firstTime && checkForRightSplice( regPrev )) {
				edge->addWinding( ePrev );
				deleteRegion(regPrev);
				mesh->remove( ePrev );
			}
			firstTime = false;
			regPrev = reg;
			ePrev = edge;
		}
		regPrev->dirty = true;
		assert( regPrev->windingNumber - edge->winding() == reg->windingNumber );
		
		if( cleanUp ) {
			/* Check for intersections between newly adjacent edges. */
			walkDirtyRegions( regPrev );
		}
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::spliceMergeVertices( HalfEdge *e1, HalfEdge *e2 )
	/*
	 * Two vertices with idential coordinates are combined into one.
	 * e1->Org is kept, while e2->Org is discarded.
	 */
	{
		mesh->splice( e1, e2 );
	}
	
    template <typename Options, typename Allocators>
	bool Tesselator<Options, Allocators>::checkForRightSplice( ActiveRegion *regUp )
	/*
	 * Check the upper and lower edge of "regUp", to make sure that the
	 * eUp->Org is above eLo, or eLo->Org is below eUp (depending on which
	 * origin is leftmost).
	 *
	 * The main purpose is to splice right-going edges with the same
	 * dest vertex and nearly identical slopes (ie. we can't distinguish
	 * the slopes numerically).	 However the splicing can also help us
	 * to recover from numerical errors.  For example, suppose at one
	 * point we checked eUp and eLo, and decided that eUp->Org is barely
	 * above eLo.  Then later, we split eLo into two edges (eg. from
	 * a splice operation like this one).  This can change the result of
	 * our test so that now eUp->Org is incident to eLo, or barely below it.
	 * We must correct this condition to maintain the dictionary invariants.
	 *
	 * One possibility is to check these edges for intersection again
	 * (ie. CheckForIntersect).	 This is what we do if possible.  However
	 * CheckForIntersect requires that tess->event lies between eUp and eLo,
	 * so that it has something to fall back on when the intersection
	 * calculation gives us an unusable answer.	 So, for those cases where
	 * we can't check for intersection, this routine fixes the problem
	 * by just splicing the offending vertex into the other edge.
	 * This is a guaranteed solution, no matter how degenerate things get.
	 * Basically this is a combinatorial solution to a numerical problem.
	 */
	{
		ActiveRegion *regLo = regUp->regionBelow();
		HalfEdge *eUp = regUp->eUp;
		HalfEdge *eLo = regLo->eUp;
		
		if( vertAreLessOrEqual( eUp->Org(), eLo->Org() )) {
			if( edgeSign( eLo->Dst(), eUp->Org(), eLo->Org() ) > 0 ) return false;
			
			/* eUp->Org appears to be below eLo */
			if( ! vertAreEqual( eUp->Org(), eLo->Org() )) {
				/* splice eUp->Org into eLo */
				mesh->splitEdge( eLo->Sym() );
				mesh->splice( eUp, eLo->Oprev() );
				regUp->dirty = regLo->dirty = true;
				
			} else if( eUp->Org() != eLo->Org() ) {
				/* merge the two vertices, discarding eUp->Org */
				pq->remove( eUp->Org()->pqHandle );
				spliceMergeVertices( eLo->Oprev(), eUp );
			}
		} else {
		if( edgeSign( eUp->Dst(), eLo->Org(), eUp->Org() ) <= 0 ) return false;
			
			/* eLo->Org appears to be above eUp, so splice eLo->Org into eUp */
			regUp->regionAbove()->dirty = regUp->dirty = true;
			mesh->splitEdge( eUp->Sym() );
			mesh->splice( eLo->Oprev(), eUp );
		}
		return true;
	}
	
    template <typename Options, typename Allocators>
	bool Tesselator<Options, Allocators>::checkForLeftSplice( ActiveRegion *regUp )
	/*
	 * Check the upper and lower edge of "regUp", to make sure that the
	 * eUp->Dst is above eLo, or eLo->Dst is below eUp (depending on which
	 * destination is rightmost).
	 *
	 * Theoretically, this should always be true.  However, splitting an edge
	 * into two pieces can change the results of previous tests.  For example,
	 * suppose at one point we checked eUp and eLo, and decided that eUp->Dst
	 * is barely above eLo.	 Then later, we split eLo into two edges (eg. from
	 * a splice operation like this one).  This can change the result of
	 * the test so that now eUp->Dst is incident to eLo, or barely below it.
	 * We must correct this condition to maintain the dictionary invariants
	 * (otherwise new edges might get inserted in the wrong place in the
	 * dictionary, and bad stuff will happen).
	 *
	 * We fix the problem by just splicing the offending vertex into the
	 * other edge.
	 */
	{
		ActiveRegion *regLo = regUp->regionBelow();
		HalfEdge *eUp = regUp->eUp;
		HalfEdge *eLo = regLo->eUp;
		HalfEdge *edge;
		
		assert( ! vertAreEqual( eUp->Dst(), eLo->Dst() ));
		
		if( vertAreLessOrEqual( eUp->Dst(), eLo->Dst() )) {
			if( edgeSign( eUp->Dst(), eLo->Dst(), eUp->Org() ) < 0 ) return false;
			
			/* eLo->Dst is above eUp, so splice eLo->Dst into eUp */
			regUp->regionAbove()->dirty = regUp->dirty = true;
			edge = mesh->splitEdge( eUp );
			mesh->splice( eLo->Sym(), edge );
			edge->Lface()->inside = regUp->inside;
		} else {
			if( edgeSign( eLo->Dst(), eUp->Dst(), eLo->Org() ) > 0 ) return false;
			
			/* eUp->Dst is below eLo, so splice eUp->Dst into eLo */
			regUp->dirty = regLo->dirty = true;
			edge = mesh->splitEdge( eLo );
			mesh->splice( eUp->Lnext(), eLo->Sym() );
			edge->Rface()->inside = regUp->inside;
		}
		return true;
	}
	
	
    template <typename Options, typename Allocators>
	bool Tesselator<Options, Allocators>::checkForIntersect( ActiveRegion *regUp )
	/*
	 * Check the upper and lower edges of the given region to see if
	 * they intersect.	If so, create the intersection and add it
	 * to the data structures.
	 *
	 * Returns true if adding the new intersection resulted in a recursive
	 * call to AddRightEdges(); in this case all "dirty" regions have been
	 * checked for intersections, and possibly regUp has been deleted.
	 */
	{
		ActiveRegion *regLo = regUp->regionBelow();
		HalfEdge *eUp = regUp->eUp;
		HalfEdge *eLo = regLo->eUp;
		Vertex *orgUp = eUp->Org();
		Vertex *orgLo = eLo->Org();
		Vertex *dstUp = eUp->Dst();
		Vertex *dstLo = eLo->Dst();
		Coord tMinUp, tMaxLo;
		Vertex *orgMin;
		HalfEdge *edge;
        InternalVec isect;
        
		assert( ! vertAreEqual( dstLo, dstUp ));
		assert( edgeSign( dstUp, event, orgUp ) <= 0 );
		assert( edgeSign( dstLo, event, orgLo ) >= 0 );
		assert( orgUp != event && orgLo != event );
		assert( ! regUp->fixUpperEdge && ! regLo->fixUpperEdge );
		
		if( orgUp == orgLo ) return false;	/* right endpoints are the same */
		
		tMinUp = MIN( orgUp->getT(), dstUp->getT() );
		tMaxLo = MAX( orgLo->getT(), dstLo->getT() );
		if( tMinUp > tMaxLo ) return false;	/* t ranges do not overlap */
		
		if( vertAreLessOrEqual( orgUp, orgLo )) {
			if( edgeSign( dstLo, orgUp, orgLo ) > 0 ) return false;
		} else {
			if( edgeSign( dstUp, orgLo, orgUp ) < 0 ) return false;
		}
		
		/* At this point the edges intersect, at least marginally */
		edgeIntersect( dstUp, orgUp, dstLo, orgLo, &isect );
		/* The following properties are guaranteed: */
		assert( MIN( orgUp->getT(), dstUp->getT() ) <= isect.getT() );
		assert( isect.getT() <= MAX( orgLo->getT(), dstLo->getT() ));
		assert( MIN( dstLo->getS(), dstUp->getS() ) <= isect.getS() );
		assert( isect.getS() <= MAX( orgLo->getS(), orgUp->getS() ));
		
		if( vertAreLessOrEqual( &isect, event )) {
			/* The intersection point lies slightly to the left of the sweep line,
			 * so move it until it''s slightly to the right of the sweep line.
			 * (If we had perfect numerical precision, this would never happen
			 * in the first place).	 The easiest and safest thing to do is
			 * replace the intersection by tess->event.
			 */
			isect.s = event->getS();
			isect.t = event->getT();
		}
		/* Similarly, if the computed intersection lies to the right of the
		 * rightmost origin (which should rarely happen), it can cause
		 * unbelievable inefficiency on sufficiently degenerate inputs.
		 * (If you have the test program, try running test54.d with the
		 * "X zoom" option turned on).
		 */
		orgMin = vertAreLessOrEqual( orgUp, orgLo ) ? orgUp : orgLo;
		if( vertAreLessOrEqual( orgMin, &isect )) {
			isect.s = orgMin->getS();
			isect.t = orgMin->getT();
		}
		
		if( vertAreEqual( &isect, orgUp ) || vertAreEqual( &isect, orgLo )) {
			/* Easy case -- intersection at one of the right endpoints */
			checkForRightSplice( regUp );
			return false;
		}
		
		if(	   (! vertAreEqual( dstUp, event )
				&& edgeSign( dstUp, event, &isect ) >= 0)
		   || (! vertAreEqual( dstLo, event )
			   && edgeSign( dstLo, event, &isect ) <= 0 ))
		{
			/* Very unusual -- the new upper or lower edge would pass on the
			 * wrong side of the sweep event, or through it.  This can happen
			 * due to very small numerical errors in the intersection calculation.
			 */
			if( dstLo == event ) {
				/* splice dstLo into eUp, and process the new region(s) */
				mesh->splitEdge( eUp->Sym() );
				mesh->splice( eLo->Sym(), eUp );
				regUp = topLeftRegion( regUp );
				eUp = regUp->regionBelow()->eUp;
				finishLeftRegions( regUp->regionBelow(), regLo );
				addRightEdges( regUp, eUp->Oprev(), eUp, eUp, true );
				return true;
			}
			if( dstUp == event ) {
				/* splice dstUp into eLo, and process the new region(s) */
				mesh->splitEdge( eLo->Sym() );
				mesh->splice( eUp->Lnext(), eLo->Oprev() );
				regLo = regUp;
				regUp = topRightRegion( regUp );
				edge = regUp->regionBelow()->eUp->Rprev();
				regLo->eUp = eLo->Oprev();
				eLo = finishLeftRegions( regLo, nullptr );
				addRightEdges( regUp, eLo->Onext(), eUp->Rprev(), edge, true );
				return true;
			}
			/* Special case: called from ConnectRightVertex.  If either
			 * edge passes on the wrong side of tess->event, split it
			 * (and wait for ConnectRightVertex to splice it appropriately).
			 */
			if( edgeSign( dstUp, event, &isect ) >= 0 ) {
				regUp->regionAbove()->dirty = regUp->dirty = true;
				mesh->splitEdge( eUp->Sym() );
                eUp->Org()->sVec = event->sVec;
			}
			if( edgeSign( dstLo, event, &isect ) <= 0 ) {
				regUp->dirty = regLo->dirty = true;
				mesh->splitEdge( eLo->Sym() );
				eLo->Org()->sVec = event->sVec;
			}
			/* leave the rest for ConnectRightVertex */
			return false;
		}
		
		/* General case -- split both edges, splice into new vertex.
		 * When we do the splice operation, the order of the arguments is
		 * arbitrary as far as correctness goes.  However, when the operation
		 * creates a new face, the work done is proportional to the size of
		 * the new face.  We expect the faces in the processed part of
		 * the mesh (ie. eUp->Lface) to be smaller than the faces in the
		 * unprocessed original contours (which will be eLo->Oprev->Lface).
		 */
		mesh->splitEdge( eUp->Sym() );
		mesh->splitEdge( eLo->Sym() );
		mesh->splice( eLo->Oprev(), eUp );
        auto result = options.addPoint(isect);
        eUp->Org()->sVec = result.first;
        eUp->Org()->idx = result.second;
		eUp->Org()->pqHandle = pq->insert( eUp->Org() );
        assert(eUp->Org()->pqHandle != PriorityQ::INV_HANDLE);
		regUp->regionAbove()->dirty = regUp->dirty = regLo->dirty = true;
		return false;
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::walkDirtyRegions( ActiveRegion *regUp )
	/*
	 * When the upper or lower edge of any region changes, the region is
	 * marked "dirty".	This routine walks through all the dirty regions
	 * and makes sure that the dictionary invariants are satisfied
	 * (see the comments at the beginning of this file).  Of course
	 * new dirty regions can be created as we make changes to restore
	 * the invariants.
	 */
	{
		ActiveRegion *regLo = regUp->regionBelow();
		HalfEdge *eUp, *eLo;
		
		for( ;; ) {
			/* Find the lowest dirty region (we walk from the bottom up). */
			while( regLo->dirty ) {
				regUp = regLo;
				regLo = regLo->regionBelow();
			}
			if( ! regUp->dirty ) {
				regLo = regUp;
				regUp = regUp->regionAbove();
				if( regUp == nullptr || ! regUp->dirty ) {
					/* We've walked all the dirty regions */
					return;
				}
			}
			regUp->dirty = false;
			eUp = regUp->eUp;
			eLo = regLo->eUp;
			
			if( eUp->Dst() != eLo->Dst() ) {
				/* Check that the edge ordering is obeyed at the Dst vertices. */
				if( checkForLeftSplice( regUp )) {
					
					/* If the upper or lower edge was marked fixUpperEdge, then
					 * we no longer need it (since these edges are needed only for
					 * vertices which otherwise have no right-going edges).
					 */
					if( regLo->fixUpperEdge ) {
						deleteRegion( regLo );
						mesh->remove( eLo );
						regLo = regUp->regionBelow();
						eLo = regLo->eUp;
					} else if( regUp->fixUpperEdge ) {
						deleteRegion( regUp );
						mesh->remove( eUp );
						regUp = regLo->regionAbove();
						eUp = regUp->eUp;
					}
				}
			}
			if( eUp->Org() != eLo->Org() ) {
				if(	   eUp->Dst() != eLo->Dst()
				   && ! regUp->fixUpperEdge && ! regLo->fixUpperEdge
				   && (eUp->Dst() == event || eLo->Dst() == event) )
				{
					/* When all else fails in CheckForIntersect(), it uses tess->event
					 * as the intersection location.  To make this possible, it requires
					 * that tess->event lie between the upper and lower edges, and also
					 * that neither of these is marked fixUpperEdge (since in the worst
					 * case it might splice one of these edges into tess->event, and
					 * violate the invariant that fixable edges are the only right-going
					 * edge from their associated vertex).
					 */
					if( checkForIntersect( regUp )) {
						/* WalkDirtyRegions() was called recursively; we're done */
						return;
					}
				} else {
					/* Even though we can't use CheckForIntersect(), the Org vertices
					 * may violate the dictionary edge ordering.  Check and correct this.
					 */
					checkForRightSplice( regUp );
				}
			}
			if( eUp->Org() == eLo->Org() && eUp->Dst() == eLo->Dst() ) {
				/* A degenerate loop consisting of only two edges -- delete it. */
				eLo->addWinding( eUp );
				deleteRegion( regUp );
				mesh->remove( eUp );
				regUp = regLo->regionAbove();
			}
		}
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::connectRightVertex( ActiveRegion *regUp, HalfEdge *eBottomLeft )
	/*
	 * Purpose: connect a "right" vertex vEvent (one where all edges go left)
	 * to the unprocessed portion of the mesh.	Since there are no right-going
	 * edges, two regions (one above vEvent and one below) are being merged
	 * into one.  "regUp" is the upper of these two regions.
	 *
	 * There are two reasons for doing this (adding a right-going edge):
	 *	- if the two regions being merged are "inside", we must add an edge
	 *	  to keep them separated (the combined region would not be monotone).
	 *	- in any case, we must leave some record of vEvent in the dictionary,
	 *	  so that we can merge vEvent with features that we have not seen yet.
	 *	  For example, maybe there is a vertical edge which passes just to
	 *	  the right of vEvent; we would like to splice vEvent into this edge.
	 *
	 * However, we don't want to connect vEvent to just any vertex.	 We don''t
	 * want the new edge to cross any other edges; otherwise we will create
	 * intersection vertices even when the input data had no self-intersections.
	 * (This is a bad thing; if the user's input data has no intersections,
	 * we don't want to generate any false intersections ourselves.)
	 *
	 * Our eventual goal is to connect vEvent to the leftmost unprocessed
	 * vertex of the combined region (the union of regUp and regLo).
	 * But because of unseen vertices with all right-going edges, and also
	 * new vertices which may be created by edge intersections, we don''t
	 * know where that leftmost unprocessed vertex is.	In the meantime, we
	 * connect vEvent to the closest vertex of either chain, and mark the region
	 * as "fixUpperEdge".  This flag says to delete and reconnect this edge
	 * to the next processed vertex on the boundary of the combined region.
	 * Quite possibly the vertex we connected to will turn out to be the
	 * closest one, in which case we won''t need to make any changes.
	 */
	{
		HalfEdge *eNew;
		HalfEdge *eTopLeft = eBottomLeft->Onext();
		ActiveRegion *regLo = regUp->regionBelow();
		HalfEdge *eUp = regUp->eUp;
		HalfEdge *eLo = regLo->eUp;
		int degenerate = false;
		
		if( eUp->Dst() != eLo->Dst() ) {
			checkForIntersect( regUp );
		}
		
		/* Possible new degeneracies: upper or lower edge of regUp may pass
		 * through vEvent, or may coincide with new intersection vertex
		 */
		if( vertAreEqual( eUp->Org(), event )) {
			mesh->splice( eTopLeft->Oprev(), eUp );
			regUp = topLeftRegion( regUp );
			eTopLeft = regUp->regionBelow()->eUp;
			finishLeftRegions( regUp->regionBelow(), regLo );
			degenerate = true;
		}
		if( vertAreEqual( eLo->Org(), event )) {
			mesh->splice( eBottomLeft, eLo->Oprev() );
			eBottomLeft = finishLeftRegions( regLo, nullptr );
			degenerate = true;
		}
		if( degenerate ) {
			addRightEdges( regUp, eBottomLeft->Onext(), eTopLeft, eTopLeft, true );
			return;
		}
		
		/* Non-degenerate situation -- need to add a temporary, fixable edge.
		 * Connect to the closer of eLo->Org, eUp->Org.
		 */
		if( vertAreLessOrEqual( eLo->Org(), eUp->Org() )) {
			eNew = eLo->Oprev();
		} else {
			eNew = eUp;
		}
		eNew = mesh->connect( eBottomLeft->Lprev(), eNew );
		
		/* Prevent cleanup, otherwise eNew might disappear before we've even
		 * had a chance to mark it as a temporary edge.
		 */
		addRightEdges( regUp, eNew, eNew->Onext(), eNew->Onext(), false );
		eNew->Sym()->activeRegion()->fixUpperEdge = true;
		walkDirtyRegions( regUp );
	}
	
	/* Because vertices at exactly the same location are merged together
	 * before we process the sweep event, some degenerate cases can't occur.
	 * However if someone eventually makes the modifications required to
	 * merge features which are close together, the cases below marked
	 * TOLERANCE_NONZERO will be useful.  They were debugged before the
	 * code to merge identical vertices in the main loop was added.
	 */
#define TOLERANCE_NONZERO	false
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::connectLeftDegenerate( ActiveRegion *regUp, Vertex *vEvent )
	/*
	 * The event vertex lies exacty on an already-processed edge or vertex.
	 * Adding the new vertex involves splicing it into the already-processed
	 * part of the mesh.
	 */
	{
		HalfEdge *edge, *eTopLeft, *eTopRight, *eLast;
		ActiveRegion *reg;
		
		edge = regUp->eUp;
		if( vertAreEqual( edge->Org(), vEvent )) {
			/* edge->Org is an unprocessed vertex - just combine them, and wait
			 * for edge->Org to be pulled from the queue
			 */
			assert( TOLERANCE_NONZERO );
			spliceMergeVertices( edge, vEvent->anEdge );
			return;
		}
		
		if( ! vertAreEqual( edge->Dst(), vEvent )) {
			/* General case -- splice vEvent into edge edge which passes through it */
			mesh->splitEdge( edge->Sym() );
			if( regUp->fixUpperEdge ) {
				/* This edge was fixable -- delete unused portion of original edge */
				mesh->remove( edge->Onext() );
				regUp->fixUpperEdge = false;
			}
			mesh->splice( vEvent->anEdge, edge );
			sweepEvent( vEvent );	/* recurse */
			return;
		}
		
		/* vEvent coincides with edge->Dst, which has already been processed.
		 * splice in the additional right-going edges.
		 */
		assert( TOLERANCE_NONZERO );
		regUp = topRightRegion( regUp );
		reg = regUp->regionBelow();
		eTopRight = reg->eUp->Sym();
		eTopLeft = eLast = eTopRight->Onext();
		if( reg->fixUpperEdge ) {
			/* Here edge->Dst has only a single fixable edge going right.
			 * We can delete it since now we have some real right-going edges.
			 */
			assert( eTopLeft != eTopRight );   /* there are some left edges too */
			deleteRegion( reg );
			mesh->remove( eTopRight );
			eTopRight = eTopLeft->Oprev();
		}
		mesh->splice( vEvent->anEdge, eTopRight );
		if( ! edgeGoesLeft( eTopLeft )) {
			/* edge->Dst had no left-going edges -- indicate this to AddRightEdges() */
			eTopLeft = nullptr;
		}
		addRightEdges( regUp, eTopRight->Onext(), eLast, eTopLeft, true );
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::connectLeftVertex( Vertex *vEvent )
	/*
	 * Purpose: connect a "left" vertex (one where both edges go right)
	 * to the processed portion of the mesh.  Let R be the active region
	 * containing vEvent, and let U and L be the upper and lower edge
	 * chains of R.	 There are two possibilities:
	 *
	 * - the normal case: split R into two regions, by connecting vEvent to
	 *	 the rightmost vertex of U or L lying to the left of the sweep line
	 *
	 * - the degenerate case: if vEvent is close enough to U or L, we
	 *	 merge vEvent into that edge chain.	 The subcases are:
	 *	- merging with the rightmost vertex of U or L
	 *	- merging with the active edge of U or L
	 *	- merging with an already-processed portion of U or L
	 */
	{
		ActiveRegion *regUp, *regLo, *reg;
		HalfEdge *eUp, *eLo, *eNew;
		ActiveRegion tmp;
		
		/* assert( vEvent->anEdge->Onext->Onext == vEvent->anEdge ); */
		
		/* Get a pointer to the active region containing vEvent */
		tmp.eUp = vEvent->anEdge->Sym();
		/* __GL_DICTLISTKEY */ /* tessDictListSearch */
		regUp = (ActiveRegion *)dict->search( &tmp )->key;
		regLo = regUp->regionBelow();
		if( !regLo ) {
			// This may happen if the input polygon is coplanar.
			return;
		}
		eUp = regUp->eUp;
		eLo = regLo->eUp;
		
		/* Try merging with U or L first */
		if( edgeSign( eUp->Dst(), vEvent, eUp->Org() ) == 0 ) {
			connectLeftDegenerate( regUp, vEvent );
			return;
		}
		
		/* Connect vEvent to rightmost processed vertex of either chain.
		 * e->Dst is the vertex that we will connect to vEvent.
		 */
		reg = vertAreLessOrEqual( eLo->Dst(), eUp->Dst() ) ? regUp : regLo;
		
		if( regUp->inside || reg->fixUpperEdge) {
			if( reg == regUp ) {
				eNew = mesh->connect( vEvent->anEdge->Sym(), eUp->Lnext() );
			} else {
				HalfEdge *tempHalfEdge = mesh->connect( eLo->Dnext(), vEvent->anEdge);
				
				eNew = tempHalfEdge->Sym();
			}
			if( reg->fixUpperEdge ) {
				fixUpperEdge( reg, eNew );
			} else {
				computeWinding( addRegionBelow( regUp, eNew ));
			}
			sweepEvent( vEvent );
		} else {
			/* The new vertex is in a region which does not belong to the polygon.
			 * We don''t need to connect this vertex to the rest of the mesh.
			 */
			addRightEdges( regUp, vEvent->anEdge, vEvent->anEdge, nullptr, true );
		}
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::sweepEvent( Vertex *vEvent )
	/*
	 * Does everything necessary when the sweep line crosses a vertex.
	 * Updates the mesh and the edge dictionary.
	 */
	{
		ActiveRegion *regUp, *reg;
		HalfEdge *edge, *eTopLeft, *eBottomLeft;
		
		event = vEvent;		/* for access in EdgeLeq() */
		
		/* Check if this vertex is the right endpoint of an edge that is
		 * already in the dictionary.  In this case we don't need to waste
		 * time searching for the location to insert new edges.
		 */
		edge = vEvent->anEdge;
		while( edge->activeRegion() == nullptr ) {
			edge = edge->Onext();
			if( edge == vEvent->anEdge ) {
				/* All edges go right -- not incident to any processed edges */
				connectLeftVertex( vEvent );
				return;
			}
		}
		
		/* Processing consists of two phases: first we "finish" all the
		 * active regions where both the upper and lower edges terminate
		 * at vEvent (ie. vEvent is closing off these regions).
		 * We mark these faces "inside" or "outside" the polygon according
		 * to their winding number, and delete the edges from the dictionary.
		 * This takes care of all the left-going edges from vEvent.
		 */
		regUp = topLeftRegion( edge->activeRegion() );
		reg = regUp->regionBelow();
		eTopLeft = reg->eUp;
		eBottomLeft = finishLeftRegions( reg, nullptr );
		
		/* Next we process all the right-going edges from vEvent.  This
		 * involves adding the edges to the dictionary, and creating the
		 * associated "active regions" which record information about the
		 * regions between adjacent dictionary edges.
		 */
		if( eBottomLeft->Onext() == eTopLeft ) {
			/* No right-going edges -- add a temporary "fixable" edge */
			connectRightVertex( regUp, eBottomLeft );
		} else {
			addRightEdges( regUp, eBottomLeft->Onext(), eTopLeft, eTopLeft, true );
		}
	}
	
	
	/* Make the sentinel coordinates big enough that they will never be
	 * merged with real input features.
	 */
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::addSentinel( typename Options::Coord smin, typename Options::Coord smax, typename Options::Coord t )
	/*
	 * We add two sentinel edges above and below all other edges,
	 * to avoid special cases at the top and bottom.
	 */
	{
		HalfEdge *edge;
		ActiveRegion *reg = allocators.activeRegionAlloc.construct();
		
		edge = mesh->makeEdge();
		
        edge->Org()->sVec = options.addSentinelPoint(smax, t);
        edge->Dst()->sVec = options.addSentinelPoint(smin, t);
		event = edge->Dst();		/* initialize it */
		
		reg->eUp = edge;
		reg->windingNumber = 0;
		reg->inside = false;
		reg->fixUpperEdge = false;
		reg->sentinel = true;
		reg->dirty = false;
		reg->nodeUp = dict->insert( reg );
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::initEdgeDict( )
	/*
	 * We maintain an ordering of edge intersections with the sweep line.
	 * This order is maintained in a dynamic dictionary.
	 */
	{
		typename Options::Coord w, h;
		typename Options::Coord smin, smax, tmin, tmax;
		
		dict = new Dict( this, (LeqFunc)edgeLeq );
		
		/* If the bbox is empty, ensure that sentinels are not coincident by slightly enlarging it. */
		w = (bmax[0] - bmin[0]) + 0.01f;
		h = (bmax[1] - bmin[1]) + 0.01f;
		
		smin = bmin[0] - w;
		smax = bmax[0] + w;
		tmin = bmin[1] - h;
		tmax = bmax[1] + h;
		
		addSentinel( smin, smax, tmin );
		addSentinel( smin, smax, tmax );
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::doneEdgeDict( )
	{
		ActiveRegion *reg;
        int fixedEdges = 0;
		
		while( (reg = (ActiveRegion *)dict->min()->key) != nullptr ) {
			/*
			 * At the end of all processing, the dictionary should contain
			 * only the two sentinel edges, plus at most one "fixable" edge
			 * created by ConnectRightVertex().
			 */
			if( ! reg->sentinel ) {
				assert( reg->fixUpperEdge );
                ++fixedEdges;
				assert( fixedEdges == 1 );
			}
			assert( reg->windingNumber == 0 );
			deleteRegion( reg );
			/*	  tessMeshDelete( reg->eUp );*/
		}
		delete dict;
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::removeDegenerateEdges()
	/*
	 * Remove zero-length edges, and contours with fewer than 3 vertices.
	 */
	{
		HalfEdge *edge, *eNext, *eLnext;
		HalfEdge *eHead = mesh->eEnd();
		
		/*LINTED*/
		for( edge = eHead->next(); edge != eHead; edge = eNext ) {
			eNext = edge->next();
			eLnext = edge->Lnext();
			
			if( vertAreEqual( edge->Org(), edge->Dst() ) && edge->Lnext()->Lnext() != edge ) {
				/* Zero-length edge, contour has at least 3 edges */
				
				spliceMergeVertices( eLnext, edge );	/* deletes edge->Org */
				mesh->remove( edge );
				edge = eLnext;
				eLnext = edge->Lnext();
			}
			if( eLnext->Lnext() == edge ) {
				/* Degenerate contour (one or two edges) */
				
				if( eLnext != edge ) {
					if( eLnext == eNext || eLnext == eNext->Sym() ) { eNext = eNext->next(); }
					mesh->remove( eLnext );
				}
				if( edge == eNext || edge == eNext->Sym() ) { eNext = eNext->next(); }
				mesh->remove( edge );
			}
		}
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::initPriorityQ( )
	/*
	 * Insert all vertices into the priority queue which determines the
	 * order in which vertices cross the sweep line.
	 */
	{
		Vertex *v, *vHead;
		int vertexCountQ = 0;
		
		vHead = mesh->vEnd();
		for( v = vHead->next; v != vHead; v = v->next ) {
			vertexCountQ++;
		}
		/* Make sure there is enough space for sentinels. */
		vertexCountQ += 8;
		
        pq = new PriorityQ( this, vertexCountQ, (typename PriorityQ::LeqFunc)vertAreLessOrEqual<Vertex, Vertex> );
		
		vHead = mesh->vEnd();
		for( v = vHead->next; v != vHead; v = v->next ) {
			v->pqHandle = pq->insert( v );
		}
		pq->init( );
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::donePriorityQ( )
	{
        delete pq; pq = nullptr;
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::removeDegenerateFaces( )
	/*
	 * Delete any degenerate faces with only two edges.	 WalkDirtyRegions()
	 * will catch almost all of these, but it won't catch degenerate faces
	 * produced by splice operations on already-processed edges.
	 * The two places this can happen are in FinishLeftRegions(), when
	 * we splice in a "temporary" edge produced by ConnectRightVertex(),
	 * and in CheckForLeftSplice(), where we splice already-processed
	 * edges to ensure that our dictionary invariants are not violated
	 * by numerical errors.
	 *
	 * In both these cases it is *very* dangerous to delete the offending
	 * edge at the time, since one of the routines further up the stack
	 * will sometimes be keeping a pointer to that edge.
	 */
	{
		Face *f, *fNext;
		HalfEdge *edge;
		
		/*LINTED*/
		for( f = mesh->fBegin(); f != mesh->fEnd(); f = fNext ) {
			fNext = f->next;
			edge = f->anEdge;
			assert( edge->Lnext() != edge );
			
			if( edge->Lnext()->Lnext() == edge ) {
				/* A face with only two edges */
				edge->Onext()->addWinding( edge );
				mesh->remove( edge );
			}
		}
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::computeInterior( )
	/*
	 * tessComputeInterior( tess ) computes the planar arrangement specified
	 * by the given contours, and further subdivides this arrangement
	 * into regions.  Each region is marked "inside" if it belongs
	 * to the polygon, according to the rule given by tess->windingRule.
	 * Each interior region is guaranteed be monotone.
	 */
	{
		Vertex *v, *vNext;
		
		/* Each vertex defines an event for our sweep line.	 Start by inserting
		 * all the vertices in a priority queue.  Events are processed in
		 * lexicographic order, ie.
		 *
		 *	e1 < e2	 iff  e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y)
		 */
		removeDegenerateEdges();
		initPriorityQ( );
		initEdgeDict( );
		
		while( (v = pq->extractMin( )) != nullptr ) {
			for( ;; ) {
				vNext = pq->minimum( );
				if( vNext == nullptr || ! vertAreEqual( vNext, v )) break;
				
				/* Merge together all vertices at exactly the same location.
				 * This is more efficient than processing them one at a time,
				 * simplifies the code (see ConnectLeftDegenerate), and is also
				 * important for correct handling of certain degenerate cases.
				 * For example, suppose there are two identical edges A and B
				 * that belong to different contours (so without this code they would
				 * be processed by separate sweep events).	Suppose another edge C
				 * crosses A and B from above.	When A is processed, we split it
				 * at its intersection point with C.  However this also splits C,
				 * so when we insert B we may compute a slightly different
				 * intersection point.	This might leave two edges with a small
				 * gap between them.  This kind of error is especially obvious
				 * when using boundary extraction (TESS_BOUNDARY_ONLY).
				 */
				vNext = pq->extractMin( );
				spliceMergeVertices( v->anEdge, vNext->anEdge );
			}
			sweepEvent( v );
		}
		
		/* Set tess->event for debugging purposes */
		event = ((ActiveRegion *) dict->min()->key)->eUp->Org();
		doneEdgeDict( );
		donePriorityQ( );
		
		removeDegenerateFaces( );
		mesh->checkMesh();
	}
}
