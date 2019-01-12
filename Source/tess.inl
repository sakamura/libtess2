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

#include <stddef.h>
#include <assert.h>
#include <setjmp.h>
#include "bucketalloc.h"
#include "tess.h"
#include "mesh.h"
#include "sweep.h"
#include "geom.h"
#include "edgestack.h"
#include "allocators.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace Tess
{
	/* tessMeshTessellateMonoRegion( face ) tessellates a monotone region
	 * (what else would it do??)  The region must consist of a single
	 * loop of half-edges (see mesh.h) oriented CCW.  "Monotone" in this
	 * case means that any vertical line intersects the interior of the
	 * region in a single interval.	 
	 *
	 * Tessellation consists of adding interior edges (actually pairs of
	 * half-edges), to split the region into non-overlapping triangles.
	 *
	 * The basic idea is explained in Preparata and Shamos (which I don''t
	 * have handy right now), although their implementation is more
	 * complicated than this one.  The are two edge chains, an upper chain
	 * and a lower chain.  We process all vertices from both chains in order,
	 * from right to left.
	 *
	 * The algorithm ensures that the following invariant holds after each
	 * vertex is processed: the untessellated region consists of two
	 * chains, where one chain (say the upper) is a single edge, and
	 * the other chain is concave.	The left vertex of the single edge
	 * is always to the left of all vertices in the concave chain.
	 *
	 * Each step consists of adding the rightmost unprocessed vertex to one
	 * of the two chains, and forming a fan of triangles from the rightmost
	 * of two chain endpoints.	Determining whether we can add each triangle
	 * to the fan is a simple orientation test.	 By making the fan as large
	 * as possible, we restore the invariant (check it yourself).
	 */
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::meshTessellateMonoRegion( Face *face )
	{
		HalfEdge *up, *lo;
		
		/* All edges are oriented CCW around the boundary of the region.
		 * First, find the half-edge whose origin vertex is rightmost.
		 * Since the sweep goes from left to right, face->anEdge should
		 * be close to the edge we want.
		 */
		up = face->anEdge;
		assert( up->Lnext() != up && up->Lnext()->Lnext() != up );
		
		for( ; vertAreLessOrEqual( up->Dst(), up->Org() ); up = up->Lprev() )
			;
		for( ; vertAreLessOrEqual( up->Org(), up->Dst() ); up = up->Lnext() )
			;
		lo = up->Lprev();
		
		while( up->Lnext() != lo ) {
			if( vertAreLessOrEqual( up->Dst(), lo->Org() )) {
				/* up->Dst is on the left.	It is safe to form triangles from lo->Org.
				 * The edgeGoesLeft test guarantees progress even when some triangles
				 * are CW, given that the upper and lower chains are truly monotone.
				 */
				while( lo->Lnext() != up && (edgeGoesLeft( lo->Lnext() )
										   || edgeSign( lo->Org(), lo->Dst(), lo->Lnext()->Dst() ) <= 0 )) {
					HalfEdge *tempHalfEdge= mesh->connect( lo->Lnext(), lo );
					lo = tempHalfEdge->Sym();
				}
				lo = lo->Lprev();
			} else {
				/* lo->Org is on the left.	We can make CCW triangles from up->Dst. */
				while( lo->Lnext() != up && (edgeGoesRight( up->Lprev() )
										   || edgeSign( up->Dst(), up->Org(), up->Lprev()->Org() ) >= 0 )) {
					HalfEdge *tempHalfEdge= mesh->connect( up, up->Lprev() );
					up = tempHalfEdge->Sym();
				}
				up = up->Lnext();
			}
		}
		
		/* Now lo->Org == up->Dst == the leftmost vertex.  The remaining region
		 * can be tessellated in a fan from this leftmost vertex.
		 */
		assert( lo->Lnext() != up );
		while( lo->Lnext()->Lnext() != up ) {
			HalfEdge *tempHalfEdge= mesh->connect( lo->Lnext(), lo );
			lo = tempHalfEdge->Sym();
		}
	}
	
	/* tessMeshTessellateInterior( mesh ) tessellates each region of
	 * the mesh which is marked "inside" the polygon.  Each such region
	 * must be monotone.
	 */
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::meshTessellateInterior( )
	{
		Face *f, *next;
		
		/*LINTED*/
		for( f = mesh->fBegin(); f != mesh->fEnd(); f = next ) {
			/* Make sure we don't try to tessellate the new triangles. */
			next = f->next;
			if( f->inside ) {
				meshTessellateMonoRegion( f );
			}
		}
	}

    
    //	Starting with a valid triangulation, uses the Edge Flip algorithm to
	//	refine the triangulation into a Constrained Delaunay Triangulation.
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::meshRefineDelaunay( )
	{
        using EdgeStack = EdgeStackT<Options, Allocators>;
        
		// At this point, we have a valid, but not optimal, triangulation.
		// We refine the triangulation using the Edge Flip algorithm
		//
		//	1) Find all internal edges
		//	2) Mark all dual edges
		//	3) insert all dual edges into a queue

		Face *f;
		EdgeStack stack(this);
		HalfEdge *edge;
		int maxFaces = 0, maxIter = 0, iter = 0;

		for( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next ) {
			if ( f->inside) {
				edge = f->anEdge;
				do {
					edge->setMark(edgeIsInternal(edge)); /* Mark internal edges */
					if (edge->mark() && !edge->Sym()->mark()) stack.push(edge); /* Insert into queue */
					edge = edge->Lnext();
				} while (edge != f->anEdge);
				maxFaces++;
			}
		}
	
		// The algorithm should converge on O(n^2), since the predicate is not robust,
		// we'll save guard against infinite loop.
		maxIter = maxFaces * maxFaces;

		// Pop stack until we find a reversed edge
		// Flip the reversed edge, and insert any of the four opposite edges
		// which are internal and not already in the stack (!marked)
		while (!stack.empty() && iter < maxIter) {
			edge = stack.pop();
			edge->setMark(0);
			edge->Sym()->setMark(0);
			if (!edgeIsLocallyDelaunay(edge)) {
				HalfEdge *edges[4];
				int i;
				mesh->flipEdge(edge);
				// for each opposite edge
				edges[0] = edge->Lnext();
				edges[1] = edge->Lprev();
				edges[2] = edge->Sym()->Lnext();
				edges[3] = edge->Sym()->Lprev();
				for (i = 0; i < 4; i++) {
					if (!edges[i]->mark() && edgeIsInternal(edges[i])) {
						edges[i]->setMark(1);
						edges[i]->Sym()->setMark(1);
						stack.push(edges[i]);
					}
				}
			}
			iter++;
		}
	}
	
	
	/* tessMeshDiscardExterior( mesh ) zaps (ie. sets to nullptr) all faces
	 * which are not marked "inside" the polygon.  Since further mesh operations
	 * on nullptr faces are not allowed, the main purpose is to clean up the
	 * mesh so that exterior loops are not represented in the data structure.
	 */
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::meshDiscardExterior()
	{
		Face *f, *next;
		
		/*LINTED*/
		for( f = mesh->fBegin(); f != mesh->fEnd(); f = next ) {
			/* Since f will be destroyed, save its next pointer. */
			next = f->next;
			if( ! f->inside ) {
				mesh->zapFace( f );
			}
		}
	}
	
	/* tessMeshSetWindingNumber( mesh, value, keepOnlyBoundary ) resets the
	 * winding numbers on all edges so that regions marked "inside" the
	 * polygon have a winding number of "value", and regions outside
	 * have a winding number of 0.
	 *
	 * If keepOnlyBoundary is true, it also deletes all edges which do not
	 * separate an interior region from an exterior one.
	 */
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::meshSetWindingNumber( int value, bool keepOnlyBoundary )
	{
		HalfEdge *edge, *eNext;
		
		for( edge = mesh->eBegin(); edge != mesh->eEnd(); edge = eNext ) {
			eNext = edge->next();
			if( edge->Rface()->inside != edge->Lface()->inside ) {
				
				/* This is a boundary edge (one side is interior, one is exterior). */
				edge->setWinding((edge->Lface()->inside) ? value : -value);
			} else {
				
				/* Both regions are interior, or both are exterior. */
				if( ! keepOnlyBoundary ) {
					edge->setWinding( 0 );
				} else {
					mesh->remove( edge );
				}
			}
		}
	}
	
    template <typename Options, typename Allocators>
    Tesselator<Options, Allocators>::Tesselator(Options& _options) :
        options(_options),
        allocators(options)
	{
		bmin[0] = std::numeric_limits<typename Options::Coord>::max();
		bmin[1] = std::numeric_limits<typename Options::Coord>::max();
		bmax[0] = -std::numeric_limits<typename Options::Coord>::max();
		bmax[1] = -std::numeric_limits<typename Options::Coord>::max();
		
		// Initialize to begin polygon.
		mesh = nullptr;
	}

    template <typename Options, typename Allocators>
	Tesselator<Options, Allocators>::~Tesselator()
	{
		if( mesh != nullptr ) {
			delete mesh;
			mesh = nullptr;
		}
	}
	
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::outputPolymesh( int polySize )
	{
		Vertex* v = 0;
		Face* f = 0;
		HalfEdge* edge = 0;
		int faceVerts, i;
		
		// Assume that the input data is triangles now.
		// Try to merge as many polygons as possible
		if (polySize > 3)
		{
			mesh->mergeConvexFaces( polySize );
		}
		
		// Output indices.
		for ( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
		{
			if ( !f->inside ) continue;
		
			// Store polygon
			edge = f->anEdge;
			faceVerts = 0;
			do
			{
				v = edge->Org();
                options.addVertex(v->idx, v);
				faceVerts++;
				edge = edge->Lnext();
			}
			while (edge != f->anEdge);
			// Fill unused.
			for (i = faceVerts; i < polySize; ++i)
                options.addEmptyVertex();
		}
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::outputContours()
	{
		Face *f = 0;
		HalfEdge *edge = 0;
		HalfEdge *start = 0;

        for ( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
		{
			if ( !f->inside ) continue;
			
			start = edge = f->anEdge;
			do
			{
                options.addContour(edge->Org()->idx, edge->Org());
				edge = edge->Lnext();
			}
			while ( edge != start );
            options.addContour(edge->Org()->idx, edge->Org());   // Repeat start index (makes a full contour loop).
		}
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::beginContour()
	{
		if ( mesh == nullptr )
			mesh = new Mesh(this);
		
		e = nullptr;
	}
	
    template <typename Options, typename Allocators>
    void Tesselator<Options, Allocators>::addVertex( const Vec& vec )
	{
		if( e == nullptr ) {
			/* Make a self-loop (one vertex, one edge). */
			e = mesh->makeEdge();
			mesh->splice(e, e->Sym() );
		} else {
			/* Create a new vertex and edge which immediately follow e
			 * in the ordering around the left face.
			 */
			mesh->splitEdge( e );
			e = e->Lnext();
		}
		
		/* The new vertex is now e->Org. */
        auto result = options.addPoint(vec);

        auto sVec = e->Org()->sVec = result.first;
        Coord s = Options::getS(sVec);
        Coord t = Options::getT(sVec);
		if (bmin[0] > s) bmin[0] = s;
		if (bmin[1] > t) bmin[1] = t;
		if (bmax[0] < s) bmax[0] = s;
		if (bmax[1] < t) bmax[1] = t;
	
		/* Store the insertion number so that the vertex can be later recognized. */
		e->Org()->idx = result.second;
		
		/* The winding of an edge says how the winding number changes as we
		 * cross from the edge''s right face to its left face.	We add the
		 * vertices in such an order that a CCW contour will add +1 to
		 * the winding number of the region inside the contour.
		 */
		e->setWinding(options.reverseContours() ? -1 : 1);
		e->Sym()->setWinding(options.reverseContours() ? 1 : -1);
	}
	
    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::addContour( const void* verticesToAdd, int stride, int numVertices )
	{
		const unsigned char *src = (const unsigned char*)verticesToAdd;
		int i;
		
		beginContour();
		
		for( i = 0; i < numVertices; ++i )
		{
			const typename Options::Coord* coords = (const typename Options::Coord*)src;
			src += stride;
			addVertex(coords[0], coords[1]);
		}
	}

    template <typename Options, typename Allocators>
	void Tesselator<Options, Allocators>::tesselate( ElementType elementType, int polySize )
	{
		/* tessComputeInterior( tess ) computes the planar arrangement specified
		 * by the given contours, and further subdivides this arrangement
		 * into regions.  Each region is marked "inside" if it belongs
		 * to the polygon, according to the rule given by tess->windingRule.
		 * Each interior region is guaranteed be monotone.
		 */
		computeInterior();
		
		/* If the user wants only the boundary contours, we throw away all edges
		 * except those which separate the interior from the exterior.
		 * Otherwise we tessellate all the regions marked "inside".
		 */
		if (elementType == TESS_BOUNDARY_CONTOURS) {
			meshSetWindingNumber( 1, true );
		} else {
			meshTessellateInterior( );
			if (options.constrainedDelaunayTriangulation())
				meshRefineDelaunay( );
		}
		
		mesh->checkMesh( );
		
		if (elementType == TESS_BOUNDARY_CONTOURS) {
			outputContours( );	   /* output contours */
		}
		else
		{
			outputPolymesh( polySize );	 /* output polygons */
		}
		
		delete mesh;
		mesh = nullptr;
	}
}
