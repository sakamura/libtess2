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

#include "options.h"

namespace Tess
{
	enum ElementType
	{
		TESS_POLYGONS,
		TESS_CONNECTED_POLYGONS,
		TESS_BOUNDARY_CONTOURS,
	};
	
	static const int sTessUndef = (~(int)0);

    template <typename Options, typename Allocators>
	class MeshT;

    template <typename Options, typename Allocators>
    struct DictT;

    template <typename Options, typename Allocators>
    struct PriorityQT;
	
    template <typename Options, typename Allocators>
    struct VertexT;
	
    template <typename Options, typename Allocators>
    struct HalfEdgeT;
    
    template <typename Options, typename Allocators>
    struct EdgePairT;
    
    template <typename Options, typename Allocators>
    struct FaceT;
    
    template <typename Options, typename Allocators>
    struct ActiveRegionT;
    
    template <typename Options>
    struct BaseAllocators;
    
    template <typename Options = BaseOptions, typename Allocators = BaseAllocators<Options> >
	class Tesselator
	{
        using Mesh = MeshT<Options, Allocators>;
        using Vertex = VertexT<Options, Allocators>;
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        using Face = FaceT<Options, Allocators>;
        using ActiveRegion = ActiveRegionT<Options, Allocators>;
        using PriorityQ = PriorityQT<Options, Allocators>;
        using Dict = DictT<Options, Allocators>;
        using Coord = typename Options::Coord;
        using Vec = typename Options::Vec;
        using SweepPlaneVec = typename Options::SweepPlaneVec;
        using InternalVec = typename Options::InternalVec;

    public:
        Options options;          // Options, as set up in constructor.
        Allocators allocators;    // Allocators, as used in the different tesselators.
        
    private:
		/*** state needed for collecting the input data ***/
		Mesh	*mesh;		/* stores the input contours, and eventually
								 the tessellation itself */
		
		Coord bmin[2];
		Coord bmax[2];
		
		/*** state needed for the line sweep ***/
		Dict *dict;		/* edge dictionary for sweep line */
		PriorityQ *pq;		/* priority queue of vertex events */
		Vertex *event;		/* current sweep event being processed */
		HalfEdge *e;
		
	public:
		Tesselator(Options& options);
		~Tesselator();

		void beginContour();
		
		void addVertex(const Vec& vec);
		
		// addContour() - Adds a contour to be tesselated.
		// The type of the vertex coordinates is assumed to be float.
		// Parameters:
		//	 pointer - pointer to the first coordinate of the first vertex in the array.
		//	 stride - defines offset in bytes between consecutive vertices.
		//	 count - number of vertices in contour.
		void addContour(const void* pointer, int stride, int count);
		
		// tesselate() - tesselate contours.
		// Parameters:
		//	 elementType - defines the tesselation result element type, must be one of ElementType.
		//	 polySize - defines maximum vertices per polygons if output is polygons.
		//	 normal - defines the normal of the input contours, of nullptr the normal is calculated automatically.
		// Returns:
		//	 1 if succeed, 0 if failed.
		void tesselate(ElementType elementType, int polySize);
		
	private:
		// tess.cpp
		void meshTessellateMonoRegion(Face *face);
		void meshTessellateInterior();
		void meshRefineDelaunay();
		void meshDiscardExterior();
		void meshSetWindingNumber(int value, bool keepOnlyBoundary);
		void outputPolymesh(int polySize);
		void outputContours();
		
		// sweep.cpp
		static bool edgeLeq( const Tesselator<Options, Allocators>* tess, ActiveRegion *reg1, ActiveRegion *reg2 )
		{
			return tess->_edgeLeq(reg1, reg2);
		}
		bool _edgeLeq( ActiveRegion *reg1, ActiveRegion *reg2 ) const;
		void deleteRegion( ActiveRegion *reg );
		void fixUpperEdge( ActiveRegion *reg, HalfEdge *newEdge );
		ActiveRegion* topLeftRegion( ActiveRegion *reg );
		ActiveRegion* topRightRegion( ActiveRegion *reg );
		ActiveRegion* addRegionBelow( ActiveRegion *regAbove, HalfEdge *eNewUp );
		bool isWindingInside( int n ) const;
		void computeWinding( ActiveRegion *reg );
		void finishRegion( ActiveRegion *reg );
		HalfEdge* finishLeftRegions( ActiveRegion *regFirst, ActiveRegion *regLast );
		void addRightEdges( ActiveRegion *regUp, HalfEdge *eFirst, HalfEdge *eLast, HalfEdge *eTopLeft, bool cleanUp );
		void spliceMergeVertices(HalfEdge *e1, HalfEdge *e2);
		bool checkForRightSplice( ActiveRegion *regUp );
		bool checkForLeftSplice( ActiveRegion *regUp );
		bool checkForIntersect( ActiveRegion *regUp );
		void walkDirtyRegions( ActiveRegion *regUp );
		void connectRightVertex( ActiveRegion *regUp, HalfEdge *eBottomLeft );
		void connectLeftDegenerate( ActiveRegion *regUp, Vertex *vEvent );
		void connectLeftVertex( Vertex *vEvent );
		void sweepEvent( Vertex *vEvent );
		void addSentinel( typename Options::Coord smin, typename Options::Coord smax, typename Options::Coord t );
		void initEdgeDict( );
		void doneEdgeDict( );
		void removeDegenerateEdges();
		void initPriorityQ( );
		void donePriorityQ( );
		void removeDegenerateFaces( );

		/* computeInterior( ) computes the planar arrangement specified
		 * by the given contours, and further subdivides this arrangement
		 * into regions.  Each region is marked "inside" if it belongs
		 * to the polygon, according to the rule given by windingRule.
		 * Each interior region is guaranteed be monotone.
		 */
		void computeInterior();
	};
}

#include "tess.inl"
