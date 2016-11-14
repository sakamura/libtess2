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

#ifndef TESS_H
#define TESS_H

namespace Tess
{
    // See OpenGL Red Book for description of the winding rules
    // http://www.glprogramming.com/red/chapter11.html
    enum WindingRule
    {
        TESS_WINDING_ODD,
        TESS_WINDING_NONZERO,
        TESS_WINDING_POSITIVE,
        TESS_WINDING_NEGATIVE,
        TESS_WINDING_ABS_GEQ_TWO,
    };
    
    enum ElementType
    {
        TESS_POLYGONS,
        TESS_CONNECTED_POLYGONS,
        TESS_BOUNDARY_CONTOURS,
        TESS_CONSTRAINED_DELAUNAY_TRIANGLES
    };
    
    static const int sTessUndef = (~(int)0);

    struct TESSmesh;
    struct Dict;
    struct PriorityQ;
    struct TESSvertex;
    struct TESShalfEdge;
    struct TESSface;
    struct ActiveRegion;
    
    class Tesselator
    {
        /*** state needed for collecting the input data ***/
        TESSmesh	*mesh;		/* stores the input contours, and eventually
                                 the tessellation itself */
        
        float bmin[2];
        float bmax[2];
        
        /*** state needed for the line sweep ***/
        WindingRule	windingRule;	/* rule for determining polygon interior */
        Dict *dict;		/* edge dictionary for sweep line */
        PriorityQ *pq;		/* priority queue of vertex events */
        TESSvertex *event;		/* current sweep event being processed */
        int vertexIndexCounter;
        TESShalfEdge *e;
        
        /*** outputs ***/
        float *vertices;
        int *vertexIndices;
        int vertexCount;
        int *elements;
        int elementCount;
        
    public:
        Tesselator();
        ~Tesselator();

        void beginContour();
        
        void addVertex(float x, float y);
        
        // addContour() - Adds a contour to be tesselated.
        // The type of the vertex coordinates is assumed to be float.
        // Parameters:
        //   pointer - pointer to the first coordinate of the first vertex in the array.
        //   stride - defines offset in bytes between consecutive vertices.
        //   count - number of vertices in contour.
        void addContour(const void* pointer, int stride, int count);
        
        // tesselate() - tesselate contours.
        // Parameters:
        //   windingRule - winding rules used for tesselation, must be one of WindingRule.
        //   elementType - defines the tesselation result element type, must be one of ElementType.
        //   polySize - defines maximum vertices per polygons if output is polygons. If elementType is TESS_CONSTRAINED_DELAUNAY_TRIANGLES, this parameter is ignored.
        //   normal - defines the normal of the input contours, of nullptr the normal is calculated automatically.
        // Returns:
        //   1 if succeed, 0 if failed.
        void tesselate(WindingRule windingRule, ElementType elementType, int polySize, const float* normal);
        
        // getVertexCount() - Returns number of vertices in the tesselated output.
        int getVertexCount() const
        {
            return vertexCount;
        }
        
        // getVertices() - Returns pointer to first coordinate of first vertex.
        const float* getVertices() const
        {
            return vertices;
        }
        
        // getVertexIndices() - Returns pointer to first vertex index.
        // Vertex indices can be used to map the generated vertices to the original vertices.
        // Every point added using addContour() will get a new index starting at 0.
        // New vertices generated at the intersections of segments are assigned value sTessUndef.
        const int* getVertexIndices() const
        {
            return vertexIndices;
        }
        
        // getElementCount() - Returns number of elements in the the tesselated output.
        int getElementCount() const
        {
            return elementCount;
        }
        
        // getElements() - Returns pointer to the first element.
        const int* getElements() const
        {
            return elements;
        }
        
    private:
        // tess.cpp
        void meshTessellateMonoRegion(TESSface *face);
        void meshTessellateInterior();
        void meshRefineDelaunay();
        void meshDiscardExterior();
        void meshSetWindingNumber(int value, bool keepOnlyBoundary);
        void outputPolymesh(ElementType elementType, int polySize);
        void outputContours();
        
        // sweep.cpp
        static bool edgeLeq( const Tesselator* tess, ActiveRegion *reg1, ActiveRegion *reg2 )
        {
            return tess->_edgeLeq(reg1, reg2);
        }
        bool _edgeLeq( ActiveRegion *reg1, ActiveRegion *reg2 ) const;
        void deleteRegion( ActiveRegion *reg );
        void fixUpperEdge( ActiveRegion *reg, TESShalfEdge *newEdge );
        ActiveRegion* topLeftRegion( ActiveRegion *reg );
        ActiveRegion* topRightRegion( ActiveRegion *reg );
        ActiveRegion* addRegionBelow( ActiveRegion *regAbove, TESShalfEdge *eNewUp );
        bool isWindingInside( int n ) const;
        void computeWinding( ActiveRegion *reg );
        void finishRegion( ActiveRegion *reg );
        TESShalfEdge* finishLeftRegions( ActiveRegion *regFirst, ActiveRegion *regLast );
        void addRightEdges( ActiveRegion *regUp, TESShalfEdge *eFirst, TESShalfEdge *eLast, TESShalfEdge *eTopLeft, bool cleanUp );
        void spliceMergeVertices(TESShalfEdge *e1, TESShalfEdge *e2);
        bool checkForRightSplice( ActiveRegion *regUp );
        bool checkForLeftSplice( ActiveRegion *regUp );
        bool checkForIntersect( ActiveRegion *regUp );
        void walkDirtyRegions( ActiveRegion *regUp );
        void connectRightVertex( ActiveRegion *regUp, TESShalfEdge *eBottomLeft );
        void connectLeftDegenerate( ActiveRegion *regUp, TESSvertex *vEvent );
        void connectLeftVertex( TESSvertex *vEvent );
        void sweepEvent( TESSvertex *vEvent );
        void addSentinel( float smin, float smax, float t );
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

#endif
