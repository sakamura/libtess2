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

#include <stddef.h>
#include <assert.h>
#include <setjmp.h>
#include "bucketalloc.h"
#include "tess.h"
#include "mesh.h"
#include "sweep.h"
#include "geom.h"
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
     * the other chain is concave.  The left vertex of the single edge
     * is always to the left of all vertices in the concave chain.
     *
     * Each step consists of adding the rightmost unprocessed vertex to one
     * of the two chains, and forming a fan of triangles from the rightmost
     * of two chain endpoints.  Determining whether we can add each triangle
     * to the fan is a simple orientation test.  By making the fan as large
     * as possible, we restore the invariant (check it yourself).
     */
    void Tesselator::meshTessellateMonoRegion( TESSface *face )
    {
        TESShalfEdge *up, *lo;
        
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
                /* up->Dst is on the left.  It is safe to form triangles from lo->Org.
                 * The edgeGoesLeft test guarantees progress even when some triangles
                 * are CW, given that the upper and lower chains are truly monotone.
                 */
                while( lo->Lnext() != up && (edgeGoesLeft( lo->Lnext() )
                                           || edgeSign( lo->Org(), lo->Dst(), lo->Lnext()->Dst() ) <= 0 )) {
                    TESShalfEdge *tempHalfEdge= mesh->connect( lo->Lnext(), lo );
                    lo = tempHalfEdge->Sym();
                }
                lo = lo->Lprev();
            } else {
                /* lo->Org is on the left.  We can make CCW triangles from up->Dst. */
                while( lo->Lnext() != up && (edgeGoesRight( up->Lprev() )
                                           || edgeSign( up->Dst(), up->Org(), up->Lprev()->Org() ) >= 0 )) {
                    TESShalfEdge *tempHalfEdge= mesh->connect( up, up->Lprev() );
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
            TESShalfEdge *tempHalfEdge= mesh->connect( lo->Lnext(), lo );
            lo = tempHalfEdge->Sym();
        }
    }
    
    /* tessMeshTessellateInterior( mesh ) tessellates each region of
     * the mesh which is marked "inside" the polygon.  Each such region
     * must be monotone.
     */
    void Tesselator::meshTessellateInterior( )
    {
        TESSface *f, *next;
        
        /*LINTED*/
        for( f = mesh->fBegin(); f != mesh->fEnd(); f = next ) {
            /* Make sure we don't try to tessellate the new triangles. */
            next = f->next;
            if( f->inside ) {
                meshTessellateMonoRegion( f );
            }
        }
    }
    
    
    struct EdgeStackNode
    {
        TESShalfEdge *edge;
        EdgeStackNode *next;
        
        static void* operator new( std::size_t count ) { return BucketAlloc<EdgeStackNode>::get(count).alloc(); }
        static void operator delete( void* ptr ) { BucketAlloc<EdgeStackNode>::get().free(ptr); }
    };
    
    struct EdgeStack
    {
        EdgeStackNode *top;
    };
    
    void stackInit( EdgeStack *stack )
    {
        stack->top = nullptr;
    }
    
    void stackDelete( EdgeStack *stack )
    {
    }
    
    int stackEmpty( EdgeStack *stack )
    {
        return stack->top == nullptr;
    }
    
    void stackPush( EdgeStack *stack, TESShalfEdge *e )
    {
        EdgeStackNode *node = new EdgeStackNode;
        if ( ! node ) return;
        node->edge = e;
        node->next = stack->top;
        stack->top = node;
    }
    
    TESShalfEdge *stackPop( EdgeStack *stack )
    {
        TESShalfEdge *e = nullptr;
        EdgeStackNode *node = stack->top;
        if (node) {
            stack->top = node->next;
            e = node->edge;
            delete node;
        }
        return e;
    }
    
    /*
     Starting with a valid triangulation, uses the Edge Flip algorithm to
     refine the triangulation into a Constrained Delaunay Triangulation.
     */
    void Tesselator::meshRefineDelaunay( )
    {
        /* At this point, we have a valid, but not optimal, triangulation.
         We refine the triangulation using the Edge Flip algorithm */
        
        /*
         1) Find all internal edges
         2) Mark all dual edges
         3) insert all dual edges into a queue
         */
        TESSface *f;
        EdgeStack stack;
        TESShalfEdge *e;
        TESShalfEdge *edges[4];
        stackInit(&stack);
        for( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next ) {
            if ( f->inside) {
                e = f->anEdge;
                do {
                    e->SetMark(edgeIsInternal(e)); /* Mark internal edges */
                    if (e->mark() && !e->Sym()->mark()) stackPush(&stack, e); /* Insert into queue */
                    e = e->Lnext();
                } while (e != f->anEdge);
            }
        }
        
        // Pop stack until we find a reversed edge
        // Flip the reversed edge, and insert any of the four opposite edges
        // which are internal and not already in the stack (!marked)
        while (!stackEmpty(&stack)) {
            e = stackPop(&stack);
            e->SetMark(0);
            e->Sym()->SetMark(0);
            if (!edgeIsLocallyDelaunay(e)) {
                int i;
                mesh->flipEdge(e);
                // for each opposite edge
                edges[0] = e->Lnext();
                edges[1] = e->Lprev();
                edges[2] = e->Sym()->Lnext();
                edges[3] = e->Sym()->Lprev();
                for (i=0;i<3;i++) {
                    if (!edges[i]->mark() && edgeIsInternal(edges[i])) {
                        edges[i]->SetMark(1);
                        edges[i]->Sym()->SetMark(1);
                        stackPush(&stack, edges[i]);
                    }
                }
            }
        }
        
        stackDelete(&stack);
    }
    
    
    /* tessMeshDiscardExterior( mesh ) zaps (ie. sets to nullptr) all faces
     * which are not marked "inside" the polygon.  Since further mesh operations
     * on nullptr faces are not allowed, the main purpose is to clean up the
     * mesh so that exterior loops are not represented in the data structure.
     */
    void Tesselator::meshDiscardExterior()
    {
        TESSface *f, *next;
        
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
    void Tesselator::meshSetWindingNumber( int value, bool keepOnlyBoundary )
    {
        TESShalfEdge *e, *eNext;
        
        for( e = mesh->eBegin(); e != mesh->eEnd(); e = eNext ) {
            eNext = e->next();
            if( e->Rface()->inside != e->Lface()->inside ) {
                
                /* This is a boundary edge (one side is interior, one is exterior). */
                e->SetWinding((e->Lface()->inside) ? value : -value);
            } else {
                
                /* Both regions are interior, or both are exterior. */
                if( ! keepOnlyBoundary ) {
                    e->SetWinding( 0 );
                } else {
                    mesh->remove( e );
                }
            }
        }
    }
    
    Tesselator::Tesselator()
    {
        bmin[0] = MAXFLOAT;
        bmin[1] = MAXFLOAT;
        bmax[0] = -MAXFLOAT;
        bmax[1] = -MAXFLOAT;
        
        windingRule = TESS_WINDING_ODD;
        
        // Initialize to begin polygon.
        mesh = nullptr;
        
        vertexIndexCounter = 0;
        
        vertices = 0;
        vertexIndices = 0;
        vertexCount = 0;
        elements = 0;
        elementCount = 0;
    }

    Tesselator::~Tesselator()
    {
        if( mesh != nullptr ) {
            delete mesh;
            mesh = nullptr;
        }
        if (vertices != nullptr) {
            delete[] vertices;
            vertices = nullptr;
        }
        if (vertexIndices != nullptr) {
            delete[] vertexIndices;
            vertexIndices = nullptr;
        }
        if (elements != nullptr) {
            delete[] elements;
            elements = nullptr;
        }
    }
    
    
    static int GetNeighbourFace(TESShalfEdge* edge)
    {
        if (!edge->Rface())
            return sTessUndef;
        if (!edge->Rface()->inside)
            return sTessUndef;
        return edge->Rface()->n;
    }
    
    void Tesselator::outputPolymesh( ElementType elementType, int polySize )
    {
        TESSvertex* v = 0;
        TESSface* f = 0;
        TESShalfEdge* edge = 0;
        int maxFaceCount = 0;
        int maxVertexCount = 0;
        int faceVerts, i;
        float *vert;
        
        // Assume that the input data is triangles now.
        // Try to merge as many polygons as possible
        if (polySize > 3)
        {
            mesh->mergeConvexFaces( polySize );
        }
        
        // Mark unused
        for ( v = mesh->vBegin(); v != mesh->vEnd(); v = v->next )
            v->n = sTessUndef;
        
        // Create unique IDs for all vertices and faces.
        for ( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
        {
            f->n = sTessUndef;
            if( !f->inside ) continue;
            
            edge = f->anEdge;
            faceVerts = 0;
            do
            {
                v = edge->Org();
                if ( v->n == sTessUndef )
                {
                    v->n = maxVertexCount;
                    maxVertexCount++;
                }
                faceVerts++;
                edge = edge->Lnext();
            }
            while (edge != f->anEdge);
            
            assert( faceVerts <= polySize );
            
            f->n = maxFaceCount;
            ++maxFaceCount;
        }
        
        elementCount = maxFaceCount;
        if (elementType == TESS_CONNECTED_POLYGONS)
            maxFaceCount *= 2;
        int* elems = elements = new int[maxFaceCount * polySize];
        vertexCount = maxVertexCount;
        vertices = new float[vertexCount * 2];
        vertexIndices = new int[vertexCount];
        
        // Output vertices.
        for ( v = mesh->vBegin(); v != mesh->vEnd(); v = v->next )
        {
            if ( v->n != sTessUndef )
            {
                // Store coordinate
                vert = &vertices[v->n*2];
                vert[0] = v->s;
                vert[1] = v->t;
                // Store vertex index.
                vertexIndices[v->n] = v->idx;
            }
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
                *elems++ = v->n;
                faceVerts++;
                edge = edge->Lnext();
            }
            while (edge != f->anEdge);
            // Fill unused.
            for (i = faceVerts; i < polySize; ++i)
                *elems++ = sTessUndef;
            
            // Store polygon connectivity
            if ( elementType == TESS_CONNECTED_POLYGONS )
            {
                edge = f->anEdge;
                do
                {
                    *elems++ = GetNeighbourFace( edge );
                    edge = edge->Lnext();
                }
                while (edge != f->anEdge);
                // Fill unused.
                for (i = faceVerts; i < polySize; ++i)
                    *elems++ = sTessUndef;
            }
        }
    }
    
    void Tesselator::outputContours()
    {
        TESSface *f = 0;
        TESShalfEdge *edge = 0;
        TESShalfEdge *start = 0;
        int startVert = 0;
        int vertCount = 0;
        
        vertexCount = 0;
        elementCount = 0;
        
        for ( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
        {
            if ( !f->inside ) continue;
            
            start = edge = f->anEdge;
            do
            {
                ++vertexCount;
                edge = edge->Lnext();
            }
            while ( edge != start );
            
            ++elementCount;
        }
        
        int* elems = elements = new int[elementCount * 2];
        float* verts = vertices = new float[vertexCount * 2];
        int* vertInds = vertexIndices = new int[vertexCount];
        
        startVert = 0;
        
        for ( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
        {
            if ( !f->inside ) continue;
            
            vertCount = 0;
            start = edge = f->anEdge;
            do
            {
                *verts++ = edge->Org()->s;
                *verts++ = edge->Org()->t;
                *vertInds++ = edge->Org()->idx;
                ++vertCount;
                edge = edge->Lnext();
            }
            while ( edge != start );
            
            elems[0] = startVert;
            elems[1] = vertCount;
            elems += 2;
            
            startVert += vertCount;
        }
    }
    
    void Tesselator::beginContour()
    {
        if ( mesh == nullptr )
            mesh = new TESSmesh;
        
        e = nullptr;
    }
    
    void Tesselator::addVertex( float x, float y )
    {
        if( e == nullptr ) {
            /* Make a self-loop (one vertex, one edge). */
            e = mesh->makeEdge();
            mesh->splice(e, e->Sym() );
        } else {
            /* Create a new vertex and edge which immediately follow tess->e
             * in the ordering around the left face.
             */
            mesh->splitEdge( e );
            e = e->Lnext();
        }
        
        /* The new vertex is now tess->e->Org. */
        e->Org()->s = x;
        e->Org()->t = y;
        if (bmin[0] > x) bmin[0] = x;
        if (bmin[1] > y) bmin[1] = y;
        if (bmax[0] < x) bmax[0] = x;
        if (bmax[1] < y) bmax[1] = y;
        
        /* Store the insertion number so that the vertex can be later recognized. */
        e->Org()->idx = vertexIndexCounter++;
        
        /* The winding of an edge says how the winding number changes as we
         * cross from the edge''s right face to its left face.  We add the
         * vertices in such an order that a CCW contour will add +1 to
         * the winding number of the region inside the contour.
         */
        e->SetWinding(1);
        e->Sym()->SetWinding(-1);
    }
    
    void Tesselator::addContour( const void* vertices, int stride, int numVertices )
    {
        const unsigned char *src = (const unsigned char*)vertices;
        int i;
        
        beginContour();
        
        for( i = 0; i < numVertices; ++i )
        {
            const float* coords = (const float*)src;
            src += stride;
            addVertex(coords[0], coords[1]);
        }
    }
    
    void Tesselator::tesselate( WindingRule windingRule, ElementType elementType, int polySize, const float* normal )
    {
        if (vertices != nullptr) {
            delete[] vertices;
            vertices = nullptr;
        }
        if (elements != nullptr) {
            delete[] elements;
            elements = nullptr;
        }
        if (vertexIndices != nullptr) {
            delete[] vertexIndices;
            vertexIndices = nullptr;
        }
        
        vertexIndexCounter = 0;
        windingRule = windingRule;
        
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
            if (elementType == TESS_CONSTRAINED_DELAUNAY_TRIANGLES) {
                meshRefineDelaunay( );
                elementType = TESS_POLYGONS;
                polySize = 3;
            }
        }
        
        mesh->checkMesh( );
        
        if (elementType == TESS_BOUNDARY_CONTOURS) {
            outputContours( );     /* output contours */
        }
        else
        {
            outputPolymesh( elementType, polySize );     /* output polygons */
        }
        
        delete mesh;
        mesh = nullptr;
    }
}