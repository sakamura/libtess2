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
#define TRUE 1
#define FALSE 0
    
#define S_UNIT_X	(TESSreal)1.0
#define S_UNIT_Y	(TESSreal)0.0
    
#define AddWinding(eDst,eSrc)	(eDst->winding += eSrc->winding, \
eDst->Sym->winding += eSrc->Sym->winding)
    
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
    void tessMeshTessellateMonoRegion( TESSmesh *mesh, TESSface *face )
    {
        TESShalfEdge *up, *lo;
        
        /* All edges are oriented CCW around the boundary of the region.
         * First, find the half-edge whose origin vertex is rightmost.
         * Since the sweep goes from left to right, face->anEdge should
         * be close to the edge we want.
         */
        up = face->anEdge;
        assert( up->Lnext != up && up->Lnext->Lnext != up );
        
        for( ; VertLeq( up->Dst, up->Org ); up = up->Lprev )
            ;
        for( ; VertLeq( up->Org, up->Dst ); up = up->Lnext )
            ;
        lo = up->Lprev;
        
        while( up->Lnext != lo ) {
            if( VertLeq( up->Dst, lo->Org )) {
                /* up->Dst is on the left.  It is safe to form triangles from lo->Org.
                 * The EdgeGoesLeft test guarantees progress even when some triangles
                 * are CW, given that the upper and lower chains are truly monotone.
                 */
                while( lo->Lnext != up && (EdgeGoesLeft( lo->Lnext )
                                           || EdgeSign( lo->Org, lo->Dst, lo->Lnext->Dst ) <= 0 )) {
                    TESShalfEdge *tempHalfEdge= tessMeshConnect( mesh, lo->Lnext, lo );
                    lo = tempHalfEdge->Sym;
                }
                lo = lo->Lprev;
            } else {
                /* lo->Org is on the left.  We can make CCW triangles from up->Dst. */
                while( lo->Lnext != up && (EdgeGoesRight( up->Lprev )
                                           || EdgeSign( up->Dst, up->Org, up->Lprev->Org ) >= 0 )) {
                    TESShalfEdge *tempHalfEdge= tessMeshConnect( mesh, up, up->Lprev );
                    up = tempHalfEdge->Sym;
                }
                up = up->Lnext;
            }
        }
        
        /* Now lo->Org == up->Dst == the leftmost vertex.  The remaining region
         * can be tessellated in a fan from this leftmost vertex.
         */
        assert( lo->Lnext != up );
        while( lo->Lnext->Lnext != up ) {
            TESShalfEdge *tempHalfEdge= tessMeshConnect( mesh, lo->Lnext, lo );
            lo = tempHalfEdge->Sym;
        }
    }
    
    /* tessMeshTessellateInterior( mesh ) tessellates each region of
     * the mesh which is marked "inside" the polygon.  Each such region
     * must be monotone.
     */
    void tessMeshTessellateInterior( TESSmesh *mesh )
    {
        TESSface *f, *next;
        
        /*LINTED*/
        for( f = mesh->fHead.next; f != &mesh->fHead; f = next ) {
            /* Make sure we don''t try to tessellate the new triangles. */
            next = f->next;
            if( f->inside ) {
                tessMeshTessellateMonoRegion( mesh, f );
            }
        }
    }
    
    
    typedef struct EdgeStackNode EdgeStackNode;
    typedef struct EdgeStack EdgeStack;
    
    struct EdgeStackNode {
        TESShalfEdge *edge;
        EdgeStackNode *next;
    };
    
    struct EdgeStack {
        EdgeStackNode *top;
        struct BucketAlloc *nodeBucket;
    };
    
    int stackInit( EdgeStack *stack, TESSalloc *alloc )
    {
        stack->top = NULL;
        stack->nodeBucket = createBucketAlloc( alloc, "CDT nodes", sizeof(EdgeStackNode), 512 );
        return stack->nodeBucket != NULL;
    }
    
    void stackDelete( EdgeStack *stack )
    {
        deleteBucketAlloc( stack->nodeBucket );
    }
    
    int stackEmpty( EdgeStack *stack )
    {
        return stack->top == NULL;
    }
    
    void stackPush( EdgeStack *stack, TESShalfEdge *e )
    {
        EdgeStackNode *node = (EdgeStackNode *)bucketAlloc( stack->nodeBucket );
        if ( ! node ) return;
        node->edge = e;
        node->next = stack->top;
        stack->top = node;
    }
    
    TESShalfEdge *stackPop( EdgeStack *stack )
    {
        TESShalfEdge *e = NULL;
        EdgeStackNode *node = stack->top;
        if (node) {
            stack->top = node->next;
            e = node->edge;
            bucketFree( stack->nodeBucket, node );
        }
        return e;
    }
    
    /*
     Starting with a valid triangulation, uses the Edge Flip algorithm to
     refine the triangulation into a Constrained Delaunay Triangulation.
     */
    void tessMeshRefineDelaunay( TESSmesh *mesh, TESSalloc *alloc )
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
        stackInit(&stack, alloc);
        for( f = mesh->fHead.next; f != &mesh->fHead; f = f->next ) {
            if ( f->inside) {
                e = f->anEdge;
                do {
                    e->mark = EdgeIsInternal(e); /* Mark internal edges */
                    if (e->mark && !e->Sym->mark) stackPush(&stack, e); /* Insert into queue */
                    e = e->Lnext;
                } while (e != f->anEdge);
            }
        }
        
        // Pop stack until we find a reversed edge
        // Flip the reversed edge, and insert any of the four opposite edges
        // which are internal and not already in the stack (!marked)
        while (!stackEmpty(&stack)) {
            e = stackPop(&stack);
            e->mark = e->Sym->mark = 0;
            if (!tesedgeIsLocallyDelaunay(e)) {
                int i;
                tessMeshFlipEdge(mesh, e);
                // for each opposite edge
                edges[0] = e->Lnext;
                edges[1] = e->Lprev;
                edges[2] = e->Sym->Lnext;
                edges[3] = e->Sym->Lprev;
                for (i=0;i<3;i++) {
                    if (!edges[i]->mark && EdgeIsInternal(edges[i])) {
                        edges[i]->mark = edges[i]->Sym->mark = 1;
                        stackPush(&stack, edges[i]);
                    }
                }
            }
        }
        
        stackDelete(&stack);
    }
    
    
    /* tessMeshDiscardExterior( mesh ) zaps (ie. sets to NULL) all faces
     * which are not marked "inside" the polygon.  Since further mesh operations
     * on NULL faces are not allowed, the main purpose is to clean up the
     * mesh so that exterior loops are not represented in the data structure.
     */
    void tessMeshDiscardExterior( TESSmesh *mesh )
    {
        TESSface *f, *next;
        
        /*LINTED*/
        for( f = mesh->fHead.next; f != &mesh->fHead; f = next ) {
            /* Since f will be destroyed, save its next pointer. */
            next = f->next;
            if( ! f->inside ) {
                tessMeshZapFace( mesh, f );
            }
        }
    }
    
    /* tessMeshSetWindingNumber( mesh, value, keepOnlyBoundary ) resets the
     * winding numbers on all edges so that regions marked "inside" the
     * polygon have a winding number of "value", and regions outside
     * have a winding number of 0.
     *
     * If keepOnlyBoundary is TRUE, it also deletes all edges which do not
     * separate an interior region from an exterior one.
     */
    void tessMeshSetWindingNumber( TESSmesh *mesh, int value,
                                 int keepOnlyBoundary )
    {
        TESShalfEdge *e, *eNext;
        
        for( e = mesh->eHead.next; e != &mesh->eHead; e = eNext ) {
            eNext = e->next;
            if( e->Rface->inside != e->Lface->inside ) {
                
                /* This is a boundary edge (one side is interior, one is exterior). */
                e->winding = (e->Lface->inside) ? value : -value;
            } else {
                
                /* Both regions are interior, or both are exterior. */
                if( ! keepOnlyBoundary ) {
                    e->winding = 0;
                } else {
                    tessMeshDelete( mesh, e );
                }
            }
        }
    }
    
    void* heapAlloc( void* userData, size_t size )
    {
        TESS_NOTUSED( userData );
        return malloc( size );
    }
    
    void* heapRealloc( void *userData, void* ptr, size_t size )
    {
        TESS_NOTUSED( userData );
        return realloc( ptr, size );
    }
    
    void heapFree( void* userData, void* ptr )
    {
        TESS_NOTUSED( userData );
        free( ptr );
    }
    
    static TESSalloc defaulAlloc =
    {
        heapAlloc,
        heapRealloc,
        heapFree,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };
    void tessCleanupDefaultAlloc()
    {
        tessCleanupAlloc(&defaulAlloc);
    }
    
    TESStesselator* tessNewTess( TESSalloc* alloc )
    {
        TESStesselator* tess;
        
        if (alloc == NULL)
            alloc = &defaulAlloc;
        
        /* Only initialize fields which can be changed by the api.  Other fields
         * are initialized where they are used.
         */
        
        tess = (TESStesselator *)alloc->memalloc( alloc->userData, sizeof( TESStesselator ));
        if ( tess == NULL ) {
            return 0;          /* out of memory */
        }
        tess->alloc = alloc;
        /* Check and set defaults. */
        if (tess->alloc->meshEdgeBucketSize == 0)
            tess->alloc->meshEdgeBucketSize = 512;
        if (tess->alloc->meshVertexBucketSize == 0)
            tess->alloc->meshVertexBucketSize = 512;
        if (tess->alloc->meshFaceBucketSize == 0)
            tess->alloc->meshFaceBucketSize = 256;
        if (tess->alloc->dictNodeBucketSize == 0)
            tess->alloc->dictNodeBucketSize = 512;
        if (tess->alloc->regionBucketSize == 0)
            tess->alloc->regionBucketSize = 256;
        
        tess->bmin[0] = MAXFLOAT;
        tess->bmin[1] = MAXFLOAT;
        tess->bmax[0] = -MAXFLOAT;
        tess->bmax[1] = -MAXFLOAT;
        
        tess->windingRule = TESS_WINDING_ODD;
        
        if (tess->alloc->regionBucketSize < 16)
            tess->alloc->regionBucketSize = 16;
        if (tess->alloc->regionBucketSize > 4096)
            tess->alloc->regionBucketSize = 4096;
        tess->regionPool = createBucketAlloc( tess->alloc, "Regions",
                                             sizeof(ActiveRegion), tess->alloc->regionBucketSize );
        
        // Initialize to begin polygon.
        tess->mesh = NULL;
        
        tess->vertexIndexCounter = 0;
        
        tess->vertices = 0;
        tess->vertexIndices = 0;
        tess->vertexCount = 0;
        tess->elements = 0;
        tess->elementCount = 0;
        
        return tess;
    }
    
    void tessDeleteTess( TESStesselator *tess )
    {
        
        struct TESSalloc alloc = *tess->alloc;
        
        deleteBucketAlloc( tess->regionPool );
        
        if( tess->mesh != NULL ) {
            tessMeshDeleteMesh( &alloc, tess->mesh );
            tess->mesh = NULL;
        }
        if (tess->vertices != NULL) {
            alloc.memfree( alloc.userData, tess->vertices );
            tess->vertices = 0;
        }
        if (tess->vertexIndices != NULL) {
            alloc.memfree( alloc.userData, tess->vertexIndices );
            tess->vertexIndices = 0;
        }
        if (tess->elements != NULL) {
            alloc.memfree( alloc.userData, tess->elements );
            tess->elements = 0;
        }
        
        alloc.memfree( alloc.userData, tess );
    }
    
    
    static TESSindex GetNeighbourFace(TESShalfEdge* edge)
    {
        if (!edge->Rface)
            return TESS_UNDEF;
        if (!edge->Rface->inside)
            return TESS_UNDEF;
        return edge->Rface->n;
    }
    
    void OutputPolymesh( TESStesselator *tess, TESSmesh *mesh, int elementType, int polySize )
    {
        TESSvertex* v = 0;
        TESSface* f = 0;
        TESShalfEdge* edge = 0;
        int maxFaceCount = 0;
        int maxVertexCount = 0;
        int faceVerts, i;
        TESSindex *elements = 0;
        TESSreal *vert;
        
        // Assume that the input data is triangles now.
        // Try to merge as many polygons as possible
        if (polySize > 3)
        {
            tessMeshMergeConvexFaces( mesh, polySize );
        }
        
        // Mark unused
        for ( v = mesh->vHead.next; v != &mesh->vHead; v = v->next )
            v->n = TESS_UNDEF;
        
        // Create unique IDs for all vertices and faces.
        for ( f = mesh->fHead.next; f != &mesh->fHead; f = f->next )
        {
            f->n = TESS_UNDEF;
            if( !f->inside ) continue;
            
            edge = f->anEdge;
            faceVerts = 0;
            do
            {
                v = edge->Org;
                if ( v->n == TESS_UNDEF )
                {
                    v->n = maxVertexCount;
                    maxVertexCount++;
                }
                faceVerts++;
                edge = edge->Lnext;
            }
            while (edge != f->anEdge);
            
            assert( faceVerts <= polySize );
            
            f->n = maxFaceCount;
            ++maxFaceCount;
        }
        
        tess->elementCount = maxFaceCount;
        if (elementType == TESS_CONNECTED_POLYGONS)
            maxFaceCount *= 2;
        tess->elements = (TESSindex*)tess->alloc->memalloc( tess->alloc->userData,
                                                           sizeof(TESSindex) * maxFaceCount * polySize );
        tess->vertexCount = maxVertexCount;
        tess->vertices = (TESSreal*)tess->alloc->memalloc( tess->alloc->userData,
                                                          sizeof(TESSreal) * tess->vertexCount * 2 );
        tess->vertexIndices = (TESSindex*)tess->alloc->memalloc( tess->alloc->userData,
                                                                sizeof(TESSindex) * tess->vertexCount );
        
        // Output vertices.
        for ( v = mesh->vHead.next; v != &mesh->vHead; v = v->next )
        {
            if ( v->n != TESS_UNDEF )
            {
                // Store coordinate
                vert = &tess->vertices[v->n*2];
                vert[0] = v->s;
                vert[1] = v->t;
                // Store vertex index.
                tess->vertexIndices[v->n] = v->idx;
            }
        }
        
        // Output indices.
        elements = tess->elements;
        for ( f = mesh->fHead.next; f != &mesh->fHead; f = f->next )
        {
            if ( !f->inside ) continue;
            
            // Store polygon
            edge = f->anEdge;
            faceVerts = 0;
            do
            {
                v = edge->Org;
                *elements++ = v->n;
                faceVerts++;
                edge = edge->Lnext;
            }
            while (edge != f->anEdge);
            // Fill unused.
            for (i = faceVerts; i < polySize; ++i)
                *elements++ = TESS_UNDEF;
            
            // Store polygon connectivity
            if ( elementType == TESS_CONNECTED_POLYGONS )
            {
                edge = f->anEdge;
                do
                {
                    *elements++ = GetNeighbourFace( edge );
                    edge = edge->Lnext;
                }
                while (edge != f->anEdge);
                // Fill unused.
                for (i = faceVerts; i < polySize; ++i)
                    *elements++ = TESS_UNDEF;
            }
        }
    }
    
    void OutputContours( TESStesselator *tess, TESSmesh *mesh )
    {
        TESSface *f = 0;
        TESShalfEdge *edge = 0;
        TESShalfEdge *start = 0;
        TESSreal *verts = 0;
        TESSindex *elements = 0;
        TESSindex *vertInds = 0;
        int startVert = 0;
        int vertCount = 0;
        
        tess->vertexCount = 0;
        tess->elementCount = 0;
        
        for ( f = mesh->fHead.next; f != &mesh->fHead; f = f->next )
        {
            if ( !f->inside ) continue;
            
            start = edge = f->anEdge;
            do
            {
                ++tess->vertexCount;
                edge = edge->Lnext;
            }
            while ( edge != start );
            
            ++tess->elementCount;
        }
        
        tess->elements = (TESSindex*)tess->alloc->memalloc( tess->alloc->userData,
                                                           sizeof(TESSindex) * tess->elementCount * 2 );
        tess->vertices = (TESSreal*)tess->alloc->memalloc( tess->alloc->userData,
                                                          sizeof(TESSreal) * tess->vertexCount * 2 );
        tess->vertexIndices = (TESSindex*)tess->alloc->memalloc( tess->alloc->userData,
                                                                sizeof(TESSindex) * tess->vertexCount );
        
        verts = tess->vertices;
        elements = tess->elements;
        vertInds = tess->vertexIndices;
        
        startVert = 0;
        
        for ( f = mesh->fHead.next; f != &mesh->fHead; f = f->next )
        {
            if ( !f->inside ) continue;
            
            vertCount = 0;
            start = edge = f->anEdge;
            do
            {
                *verts++ = edge->Org->s;
                *verts++ = edge->Org->t;
                *vertInds++ = edge->Org->idx;
                ++vertCount;
                edge = edge->Lnext;
            }
            while ( edge != start );
            
            elements[0] = startVert;
            elements[1] = vertCount;
            elements += 2;
            
            startVert += vertCount;
        }
    }
    
    void tessBeginContour( TESStesselator *tess )
    {
        if ( tess->mesh == NULL )
            tess->mesh = tessMeshNewMesh( tess->alloc );
        
        tess->e = NULL;
    }
    
    void tessAddVertex( TESStesselator *tess, TESSreal x, TESSreal y )
    {
        if( tess->e == NULL ) {
            /* Make a self-loop (one vertex, one edge). */
            tess->e = tessMeshMakeEdge( tess->mesh );
            tessMeshSplice( tess->mesh, tess->e, tess->e->Sym );
        } else {
            /* Create a new vertex and edge which immediately follow tess->e
             * in the ordering around the left face.
             */
            tessMeshSplitEdge( tess->mesh, tess->e );
            tess->e = tess->e->Lnext;
        }
        
        /* The new vertex is now tess->e->Org. */
        tess->e->Org->s = x;
        tess->e->Org->t = y;
        if (tess->bmin[0] > x) tess->bmin[0] = x;
        if (tess->bmin[1] > y) tess->bmin[1] = y;
        if (tess->bmax[0] < x) tess->bmax[0] = x;
        if (tess->bmax[1] < y) tess->bmax[1] = y;
        
        /* Store the insertion number so that the vertex can be later recognized. */
        tess->e->Org->idx = tess->vertexIndexCounter++;
        
        /* The winding of an edge says how the winding number changes as we
         * cross from the edge''s right face to its left face.  We add the
         * vertices in such an order that a CCW contour will add +1 to
         * the winding number of the region inside the contour.
         */
        tess->e->winding = 1;
        tess->e->Sym->winding = -1;
    }
    
    void tessAddContour( TESStesselator *tess, int size, const void* vertices,
                        int stride, int numVertices )
    {
        const unsigned char *src = (const unsigned char*)vertices;
        int i;
        
        tessBeginContour(tess);
        
        for( i = 0; i < numVertices; ++i )
        {
            const TESSreal* coords = (const TESSreal*)src;
            src += stride;
            tessAddVertex(tess, coords[0], coords[1]);
        }
    }
    
    void tessTesselate( TESStesselator *tess, int windingRule, int elementType,
                      int polySize, const TESSreal* normal )
    {
        TESSmesh *mesh;
        
        if (tess->vertices != NULL) {
            tess->alloc->memfree( tess->alloc->userData, tess->vertices );
            tess->vertices = 0;
        }
        if (tess->elements != NULL) {
            tess->alloc->memfree( tess->alloc->userData, tess->elements );
            tess->elements = 0;
        }
        if (tess->vertexIndices != NULL) {
            tess->alloc->memfree( tess->alloc->userData, tess->vertexIndices );
            tess->vertexIndices = 0;
        }
        
        tess->vertexIndexCounter = 0;
        tess->windingRule = windingRule;
        
        /* tessComputeInterior( tess ) computes the planar arrangement specified
         * by the given contours, and further subdivides this arrangement
         * into regions.  Each region is marked "inside" if it belongs
         * to the polygon, according to the rule given by tess->windingRule.
         * Each interior region is guaranteed be monotone.
         */
        tessComputeInterior( tess );
        
        mesh = tess->mesh;
        
        /* If the user wants only the boundary contours, we throw away all edges
         * except those which separate the interior from the exterior.
         * Otherwise we tessellate all the regions marked "inside".
         */
        if (elementType == TESS_BOUNDARY_CONTOURS) {
            tessMeshSetWindingNumber( mesh, 1, TRUE );
        } else {
            tessMeshTessellateInterior( mesh );
            if (elementType == TESS_CONSTRAINED_DELAUNAY_TRIANGLES) {
                tessMeshRefineDelaunay( mesh, tess->alloc );
                elementType = TESS_POLYGONS;
                polySize = 3;
            }
        }
        
        tessMeshCheckMesh( mesh );
        
        if (elementType == TESS_BOUNDARY_CONTOURS) {
            OutputContours( tess, mesh );     /* output contours */
        }
        else
        {
            OutputPolymesh( tess, mesh, elementType, polySize );     /* output polygons */
        }
        
        tessMeshDeleteMesh( tess->alloc, mesh );
        tess->mesh = NULL;
    }
    
    int tessGetVertexCount( TESStesselator *tess )
    {
        return tess->vertexCount;
    }
    
    const TESSreal* tessGetVertices( TESStesselator *tess )
    {
        return tess->vertices;
    }
    
    const TESSindex* tessGetVertexIndices( TESStesselator *tess )
    {
        return tess->vertexIndices;
    }
    
    int tessGetElementCount( TESStesselator *tess )
    {
        return tess->elementCount;
    }
    
    const int* tessGetElements( TESStesselator *tess )
    {
        return tess->elements;
    }
}
