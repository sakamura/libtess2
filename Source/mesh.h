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

#pragma once

#include "tess.h"
#include "bucketalloc.h"

namespace Tess
{
	/* The mesh structure is similar in spirit, notation, and operations
	 * to the "quad-edge" structure (see L. Guibas and J. Stolfi, Primitives
	 * for the manipulation of general subdivisions and the computation of
	 * Voronoi diagrams, ACM Transactions on Graphics, 4(2):74-123, April 1985).
	 * For a simplified description, see the course notes for CS348a,
	 * "Mathematical Foundations of Computer Graphics", available at the
	 * Stanford bookstore (and taught during the fall quarter).
	 * The implementation also borrows a tiny subset of the graph-based approach
	 * use in Mantyla's Geometric Work Bench (see M. Mantyla, An Introduction
	 * to Sold Modeling, Computer Science Press, Rockville, Maryland, 1988).
	 *
	 * The fundamental data structure is the "half-edge".  Two half-edges
	 * go together to make an edge, but they point in opposite directions.
	 * Each half-edge has a pointer to its mate (the "symmetric" half-edge Sym),
	 * its origin vertex (Org), the face on its left side (Lface), and the
	 * adjacent half-edges in the CCW direction around the origin vertex
	 * (Onext) and around the left face (Lnext).  There is also a "next"
	 * pointer for the global edge list (see below).
	 *
	 * The notation used for mesh navigation:
	 *	Sym	  = the mate of a half-edge (same edge, but opposite direction)
	 *	Onext = edge CCW around origin vertex (keep same origin)
	 *	Dnext = edge CCW around destination vertex (keep same dest)
	 *	Lnext = edge CCW around left face (dest becomes new origin)
	 *	Rnext = edge CCW around right face (origin becomes new dest)
	 *
	 * "prev" means to substitute CW for CCW in the definitions above.
	 *
	 * The mesh keeps global lists of all vertices, faces, and edges,
	 * stored as doubly-linked circular lists with a dummy header node.
	 * The mesh stores pointers to these dummy headers (vHead, fHead, eHead).
	 *
	 * The circular edge list is special; since half-edges always occur
	 * in pairs (e and e->Sym), each half-edge stores a pointer in only
	 * one direction.  Starting at eHead and following the e->next pointers
	 * will visit each *edge* once (ie. e or e->Sym, but not both).
	 * e->Sym stores a pointer in the opposite direction, thus it is
	 * always true that e->Sym->next->Sym->next == e.
	 *
	 * Each vertex has a pointer to next and previous vertices in the
	 * circular list, and a pointer to a half-edge with this vertex as
	 * the origin (nullptr if this is the dummy header).  There is also a
	 * field "data" for client data.
	 *
	 * Each face has a pointer to the next and previous faces in the
	 * circular list, and a pointer to a half-edge with this face as
	 * the left face (nullptr if this is the dummy header).	 There is also
	 * a field "data" for client data.
	 *
	 * Note that what we call a "face" is really a loop; faces may consist
	 * of more than one loop (ie. not simply connected), but there is no
	 * record of this in the data structure.  The mesh may consist of
	 * several disconnected regions, so it may not be possible to visit
	 * the entire mesh by starting at a half-edge and traversing the edge
	 * structure.
	 *
	 * The mesh does NOT support isolated vertices; a vertex is deleted along
	 * with its last edge.	Similarly when two faces are merged, one of the
	 * faces is deleted (see tessMeshDelete below).	 For mesh operations,
	 * all face (loop) and vertex pointers must not be nullptr.	 However, once
	 * mesh manipulation is finished, TESSmeshZapFace can be used to delete
	 * faces of the mesh, one at a time.  All external faces can be "zapped"
	 * before the mesh is returned to the client; then a nullptr face indicates
	 * a region which is not part of the output polygon.
	 */
	
    template <typename Options, typename Allocators>
	struct VertexT {
        using Tesselator = Tess::Tesselator<Options, Allocators>;
        using Vertex = VertexT<Options, Allocators>;
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        
		/* makeVertex( newVertex, eOrig, vNext ) attaches a new vertex and makes it the
		 * origin of all edges in the vertex loop to which eOrig belongs. "vNext" gives
		 * a place to insert the new vertex in the global vertex list.	We insert
		 * the new vertex *before* vNext so that algorithms which walk the vertex
		 * list will not see the newly created vertices.
		 */
		static Vertex* makeVertex( Tesselator* t, HalfEdge *eOrig, Vertex *vNext );

		Vertex *next;	   /* next vertex (never nullptr) */
		Vertex *prev;	   /* previous vertex (never nullptr) */
		HalfEdge *anEdge;	 /* a half-edge with this origin */
		
		/* Internal data (keep hidden) */
		float s, t;		  /* projection onto the sweep plane */
		int pqHandle;	/* to allow deletion from priority queue */
		int n;			/* to allow identify unique vertices */
		int idx;			/* to allow map result to original verts */
    };
	
    template <typename Options, typename Allocators>
	struct FaceT {
        using Face = FaceT<Options, Allocators>;
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        
		Face *next;		 /* next face (never nullptr) */
		Face *prev;		 /* previous face (never nullptr) */
		HalfEdge *anEdge;	 /* a half edge with this left face */
		
		/* Internal data (keep hidden) */
		Face *trail;	 /* "stack" for conversion to strips */
		int n;		/* to allow identiy unique faces */
		char marked;	 /* flag for conversion to strips */
		char inside;	 /* this face is in the polygon interior */
    };
	
    template <typename Options, typename Allocators>
	struct HalfEdgeT {
        using Mesh = MeshT<Options, Allocators>;
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        using EdgePair = EdgePairT<Options, Allocators>;
        using Vertex = VertexT<Options, Allocators>;
        using Face = FaceT<Options, Allocators>;
        using ActiveRegion = ActiveRegionT<Options, Allocators>;
        
		HalfEdgeT() :
			next_(nullptr),
			Sym_(nullptr),
			Onext_(nullptr),
			Lnext_(nullptr),
			Org_(nullptr),
			Lface_(nullptr),
			activeRegion_(nullptr),
			winding_(0),
			mark_(0)
		{
		}
		
		/* makeEdge creates a new pair of half-edges which form their own loop.
		 * No vertex or face structures are allocated, but these must be assigned
		 * before the current edge operation is completed.
		 */
		static HalfEdge *makeEdge( Mesh* mesh, HalfEdge *eNext );
		
		/* splice( a, b ) is best described by the Guibas/Stolfi paper or the
		 * CS348a notes (see mesh.h).  Basically it modifies the mesh so that
		 * a->Onext and b->Onext are exchanged.	 This can have various effects
		 * depending on whether a and b belong to different face or vertex rings.
		 * For more explanation see tessMeshSplice() below.
		 */
		static void splice( HalfEdge *a, HalfEdge *b );

		/* When we merge two edges into one, we need to compute the combined
		 * winding of the new edge.
		 */
		void addWinding( HalfEdge *src )
		{
			winding_ += src->winding_;
			Sym()->winding_ += src->Sym()->winding_;
		}

		HalfEdge *next() { return next_; }
		const HalfEdge *next() const { return next_; }
		void setNext(HalfEdge* next) { next_ = next; }
		HalfEdge *Sym() { return Sym_; }
		const HalfEdge *Sym() const { return Sym_; }
		void setSym(HalfEdge* sym) { Sym_ = sym; }
		HalfEdge *Onext() { return Onext_; }
		const HalfEdge *Onext() const { return Onext_; }
		void setOnext(HalfEdge *Onext) { Onext_ = Onext; }
		HalfEdge *Lnext() { return Lnext_; }
		const HalfEdge *Lnext() const { return Lnext_; }
		void setLnext(HalfEdge *Lnext) { Lnext_ = Lnext; }
		Vertex *Org() { return Org_; }
		const Vertex *Org() const { return Org_; }
		void setOrg(Vertex* org) { Org_ = org; }
		Face *Lface() { return Lface_; }
		const Face *Lface() const { return Lface_; }
		void setLface(Face *lface) { Lface_ = lface; }
		
		Face *Rface() { return Sym()->Lface(); }
		const Face *Rface() const { return Sym()->Lface(); }
		void setRface(Face *rface) { Sym()->setLface(rface); }
		Vertex *Dst() { return Sym()->Org(); }
		const Vertex *Dst() const { return Sym()->Org(); }
		void setDst(Vertex* Dst) { Sym()->setOrg(Dst); }
		
		HalfEdge *Oprev() { return Sym()->Lnext(); }
		const HalfEdge *Oprev() const { return Sym()->Lnext(); }
		HalfEdge *Lprev() { return Onext()->Sym(); }
		const HalfEdge *Lprev() const { return Onext()->Sym(); }
		HalfEdge *Dprev() { return Lnext()->Sym(); }
		const HalfEdge *Dprev() const { return Lnext()->Sym(); }
		HalfEdge *Rprev() { return Sym()->Onext(); }
		const HalfEdge *Rprev() const { return Sym()->Onext(); }
		HalfEdge *Dnext() { return Rprev()->Sym(); }  /* 3 pointers */
		const HalfEdge *Dnext() const { return Rprev()->Sym(); }  /* 3 pointers */
		HalfEdge *Rnext() { return Oprev()->Sym(); }  /* 3 pointers */
		const HalfEdge *Rnext() const { return Oprev()->Sym(); }  /* 3 pointers */
		
		ActiveRegion *activeRegion() { return activeRegion_; }
		const ActiveRegion *activeRegion() const { return activeRegion_; }
		void resetActiveRegion() { activeRegion_ = nullptr; }
		void setActiveRegion(ActiveRegion * newActiveRegion) { activeRegion_ = newActiveRegion; }
		const int winding() const { return winding_; }
		void setWinding(int winding) { winding_ = winding; }
		const int mark() const { return mark_; }
		void setMark(int mark) { mark_ = mark; }

		HalfEdge *next_;	  /* doubly-linked list (prev==Sym->next) */
		HalfEdge *Sym_;		  /* same edge, opposite direction */
		HalfEdge *Onext_;	  /* next edge CCW around origin */
		HalfEdge *Lnext_;	  /* next edge CCW around left face */
		Vertex *Org_;		/* origin vertex (Overtex too long) */
		Face *Lface_;	  /* left face */
		
		/* Internal data (keep hidden) */
		ActiveRegion *activeRegion_;  /* a region with this upper edge (sweep.c) */
		int winding_;	 /* change in winding number when crossing
						 from the right face to the left face */
		int mark_; /* Used by the Edge Flip algorithm */
	};
    
    template <typename Options, typename Allocators>
	struct EdgePairT : public HalfEdgeT<Options, Allocators>
	{
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        
		HalfEdge eSym;
	};
	
    template<typename Options, typename Allocators>
	class MeshT {
        using Tesselator = Tess::Tesselator<Options, Allocators>;
        using Mesh = MeshT<Options, Allocators>;
        using Vertex = VertexT<Options, Allocators>;
        using Face = FaceT<Options, Allocators>;
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        using EdgePair = EdgePairT<Options, Allocators>;
        
		Vertex vHead;	   /* dummy header for vertex list */
		Face fHead;		 /* dummy header for face list */
		HalfEdge eHead;		 /* dummy header for edge list */
		HalfEdge eHeadSym;	 /* and its symmetric counterpart */
		
	public:
        Tesselator* t;

        MeshT(Tesselator* t);
		~MeshT();

		HalfEdge *makeEdge( );
		void splice( HalfEdge *eOrg, HalfEdge *eDst );
		void remove( HalfEdge *eDel );
		
		HalfEdge *addEdgeVertex( HalfEdge *eOrg );
		HalfEdge *splitEdge( HalfEdge *eOrg );
		HalfEdge *connect( HalfEdge *eOrg, HalfEdge *eDst );
		
		static Mesh *merge( Mesh *mesh1, Mesh *mesh2 );
		void mergeConvexFaces( int maxVertsPerFace );
		void zapFace( Face *fZap );
		
		void flipEdge( HalfEdge *edge );
		
		void checkMesh( );
		
		Vertex* vBegin() { return vHead.next; }
		Vertex* vEnd() { return &vHead; }
		Face* fBegin() { return fHead.next; }
		Face* fEnd() { return &fHead; }
		HalfEdge* eBegin() { return eHead.next(); }
		HalfEdge* eEnd() { return &eHead; }
		HalfEdge* eSymBegin() { return eHeadSym.next(); }
		HalfEdge* eSymEnd() { return &eHeadSym; }
		
		void meshUnion( Mesh *meshToMerge );
	};
	
	/* The mesh operations below have three motivations: completeness,
	 * convenience, and efficiency.	 The basic mesh operations are makeEdge,
	 * splice, and Delete.	All the other edge operations can be implemented
	 * in terms of these.  The other operations are provided for convenience
	 * and/or efficiency.
	 *
	 * When a face is split or a vertex is added, they are inserted into the
	 * global list *before* the existing vertex or face (ie. e->Org or e->Lface).
	 * This makes it easier to process all vertices or faces in the global lists
	 * without worrying about processing the same data twice.  As a convenience,
	 * when a face is split, the "inside" flag is copied from the old face.
	 * Other internal data (v->data, v->activeRegion, f->data, f->marked,
	 * f->trail, e->winding) is set to zero.
	 *
	 * ********************** Basic Edge Operations **************************
	 *
	 * tessMeshMakeEdge( mesh ) creates one edge, two vertices, and a loop.
	 * The loop (face) consists of the two new half-edges.
	 *
	 * tessMeshSplice( eOrg, eDst ) is the basic operation for changing the
	 * mesh connectivity and topology.	It changes the mesh so that
	 *	eOrg->Onext <- OLD( eDst->Onext )
	 *	eDst->Onext <- OLD( eOrg->Onext )
	 * where OLD(...) means the value before the meshSplice operation.
	 *
	 * This can have two effects on the vertex structure:
	 *	- if eOrg->Org != eDst->Org, the two vertices are merged together
	 *	- if eOrg->Org == eDst->Org, the origin is split into two vertices
	 * In both cases, eDst->Org is changed and eOrg->Org is untouched.
	 *
	 * Similarly (and independently) for the face structure,
	 *	- if eOrg->Lface == eDst->Lface, one loop is split into two
	 *	- if eOrg->Lface != eDst->Lface, two distinct loops are joined into one
	 * In both cases, eDst->Lface is changed and eOrg->Lface is unaffected.
	 *
	 * tessMeshDelete( eDel ) removes the edge eDel.  There are several cases:
	 * if (eDel->Lface != eDel->Rface), we join two loops into one; the loop
	 * eDel->Lface is deleted.	Otherwise, we are splitting one loop into two;
	 * the newly created loop will contain eDel->Dst.  If the deletion of eDel
	 * would create isolated vertices, those are deleted as well.
	 *
	 * ********************** Other Edge Operations **************************
	 *
	 * tessMeshAddEdgeVertex( eOrg ) creates a new edge eNew such that
	 * eNew == eOrg->Lnext, and eNew->Dst is a newly created vertex.
	 * eOrg and eNew will have the same left face.
	 *
	 * tessMeshSplitEdge( eOrg ) splits eOrg into two edges eOrg and eNew,
	 * such that eNew == eOrg->Lnext.  The new vertex is eOrg->Dst == eNew->Org.
	 * eOrg and eNew will have the same left face.
	 *
	 * tessMeshConnect( eOrg, eDst ) creates a new edge from eOrg->Dst
	 * to eDst->Org, and returns the corresponding half-edge eNew.
	 * If eOrg->Lface == eDst->Lface, this splits one loop into two,
	 * and the newly created loop is eNew->Lface.  Otherwise, two disjoint
	 * loops are merged into one, and the loop eDst->Lface is destroyed.
	 *
	 * ************************ Other Operations *****************************
	 *
	 * tessMeshNewMesh() creates a new mesh with no edges, no vertices,
	 * and no loops (what we usually call a "face").
	 *
	 * tessMeshUnion( mesh1, mesh2 ) forms the union of all structures in
	 * both meshes, and returns the new mesh (the old meshes are destroyed).
	 *
	 * tessMeshDeleteMesh( mesh ) will free all storage for any valid mesh.
	 *
	 * tessMeshZapFace( fZap ) destroys a face and removes it from the
	 * global face list.  All edges of fZap will have a nullptr pointer as their
	 * left face.  Any edges which also have a nullptr pointer as their right face
	 * are deleted entirely (along with any isolated vertices this produces).
	 * An entire mesh can be deleted by zapping its faces, one at a time,
	 * in any order.  Zapped faces cannot be used in further mesh operations!
	 *
	 * tessMeshCheckMesh( mesh ) checks a mesh for self-consistency.
	 */
}

#include "mesh.inl"
