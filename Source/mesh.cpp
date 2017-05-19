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

//#include "tesos.h"
#include <stddef.h>
#include <assert.h>
#include "mesh.h"
#include "geom.h"
#include "bucketalloc.h"

namespace Tess
{
    /************************ Utility Routines ************************/
    /* MakeEdge creates a new pair of half-edges which form their own loop.
     * No vertex or face structures are allocated, but these must be assigned
     * before the current edge operation is completed.
     */
    TESShalfEdge *TESShalfEdge::MakeEdge( TESSmesh* mesh, TESShalfEdge *eNext )
    {
        TESShalfEdge *e;
        TESShalfEdge *eSym;
        TESShalfEdge *ePrev;
        EdgePair *pair = new EdgePair;
        
        e = pair;
        eSym = &pair->eSym;
        
        /* Make sure eNext points to the first edge of the edge pair */
        if( eNext->Sym() < eNext ) { eNext = eNext->Sym(); }
        
        /* Insert in circular doubly-linked list before eNext.
         * Note that the prev pointer is stored in Sym->next.
         */
        ePrev = eNext->Sym()->next();
        eSym->next_ = ePrev;
        ePrev->Sym()->next_ = e;
        e->next_ = eNext;
        eNext->Sym()->next_ = eSym;
        
        e->Sym_ = eSym;
        e->Onext_ = e;
        e->Lnext_ = eSym;
        e->Org_ = nullptr;
        e->Lface_ = nullptr;
        e->winding_ = 0;
        e->activeRegion_ = nullptr;
        e->mark_ = 0;
        
        eSym->Sym_ = e;
        eSym->Onext_ = eSym;
        eSym->Lnext_ = e;
        eSym->Org_ = nullptr;
        eSym->Lface_ = nullptr;
        eSym->winding_ = 0;
        eSym->activeRegion_ = nullptr;
        eSym->mark_ = 0;
        
        return e;
    }
    
    /* Splice( a, b ) is best described by the Guibas/Stolfi paper or the
     * CS348a notes (see mesh.h).  Basically it modifies the mesh so that
     * a->Onext and b->Onext are exchanged.  This can have various effects
     * depending on whether a and b belong to different face or vertex rings.
     * For more explanation see tessMeshSplice() below.
     */
    void TESShalfEdge::Splice( TESShalfEdge *a, TESShalfEdge *b )
    {
        TESShalfEdge *aOnext = a->Onext();
        TESShalfEdge *bOnext = b->Onext();
        
        aOnext->Sym()->Lnext_ = b;
        bOnext->Sym()->Lnext_ = a;
        a->Onext_ = bOnext;
        b->Onext_ = aOnext;
    }
    
    /* MakeVertex( newVertex, eOrig, vNext ) attaches a new vertex and makes it the
     * origin of all edges in the vertex loop to which eOrig belongs. "vNext" gives
     * a place to insert the new vertex in the global vertex list.  We insert
     * the new vertex *before* vNext so that algorithms which walk the vertex
     * list will not see the newly created vertices.
     */
    TESSvertex* TESSvertex::MakeVertex( TESShalfEdge *eOrig, TESSvertex *vNext )
    {
        TESShalfEdge *e;
        TESSvertex *vPrev;
        TESSvertex *vNew = new TESSvertex;
        
        assert(vNew != nullptr);
        
        /* insert in circular doubly-linked list before vNext */
        vPrev = vNext->prev;
        vNew->prev = vPrev;
        vPrev->next = vNew;
        vNew->next = vNext;
        vNext->prev = vNew;
        
        vNew->anEdge = eOrig;
        
        /* fix other edges on this vertex loop */
        e = eOrig;
        do {
            e->SetOrg(vNew);
            e = e->Onext();
        } while( e != eOrig );

        return vNew;
    }
    
    /* MakeFace( newFace, eOrig, fNext ) attaches a new face and makes it the left
     * face of all edges in the face loop to which eOrig belongs.  "fNext" gives
     * a place to insert the new face in the global face list.  We insert
     * the new face *before* fNext so that algorithms which walk the face
     * list will not see the newly created faces.
     */
    static void MakeFace( TESSface *newFace, TESShalfEdge *eOrig, TESSface *fNext )
    {
        TESShalfEdge *e;
        TESSface *fPrev;
        TESSface *fNew = newFace;
        
        assert(fNew != nullptr);
        
        /* insert in circular doubly-linked list before fNext */
        fPrev = fNext->prev;
        fNew->prev = fPrev;
        fPrev->next = fNew;
        fNew->next = fNext;
        fNext->prev = fNew;
        
        fNew->anEdge = eOrig;
        fNew->trail = nullptr;
        fNew->marked = false;
        
        /* The new face is marked "inside" if the old one was.  This is a
         * convenience for the common case where a face has been split in two.
         */
        fNew->inside = fNext->inside;
        
        /* fix other edges on this face loop */
        e = eOrig;
        do {
            e->SetLface(fNew);
            e = e->Lnext();
        } while( e != eOrig );
    }
    
    /* KillEdge( eDel ) destroys an edge (the half-edges eDel and eDel->Sym),
     * and removes from the global edge list.
     */
    static void KillEdge( TESSmesh *mesh, TESShalfEdge *eDel )
    {
        TESShalfEdge *ePrev, *eNext;
        
        /* Half-edges are allocated in pairs, see EdgePair above */
        if( eDel->Sym() < eDel ) { eDel = eDel->Sym(); }
        
        /* delete from circular doubly-linked list */
        eNext = eDel->next();
        ePrev = eDel->Sym()->next();
        eNext->Sym()->SetNext(ePrev);
        ePrev->Sym()->SetNext(eNext);
        delete reinterpret_cast<EdgePair*>(eDel);
    }
    
    
    /* KillVertex( vDel ) destroys a vertex and removes it from the global
     * vertex list.  It updates the vertex loop to point to a given new vertex.
     */
    static void KillVertex( TESSmesh *mesh, TESSvertex *vDel, TESSvertex *newOrg )
    {
        TESShalfEdge *e, *eStart = vDel->anEdge;
        TESSvertex *vPrev, *vNext;
        
        /* change the origin of all affected edges */
        e = eStart;
        do {
            e->SetOrg(newOrg);
            e = e->Onext();
        } while( e != eStart );
        
        /* delete from circular doubly-linked list */
        vPrev = vDel->prev;
        vNext = vDel->next;
        vNext->prev = vPrev;
        vPrev->next = vNext;
        delete vDel;
    }
    
    /* KillFace( fDel ) destroys a face and removes it from the global face
     * list.  It updates the face loop to point to a given new face.
     */
    static void KillFace( TESSmesh *mesh, TESSface *fDel, TESSface *newLface )
    {
        TESShalfEdge *e, *eStart = fDel->anEdge;
        TESSface *fPrev, *fNext;
        
        /* change the left face of all affected edges */
        e = eStart;
        do {
            e->SetLface(newLface);
            e = e->Lnext();
        } while( e != eStart );
        
        /* delete from circular doubly-linked list */
        fPrev = fDel->prev;
        fNext = fDel->next;
        fNext->prev = fPrev;
        fPrev->next = fNext;
        delete fDel;
    }
    
    
    /****************** Basic Edge Operations **********************/
    
    /* tessMeshMakeEdge creates one edge, two vertices, and a loop (face).
     * The loop consists of the two new half-edges.
     */
    TESShalfEdge *TESSmesh::makeEdge( )
    {
        TESSface *newFace = new TESSface;
        TESShalfEdge *e = EdgePair::MakeEdge( this, &eHead );

        TESSvertex::MakeVertex( e, &vHead );
        TESSvertex::MakeVertex( e->Sym(), &vHead );

        MakeFace( newFace, e, &fHead );
        return e;
    }
    
    
    /* tessMeshSplice( eOrg, eDst ) is the basic operation for changing the
     * mesh connectivity and topology.  It changes the mesh so that
     *	eOrg->Onext <- OLD( eDst->Onext )
     *	eDst->Onext <- OLD( eOrg->Onext )
     * where OLD(...) means the value before the meshSplice operation.
     *
     * This can have two effects on the vertex structure:
     *  - if eOrg->Org != eDst->Org, the two vertices are merged together
     *  - if eOrg->Org == eDst->Org, the origin is split into two vertices
     * In both cases, eDst->Org is changed and eOrg->Org is untouched.
     *
     * Similarly (and independently) for the face structure,
     *  - if eOrg->Lface == eDst->Lface, one loop is split into two
     *  - if eOrg->Lface != eDst->Lface, two distinct loops are joined into one
     * In both cases, eDst->Lface is changed and eOrg->Lface is unaffected.
     *
     * Some special cases:
     * If eDst == eOrg, the operation has no effect.
     * If eDst == eOrg->Lnext, the new face will have a single edge.
     * If eDst == eOrg->Lprev, the old face will have a single edge.
     * If eDst == eOrg->Onext, the new vertex will have a single edge.
     * If eDst == eOrg->Oprev, the old vertex will have a single edge.
     */
    void TESSmesh::splice( TESShalfEdge *eOrg, TESShalfEdge *eDst )
    {
        int joiningLoops = false;
        int joiningVertices = false;
        
        assert( eOrg != eDst );
        
        if( eDst->Org() != eOrg->Org() ) {
            /* We are merging two disjoint vertices -- destroy eDst->Org */
            joiningVertices = true;
            KillVertex( this, eDst->Org(), eOrg->Org() );
        }
        if( eDst->Lface() != eOrg->Lface() ) {
            /* We are connecting two disjoint loops -- destroy eDst->Lface */
            joiningLoops = true;
            KillFace( this, eDst->Lface(), eOrg->Lface() );
        }
        
        /* Change the edge structure */
        TESShalfEdge::Splice( eDst, eOrg );
        
        if( ! joiningVertices ) {
            /* We split one vertex into two -- the new vertex is eDst->Org.
             * Make sure the old vertex points to a valid half-edge.
             */
            TESSvertex::MakeVertex( eDst, eOrg->Org() );
            eOrg->Org()->anEdge = eOrg;
        }
        if( ! joiningLoops ) {
            TESSface *newFace = new TESSface;
            
            /* We split one loop into two -- the new loop is eDst->Lface.
             * Make sure the old face points to a valid half-edge.
             */
            MakeFace( newFace, eDst, eOrg->Lface() );
            eOrg->Lface()->anEdge = eOrg;
        }
    }
    
    
    /* tessMeshDelete( eDel ) removes the edge eDel.  There are several cases:
     * if (eDel->Lface != eDel->Rface), we join two loops into one; the loop
     * eDel->Lface is deleted.  Otherwise, we are splitting one loop into two;
     * the newly created loop will contain eDel->Dst.  If the deletion of eDel
     * would create isolated vertices, those are deleted as well.
     *
     * This function could be implemented as two calls to tessMeshSplice
     * plus a few calls to memFree, but this would allocate and delete
     * unnecessary vertices and faces.
     */
    void TESSmesh::remove( TESShalfEdge *eDel )
    {
        TESShalfEdge *eDelSym = eDel->Sym();
        int joiningLoops = false;
        
        /* First step: disconnect the origin vertex eDel->Org.  We make all
         * changes to get a consistent mesh in this "intermediate" state.
         */
        if( eDel->Lface() != eDel->Rface() ) {
            /* We are joining two loops into one -- remove the left face */
            joiningLoops = true;
            KillFace( this, eDel->Lface(), eDel->Rface() );
        }
        
        if( eDel->Onext() == eDel ) {
            KillVertex( this, eDel->Org(), nullptr );
        } else {
            /* Make sure that eDel->Org and eDel->Rface point to valid half-edges */
            eDel->Rface()->anEdge = eDel->Oprev();
            eDel->Org()->anEdge = eDel->Onext();
            
            TESShalfEdge::Splice( eDel, eDel->Oprev() );
            if( ! joiningLoops ) {
                TESSface *newFace= new TESSface;
                
                /* We are splitting one loop into two -- create a new loop for eDel. */
                MakeFace( newFace, eDel, eDel->Lface() );
            }
        }
        
        /* Claim: the mesh is now in a consistent state, except that eDel->Org
         * may have been deleted.  Now we disconnect eDel->Dst.
         */
        if( eDelSym->Onext() == eDelSym ) {
            KillVertex( this, eDelSym->Org(), nullptr );
            KillFace( this, eDelSym->Lface(), nullptr );
        } else {
            /* Make sure that eDel->Dst and eDel->Lface point to valid half-edges */
            eDel->Lface()->anEdge = eDelSym->Oprev();
            eDelSym->Org()->anEdge = eDelSym->Onext();
            TESShalfEdge::Splice( eDelSym, eDelSym->Oprev() );
        }
        
        /* Any isolated vertices or faces have already been freed. */
        KillEdge( this, eDel );
    }
    
    
    /******************** Other Edge Operations **********************/
    
    /* All these routines can be implemented with the basic edge
     * operations above.  They are provided for convenience and efficiency.
     */
    
    
    /* tessMeshAddEdgeVertex( eOrg ) creates a new edge eNew such that
     * eNew == eOrg->Lnext, and eNew->Dst is a newly created vertex.
     * eOrg and eNew will have the same left face.
     */
    TESShalfEdge *TESSmesh::addEdgeVertex( TESShalfEdge *eOrg )
    {
        TESShalfEdge *eNewSym;
        TESShalfEdge *eNew = EdgePair::MakeEdge( this, eOrg );
        
        eNewSym = eNew->Sym();
        
        /* Connect the new edge appropriately */
        TESShalfEdge::Splice( eNew, eOrg->Lnext() );
        
        /* Set the vertex and face information */
        eNew->SetOrg(eOrg->Dst());
        {
            TESSvertex::MakeVertex( eNewSym, eNew->Org() );
        }
        eNewSym->SetLface(eOrg->Lface());
        eNew->SetLface(eOrg->Lface());

        return eNew;
    }
    
    
    /* tessMeshSplitEdge( eOrg ) splits eOrg into two edges eOrg and eNew,
     * such that eNew == eOrg->Lnext.  The new vertex is eOrg->Dst == eNew->Org.
     * eOrg and eNew will have the same left face.
     */
    TESShalfEdge *TESSmesh::splitEdge( TESShalfEdge *eOrg )
    {
        TESShalfEdge *eNew;
        TESShalfEdge *tempHalfEdge= addEdgeVertex( eOrg );
        
        eNew = tempHalfEdge->Sym();
        
        /* Disconnect eOrg from eOrg->Dst and connect it to eNew->Org */
        TESShalfEdge::Splice( eOrg->Sym(), eOrg->Sym()->Oprev() );
        TESShalfEdge::Splice( eOrg->Sym(), eNew );
        
        /* Set the vertex and face information */
        eOrg->SetDst(eNew->Org());
        eNew->Dst()->anEdge = eNew->Sym();	/* may have pointed to eOrg->Sym */
        eNew->SetRface(eOrg->Rface());
        eNew->SetWinding(eOrg->winding());	/* copy old winding information */
        eNew->Sym()->SetWinding(eOrg->Sym()->winding());
        
        return eNew;
    }
    
    
    /* tessMeshConnect( eOrg, eDst ) creates a new edge from eOrg->Dst
     * to eDst->Org, and returns the corresponding half-edge eNew.
     * If eOrg->Lface == eDst->Lface, this splits one loop into two,
     * and the newly created loop is eNew->Lface.  Otherwise, two disjoint
     * loops are merged into one, and the loop eDst->Lface is destroyed.
     *
     * If (eOrg == eDst), the new face will have only two edges.
     * If (eOrg->Lnext == eDst), the old face is reduced to a single edge.
     * If (eOrg->Lnext->Lnext == eDst), the old face is reduced to two edges.
     */
    TESShalfEdge *TESSmesh::connect( TESShalfEdge *eOrg, TESShalfEdge *eDst )
    {
        TESShalfEdge *eNewSym;
        int joiningLoops = false;
        TESShalfEdge *eNew = EdgePair::MakeEdge( this, eOrg );
        
        eNewSym = eNew->Sym();
        
        if( eDst->Lface() != eOrg->Lface() ) {
            /* We are connecting two disjoint loops -- destroy eDst->Lface */
            joiningLoops = true;
            KillFace( this, eDst->Lface(), eOrg->Lface() );
        }
        
        /* Connect the new edge appropriately */
        TESShalfEdge::Splice( eNew, eOrg->Lnext() );
        TESShalfEdge::Splice( eNewSym, eDst );
        
        /* Set the vertex and face information */
        eNew->SetOrg(eOrg->Dst());
        eNewSym->SetOrg(eDst->Org());
        eNew->SetLface(eOrg->Lface());
        eNewSym->SetLface(eOrg->Lface());
        
        /* Make sure the old face points to a valid half-edge */
        eOrg->Lface()->anEdge = eNewSym;
        
        if( ! joiningLoops ) {
            TESSface *newFace= new TESSface;
            
            /* We split one loop into two -- the new loop is eNew->Lface */
            MakeFace( newFace, eNew, eOrg->Lface() );
        }
        return eNew;
    }
    
    
    /******************** Other Operations **********************/
    
    /* tessMeshZapFace( fZap ) destroys a face and removes it from the
     * global face list.  All edges of fZap will have a nullptr pointer as their
     * left face.  Any edges which also have a nullptr pointer as their right face
     * are deleted entirely (along with any isolated vertices this produces).
     * An entire mesh can be deleted by zapping its faces, one at a time,
     * in any order.  Zapped faces cannot be used in further mesh operations!
     */
    void TESSmesh::zapFace( TESSface *fZap )
    {
        TESShalfEdge *eStart = fZap->anEdge;
        TESShalfEdge *e, *eNext, *eSym;
        TESSface *fPrev, *fNext;
        
        /* walk around face, deleting edges whose right face is also nullptr */
        eNext = eStart->Lnext();
        do {
            e = eNext;
            eNext = e->Lnext();
            
            e->SetLface(nullptr);
            if( e->Rface() == nullptr ) {
                /* delete the edge -- see TESSmeshDelete above */
                
                if( e->Onext() == e ) {
                    KillVertex( this, e->Org(), nullptr );
                } else {
                    /* Make sure that e->Org points to a valid half-edge */
                    e->Org()->anEdge = e->Onext();
                    TESShalfEdge::Splice( e, e->Oprev() );
                }
                eSym = e->Sym();
                if( eSym->Onext() == eSym ) {
                    KillVertex( this, eSym->Org(), nullptr );
                } else {
                    /* Make sure that eSym->Org points to a valid half-edge */
                    eSym->Org()->anEdge = eSym->Onext();
                    TESShalfEdge::Splice( eSym, eSym->Oprev() );
                }
                KillEdge( this, e );
            }
        } while( e != eStart );
        
        /* delete from circular doubly-linked list */
        fPrev = fZap->prev;
        fNext = fZap->next;
        fNext->prev = fPrev;
        fPrev->next = fNext;
        delete fZap;
    }
    
    
    /* tessMeshNewMesh() creates a new mesh with no edges, no vertices,
     * and no loops (what we usually call a "face").
     */
    TESSmesh::TESSmesh( )
    {
        TESSvertex *v;
        TESSface *f;
        TESShalfEdge *e;
        TESShalfEdge *eSym;
        
        v = &vHead;
        f = &fHead;
        e = &eHead;
        eSym = &eHeadSym;
        
        v->next = v->prev = v;
        v->anEdge = nullptr;
        
        f->next = f->prev = f;
        f->anEdge = nullptr;
        f->trail = nullptr;
        f->marked = false;
        f->inside = false;
        
        e->SetNext(e);
        e->SetSym(eSym);
        
        eSym->SetNext(eSym);
        eSym->SetSym(e);
    }
    TESSmesh::~TESSmesh( )
    {
    }
    
    
    /* tessMeshUnion( mesh1, mesh2 ) forms the union of all structures in
     * both meshes, and returns the new mesh (the old meshes are destroyed).
     */
    TESSmesh *tessMeshUnion( TESSmesh *mesh1, TESSmesh *mesh2 )
    {
        TESSface *f1 = mesh1->fEnd();
        TESSvertex *v1 = mesh1->vEnd();
        TESShalfEdge *e1 = mesh1->eEnd();
        TESSface *f2 = mesh2->fEnd();
        TESSvertex *v2 = mesh2->vEnd();
        TESShalfEdge *e2 = mesh2->eEnd();
        
        /* Add the faces, vertices, and edges of mesh2 to those of mesh1 */
        if( f2->next != f2 ) {
            f1->prev->next = f2->next;
            f2->next->prev = f1->prev;
            f2->prev->next = f1;
            f1->prev = f2->prev;
        }
        
        if( v2->next != v2 ) {
            v1->prev->next = v2->next;
            v2->next->prev = v1->prev;
            v2->prev->next = v1;
            v1->prev = v2->prev;
        }
        
        if( e2->next() != e2 ) {
            e1->Sym()->next()->Sym()->SetNext(e2->next());
            e2->next()->Sym()->SetNext(e1->Sym()->next());
            e2->Sym()->next()->Sym()->SetNext(e1);
            e1->Sym()->SetNext(e2->Sym()->next());
        }
        delete mesh2;
        return mesh1;
    }
    
    
    static int CountFaceVerts( TESSface *f )
    {
        TESShalfEdge *eCur = f->anEdge;
        int n = 0;
        do
        {
            n++;
            eCur = eCur->Lnext();
        }
        while (eCur != f->anEdge);
        return n;
    }
    
    void tessMeshMergeConvexFaces( TESSmesh *mesh, int maxVertsPerFace )
    {
        TESSface *f;
        TESShalfEdge *eCur, *eNext, *eSym;
        TESSvertex *vStart;
        int curNv, symNv;
        
        for( f = mesh->fBegin(); f != mesh->fEnd(); f = f->next )
        {
            // Skip faces which are outside the result.
            if( !f->inside )
                continue;
            
            eCur = f->anEdge;
            vStart = eCur->Org();
            
            while (1)
            {
                eNext = eCur->Lnext();
                eSym = eCur->Sym();
                
                // Try to merge if the neighbour face is valid.
                if( eSym && eSym->Lface() && eSym->Lface()->inside )
                {
                    // Try to merge the neighbour faces if the resulting polygons
                    // does not exceed maximum number of vertices.
                    curNv = CountFaceVerts( f );
                    symNv = CountFaceVerts( eSym->Lface() );
                    if( (curNv+symNv-2) <= maxVertsPerFace )
                    {
                        // Merge if the resulting poly is convex.
                        if( vertAreCCW( eCur->Lprev()->Org(), eCur->Org(), eSym->Lnext()->Lnext()->Org() ) &&
                           vertAreCCW( eSym->Lprev()->Org(), eSym->Org(), eCur->Lnext()->Lnext()->Org() ) )
                        {
                            eNext = eSym->Lnext();
                            mesh->remove( eSym );
                            eCur = 0;
                        }
                    }
                }
                
                if( eCur && eCur->Lnext()->Org() == vStart )
                    break;
                
                // Continue to next edge.
                eCur = eNext;
            }
        }
    }
    
    void tessMeshFlipEdge( TESSmesh *mesh, TESShalfEdge *edge )
    {
        TESShalfEdge *a0 = edge;
        TESShalfEdge *a1 = a0->Lnext();
        TESShalfEdge *a2 = a1->Lnext();
        TESShalfEdge *b0 = edge->Sym();
        TESShalfEdge *b1 = b0->Lnext();
        TESShalfEdge *b2 = b1->Lnext();
        
        TESSvertex *aOrg = a0->Org();
        TESSvertex *aOpp = a2->Org();
        TESSvertex *bOrg = b0->Org();
        TESSvertex *bOpp = b2->Org();
        
        TESSface *fa = a0->Lface();
        TESSface *fb = b0->Lface();
        
        assert(edgeIsInternal(edge));
        assert(a2->Lnext() == a0);
        assert(b2->Lnext() == b0);
        
        a0->SetOrg( bOpp );
        a0->SetOnext( b1->Sym() );
        b0->SetOrg( aOpp );
        b0->SetOnext( a1->Sym() );
        a2->SetOnext( b0 );
        b2->SetOnext( a0 );
        b1->SetOnext( a2->Sym() );
        a1->SetOnext( b2->Sym() );
        
        a0->SetLnext( a2 );
        a2->SetLnext( b1 );
        b1->SetLnext( a0 );
        
        b0->SetLnext( b2 );
        b2->SetLnext( a1 );
        a1->SetLnext( b0 );
        
        a1->SetLface( fb );
        b1->SetLface( fa );
        
        fa->anEdge = a0;
        fb->anEdge = b0;
        
        if (aOrg->anEdge == a0) aOrg->anEdge = b1;
        if (bOrg->anEdge == b0) bOrg->anEdge = a1;
        
        assert( a0->Lnext()->Onext()->Sym() == a0 );
        assert( a0->Onext()->Sym()->Lnext() == a0 );
        assert( a0->Org()->anEdge->Org() == a0->Org() );
        
        
        assert( a1->Lnext()->Onext()->Sym() == a1 );
        assert( a1->Onext()->Sym()->Lnext() == a1 );
        assert( a1->Org()->anEdge->Org() == a1->Org() );
        
        assert( a2->Lnext()->Onext()->Sym() == a2 );
        assert( a2->Onext()->Sym()->Lnext() == a2 );
        assert( a2->Org()->anEdge->Org() == a2->Org() );
        
        assert( b0->Lnext()->Onext()->Sym() == b0 );
        assert( b0->Onext()->Sym()->Lnext() == b0 );
        assert( b0->Org()->anEdge->Org() == b0->Org() );
        
        assert( b1->Lnext()->Onext()->Sym() == b1 );
        assert( b1->Onext()->Sym()->Lnext() == b1 );
        assert( b1->Org()->anEdge->Org() == b1->Org() );
        
        assert( b2->Lnext()->Onext()->Sym() == b2 );
        assert( b2->Onext()->Sym()->Lnext() == b2 );
        assert( b2->Org()->anEdge->Org() == b2->Org() );
        
        assert(aOrg->anEdge->Org() == aOrg);
        assert(bOrg->anEdge->Org() == bOrg);
        
        assert(a0->Oprev()->Onext()->Org() == a0->Org());
    }
    
    /* tessMeshDeleteMesh( mesh ) will free all storage for any valid mesh.
     */
    void tessMeshDeleteMesh( TESSmesh *mesh )
    {
        delete mesh;
    }
    
#ifndef NDEBUG
    
    /* tessMeshCheckMesh( mesh ) checks a mesh for self-consistency.
     */
    void tessMeshCheckMesh( TESSmesh *mesh )
    {
        TESSface *fHead = mesh->fEnd();
        TESSvertex *vHead = mesh->vEnd();
        TESShalfEdge *eHead = mesh->eEnd();
        TESSface *f, *fPrev;
        TESSvertex *v, *vPrev;
        TESShalfEdge *e, *ePrev;
        
        for( fPrev = fHead ; (f = fPrev->next) != fHead; fPrev = f) {
            assert( f->prev == fPrev );
            e = f->anEdge;
            do {
                assert( e->Sym() != e );
                assert( e->Sym()->Sym() == e );
                assert( e->Lnext()->Onext()->Sym() == e );
                assert( e->Onext()->Sym()->Lnext() == e );
                assert( e->Lface() == f );
                e = e->Lnext();
            } while( e != f->anEdge );
        }
        assert( f->prev == fPrev && f->anEdge == nullptr );
        
        for( vPrev = vHead ; (v = vPrev->next) != vHead; vPrev = v) {
            assert( v->prev == vPrev );
            e = v->anEdge;
            do {
                assert( e->Sym() != e );
                assert( e->Sym()->Sym() == e );
                assert( e->Lnext()->Onext()->Sym() == e );
                assert( e->Onext()->Sym()->Lnext() == e );
                assert( e->Org() == v );
                e = e->Onext();
            } while( e != v->anEdge );
        }
        assert( v->prev == vPrev && v->anEdge == nullptr );
        
        for( ePrev = eHead ; (e = ePrev->next()) != eHead; ePrev = e) {
            assert( e->Sym()->next() == ePrev->Sym() );
            assert( e->Sym() != e );
            assert( e->Sym()->Sym() == e );
            assert( e->Org() != nullptr );
            assert( e->Dst() != nullptr );
            assert( e->Lnext()->Onext()->Sym() == e );
            assert( e->Onext()->Sym()->Lnext() == e );
        }
        assert( e->Sym()->next() == ePrev->Sym()
               && e->Sym() == mesh->eSymEnd()
               && e->Sym()->Sym() == e
               && e->Org() == nullptr && e->Dst() == nullptr
               && e->Lface() == nullptr && e->Rface() == nullptr );
    }
}

#endif
