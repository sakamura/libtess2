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

namespace Tess
{
	/* The basic operations are insertion of a new key (pqInsert),
	 * and examination/extraction of a key whose value is minimum
	 * (pqMinimum/pqExtractMin).  Deletion is also allowed (pqDelete);
	 * for this purpose pqInsert returns a "handle" which is supplied
	 * as the argument.
	 *
	 * An initial heap may be created efficiently by calling pqInsert
	 * repeatedly, then calling pqInit.	 In any case pqInit must be called
	 * before any operations other than pqInsert are used.
	 *
	 * If the heap is empty, pqMinimum/pqExtractMin will return a nullptr key.
	 * This may also be tested with pqIsEmpty.
	 */
	
	/* Since we support deletion the data structure is a little more
	 * complicated than an ordinary heap.  "nodes" is the heap itself;
	 * active nodes are stored in the range 1..pq->size.  When the
	 * heap exceeds its allocated size (pq->max), its size doubles.
	 * The children of node i are nodes 2i and 2i+1.
	 *
	 * Each node stores an index into an array "handles".  Each handle
	 * stores a key, plus a pointer back to the node which currently
	 * represents that key (ie. nodes[handles[i].node].handle == i).
	 */
	
	template <typename Options, typename Allocators>
	struct PriorityQT {
        using PriorityQ = PriorityQT<Options, Allocators>;
        using Tesselator = Tess::Tesselator<Options, Allocators>;
        using Vertex = VertexT<Options, Allocators>;
        
        enum { INV_HANDLE = 0x0fffffff };
        
        typedef Vertex *Key;
        typedef int Handle;
        struct Node { Handle handle; };
        struct HandleElem { Key key; Handle node; };
        using LeqFunc = bool (*)(const Key key1, const Key key2);

        struct Heap {
            Tesselator* t;
            
            Node *nodes;
            HandleElem *handles;
            int size, max;
            Handle freeList;
            int initialized;
            
            LeqFunc leq;

            Heap( Tesselator* t, int size, LeqFunc leq );
            ~Heap( );

            void init( );
            Handle insert( Key keyNew );
            Key extractMin( );
            void remove( Handle hCurr );
            
            Key minimum() { return handles[nodes[1].handle].key; }
            bool isEmpty() { return size == 0; }
        };

        Heap heap;
		
		Key *keys;
		Key **order;
		Handle size, max;
		int initialized;
		
		LeqFunc leq;
        
        PriorityQT( Tesselator* t, int size, LeqFunc leq );
        ~PriorityQT( );
        
        void init( );
        Handle insert( Key key );
        Key extractMin( );
        void remove( Handle handle );
        
        Key minimum( );
        int isEmpty( );
    };
}

#include "priorityq.inl"
