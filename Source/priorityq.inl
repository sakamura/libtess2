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
#include "tess.h"
#include "priorityq.h"


#define INIT_SIZE	32

/* Violates modularity, but a little faster */
#include "geom.h"
#define LEQ(x,y)	vertAreLessOrEqual(x, y)

/* Include all the code for the regular heap-based queue here. */

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
	
	
#define pqHeapMinimum(pq)	((pq)->handles[(pq)->nodes[1].handle].key)
#define pqHeapIsEmpty(pq)	((pq)->size == 0)
	
	
	
	/* really pqHeapNewPriorityQHeap */
    template <typename Options, typename Allocators>
    PriorityQT<Options, Allocators>::Heap::Heap( Tesselator* _t, int _size, LeqFunc _leq ) :
        t(_t)
	{
		size = 0;
		max = _size;
		assert(_size > 0);
		nodes = new Node[(unsigned long)size+1];
		handles = new HandleElem[(unsigned long)size+1];
		
		initialized = false;
		freeList = 0;
		leq = _leq;
		
		nodes[1].handle = 1;	/* so that Minimum() returns nullptr */
		handles[1].key = nullptr;
	}
	
	/* really pqHeapDeletePriorityQHeap */
    template <typename Options, typename Allocators>
    PriorityQT<Options, Allocators>::Heap::~Heap( )
	{
		delete[] handles;
		delete[] nodes;
	}
	
	
    template <typename Options, typename Allocators>
	static void FloatDown( typename PriorityQT<Options, Allocators>::Heap *pq, int curr )
	{
        using Node = typename PriorityQT<Options, Allocators>::Node;
        using HandleElem = typename PriorityQT<Options, Allocators>::HandleElem;
        using Handle = typename PriorityQT<Options, Allocators>::Handle;
        
		Node *n = pq->nodes;
		HandleElem *h = pq->handles;
		Handle hCurr, hChild;
		int child;
		
		hCurr = n[curr].handle;
		for( ;; ) {
			child = curr << 1;
			if( child < pq->size && LEQ( h[n[child+1].handle].key,
										h[n[child].handle].key )) {
				++child;
			}
			
			assert(child <= pq->max);
			
			hChild = n[child].handle;
			if( child > pq->size || LEQ( h[hCurr].key, h[hChild].key )) {
				n[curr].handle = hCurr;
				h[hCurr].node = curr;
				break;
			}
			n[curr].handle = hChild;
			h[hChild].node = curr;
			curr = child;
		}
	}
	
	
    template <typename Options, typename Allocators>
	static void FloatUp( typename PriorityQT<Options, Allocators>::Heap *pq, int curr )
	{
        using Node = typename PriorityQT<Options, Allocators>::Node;
        using HandleElem = typename PriorityQT<Options, Allocators>::HandleElem;
        using Handle = typename PriorityQT<Options, Allocators>::Handle;
        
		Node *n = pq->nodes;
		HandleElem *h = pq->handles;
		Handle hCurr, hParent;
		int parent;
		
		hCurr = n[curr].handle;
		for( ;; ) {
			parent = curr >> 1;
			hParent = n[parent].handle;
			if( parent == 0 || LEQ( h[hParent].key, h[hCurr].key )) {
				n[curr].handle = hCurr;
				h[hCurr].node = curr;
				break;
			}
			n[curr].handle = hParent;
			h[hParent].node = curr;
			curr = parent;
		}
	}
	
	/* really pqHeapInit */
    template <typename Options, typename Allocators>
    void PriorityQT<Options, Allocators>::Heap::init( )
	{
		int i;
		
		/* This method of building a heap is O(n), rather than O(n lg n). */
		
		for( i = size; i >= 1; --i ) {
			FloatDown( this, i );
		}
		initialized = true;
	}
	
	/* really pqHeapInsert */
    template <typename Options, typename Allocators>
    typename PriorityQT<Options, Allocators>::Handle PriorityQT<Options, Allocators>::Heap::insert( Key keyNew )
	{
		int curr;
		Handle free;
		
		curr = ++ size;
		assert((curr*2) <= max );
		
		if( freeList == 0 ) {
			free = curr;
		} else {
			free = freeList;
			freeList = handles[free].node;
		}
		
		nodes[curr].handle = free;
		handles[free].node = curr;
		handles[free].key = keyNew;
		
		if( initialized ) {
			FloatUp( this, curr );
		}
		assert(free != INV_HANDLE);
		return free;
	}
	
	/* really pqHeapExtractMin */
    template <typename Options, typename Allocators>
    typename PriorityQT<Options, Allocators>::Key PriorityQT<Options, Allocators>::Heap::extractMin( )
	{
		Node *n = nodes;
		HandleElem *h = handles;
		Handle hMin = n[1].handle;
		Key min = h[hMin].key;
		
		if( size > 0 ) {
			n[1].handle = n[size].handle;
			h[n[1].handle].node = 1;
			
			h[hMin].key = nullptr;
			h[hMin].node = freeList;
			freeList = hMin;
			
			if( -- size > 0 ) {
				FloatDown( this, 1 );
			}
		}
		return min;
	}
	
	/* really pqHeapDelete */
    template <typename Options, typename Allocators>
    void PriorityQT<Options, Allocators>::Heap::remove( Handle hCurr )
	{
		Node *n = nodes;
		HandleElem *h = handles;
		int curr;
		
		assert( hCurr >= 1 && hCurr <= max && h[hCurr].key != nullptr );
		
		curr = h[hCurr].node;
		n[curr].handle = n[size].handle;
		h[n[curr].handle].node = curr;
		
		if( curr <= -- size ) {
			if( curr <= 1 || LEQ( h[n[curr>>1].handle].key, h[n[curr].handle].key )) {
				FloatDown( this, curr );
			} else {
				FloatUp( this, curr );
			}
		}
		h[hCurr].key = nullptr;
		h[hCurr].node = freeList;
		freeList = hCurr;
	}
	
	
	
	/* Now redefine all the function names to map to their "Sort" versions. */
	
	/* really tessPqSortNewPriorityQ */
    template <typename Options, typename Allocators>
    PriorityQT<Options, Allocators>::PriorityQT( Tesselator* _t, int _size, LeqFunc _leq ) :
        heap(_t, size, leq)
	{
		keys = new Key[(unsigned long)size];
		
		size = 0;
		max = _size; //INIT_SIZE;
		initialized = false;
		leq = _leq;
	}
	
	/* really tessPqSortDeletePriorityQ */
    template <typename Options, typename Allocators>
    PriorityQT<Options, Allocators>::~PriorityQT( )
	{
		if (order != nullptr) delete[] order;
		if (keys != nullptr) delete[] keys;
	}
	
	
#define LT(x,y)		(! LEQ(y,x))
#define GT(x,y)		(! LEQ(x,y))
#define Swap(a,b)	if(1){Key *tmp = *a; *a = *b; *b = tmp;}else
	
	/* really tessPqSortInit */
    template <typename Options, typename Allocators>
	void PriorityQT<Options, Allocators>::init( )
	{
		Key **p, **r, **i, **j, *piv;
		struct { Key **p, **r; } Stack[50], *top = Stack;
		unsigned int seed = 2016473283;
		
		/* Create an array of indirect pointers to the keys, so that we
		 * the handles we have returned are still valid.
		 */
		order = new Key*[(unsigned long)size+1];
		
		p = order;
		r = p + size - 1;
		for( piv = keys, i = p; i <= r; ++piv, ++i ) {
			*i = piv;
		}
		
		/* Sort the indirect pointers in descending order,
		 * using randomized Quicksort
		 */
		top->p = p; top->r = r; ++top;
		while( --top >= Stack ) {
			p = top->p;
			r = top->r;
			while( r > p + 10 ) {
				seed = seed * 1539415821 + 1;
				i = p + seed % (r - p + 1);
				piv = *i;
				*i = *p;
				*p = piv;
				i = p - 1;
				j = r + 1;
				do {
					do { ++i; } while( GT( **i, *piv ));
					do { --j; } while( LT( **j, *piv ));
					Swap( i, j );
				} while( i < j );
				Swap( i, j ); /* Undo last swap */
				if( i - p < r - j ) {
					top->p = j+1; top->r = r; ++top;
					r = i-1;
				} else {
					top->p = p; top->r = i-1; ++top;
					p = j+1;
				}
			}
			/* Insertion sort small lists */
			for( i = p+1; i <= r; ++i ) {
				piv = *i;
				for( j = i; j > p && LT( **(j-1), *piv ); --j ) {
					*j = *(j-1);
				}
				*j = piv;
			}
		}
		max = size;
		initialized = true;
		heap.init( );	 /* always succeeds */
		
#ifndef NDEBUG
		p = order;
		r = p + size - 1;
		for( i = p; i < r; ++i ) {
			assert( LEQ( **(i+1), **i ));
		}
#endif
	}
	
	/* really tessPqSortInsert */
	/* returns INV_HANDLE iff out of memory */ 
    template <typename Options, typename Allocators>
	typename PriorityQT<Options, Allocators>::Handle PriorityQT<Options, Allocators>::insert( Key keyNew )
	{
		int curr;
		
		if( initialized ) {
			return heap.insert( keyNew );
		}
		curr = size;
		++size;
		assert(size < max);
		keys[curr] = keyNew;
		
		/* Negative handles index the sorted array. */
		return -(curr+1);
	}
	
	/* really tessPqSortExtractMin */
    template <typename Options, typename Allocators>
	typename PriorityQT<Options, Allocators>::Key PriorityQT<Options, Allocators>::extractMin( )
	{
		Key sortMin, heapMin;
		
		if( size == 0 ) {
			return heap.extractMin( );
		}
		sortMin = *(order[size-1]);
		if( ! heap.isEmpty( )) {
			heapMin = heap.minimum( );
			if( LEQ( heapMin, sortMin )) {
				return heap.extractMin( );
			}
		}
		do {
			-- size;
		} while( size > 0 && *(order[size-1]) == nullptr );
		return sortMin;
	}
	
	/* really tessPqSortMinimum */
    template <typename Options, typename Allocators>
	typename PriorityQT<Options, Allocators>::Key PriorityQT<Options, Allocators>::minimum( )
	{
		Key sortMin, heapMin;
		
		if( size == 0 ) {
			return heap.minimum( );
		}
		sortMin = *(order[size-1]);
		if( ! heap.isEmpty( )) {
			heapMin = heap.minimum( );
			if( LEQ( heapMin, sortMin )) {
				return heapMin;
			}
		}
		return sortMin;
	}
	
	/* really tessPqSortIsEmpty */
    template <typename Options, typename Allocators>
	int PriorityQT<Options, Allocators>::isEmpty( )
	{
		return (size == 0) && heap.isEmpty( );
	}
	
	/* really tessPqSortDelete */
    template <typename Options, typename Allocators>
	void PriorityQT<Options, Allocators>::remove( Handle curr )
	{
		if( curr >= 0 ) {
			heap.remove( curr );
			return;
		}
		curr = -(curr+1);
		assert( curr < max && keys[curr] != nullptr );
		
		keys[curr] = nullptr;
		while( size > 0 && *(order[size-1]) == nullptr ) {
			-- size;
		}
	}
}
