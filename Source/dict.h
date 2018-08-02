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

namespace Tess
{
	typedef void *DictKey;
	typedef struct Dict Dict;
	typedef struct DictNode DictNode;
	typedef bool (*LeqFunc)(void *frame, DictKey key1, DictKey key2);
	
	/*** Private data structures ***/
	
	struct DictNode {
		DictKey	key;
		DictNode *next;
		DictNode *prev;
	};
	
    template <typename Options, typename Allocators>
	struct DictT {
        using Tesselator = Tess::Tesselator<Options, Allocators>;
        using Dict = DictT<Options, Allocators>;
        
        Tesselator* t;
		DictNode _head;
		LeqFunc _leq;

		DictT(Tesselator* t, LeqFunc leq);
		~DictT();

		/* Search returns the node with the smallest key greater than or equal
		 * to the given key.  If there is no such key, returns a node whose
		 * key is nullptr.	Similarly, Succ(Max(d)) has a nullptr key, etc.
		 */
		DictNode *search( DictKey key );
		DictNode *insertBefore( DictNode *node, DictKey key );
		void deleteNode( DictNode *node );
		DictNode *insert( DictKey key ) { return insertBefore(&_head, key); }
		DictNode *min() { return _head.next; }
		DictNode *max() { return _head.prev; }
	};
}

#include "dict.inl"
