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
 ** Author: Michel Donais, July 2018.
 */

#pragma once

namespace Tess
{
    template <typename Options, typename Allocators>
    struct EdgeStackT
    {
        using HalfEdge = HalfEdgeT<Options, Allocators>;
        
        struct Node
        {
            HalfEdge *edge;
            Node *next;
        };
        
        Tesselator<Options, Allocators>* t;
        Node *top;
        
        EdgeStackT(Tesselator<Options, Allocators>* _t) :
            t(_t),
            top(nullptr)
        {
        }
        
        int empty()
        {
            return top == nullptr;
        }
        
        void push( HalfEdge *edge )
        {
            Node *node = t->allocators.edgeStackNodeAlloc.construct();
            if ( ! node ) return;
            node->edge = edge;
            node->next = top;
            top = node;
        }
        
        HalfEdge *pop( )
        {
            HalfEdge *edge = nullptr;
            Node *node = top;
            if (node) {
                top = node->next;
                edge = node->edge;
                t->allocators.edgeStackNodeAlloc.destroy(node);
            }
            return edge;
        }
        
        ~EdgeStackT( )
        {
            do {} while (pop() != nullptr);
        }
    };
}
