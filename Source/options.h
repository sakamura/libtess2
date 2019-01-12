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
** Original Author: Michel Donais, July 2018.
 */

#pragma once

#include <utility>

namespace Tess
{
    template <typename Options, typename Allocators>
    struct VertexT;
    
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
    
    // These are the default options for the tesselation code. As this is used in a template,
    // you can use this as a base class for your own options. If they are constexpr, they will
    // be optimized in compile-time. If they are not, the option will be checked every time it's
    // being used.
    struct BaseOptions
    {
        using Coord = float;            // One vector coordinate. For internal format.
        struct Vec                      // One vector, as seen in the outside
        {
            Coord x;
            Coord y;
        };
        struct SweepPlaneVec            // One vector, as used internally. Member variables are opaque.
        {
            Coord s;
            Coord t;
        };
        static inline Coord getS(const SweepPlaneVec& sVec)
        {
            return sVec.s;
        }
        static inline Coord getT(const SweepPlaneVec& sVec)
        {
            return sVec.t;
        }
        struct InternalVec              // One vector, as used internally, in read-write. Mmebers are not opaque.
        {
            using value_type = Coord;
            value_type s;
            value_type t;
            
            inline value_type getS() const
            {
                return s;
            }
            inline value_type getT() const
            {
                return t;
            }
        };
        using Id = struct {};
        using AddResult = std::pair<SweepPlaneVec, Id>;

        // Adding a point whose origin is a Vector (meaning it's being added from the addVertex outside world)
        inline AddResult addPoint(const Vec& vec)
        {
            return {{ vec.x, vec.y }, {}};
        }
        
        // Adding a point whose origin is a Sweep Plane Vector (meaning it's being added from the Sweep algorithm)
        inline AddResult addPoint(Coord s, Coord t)
        {
            return {{ s, t }, {}};
        }
        inline SweepPlaneVec addPoint(const InternalVec& iVec)
        {
            return { iVec.s, iVec.t };
        }
        
        // Adding a point that will be probably ignored by the end result
        inline SweepPlaneVec addSentinelPoint(Coord s, Coord t)
        {
            return { s, t };
        }
        
        // Adding a contour point. Used when outputting a contour. Returns the start two times (at beginning and at end)
        template <typename Vertex>
        inline void addContour(Id idx, const Vertex*)
        {
            addContourIdx(idx);
        }
        inline void addContourIdx(Id) {}
        
        // Adding a polygon point. Used when outputting polygons. Will fill polySize with empty idx
        template <typename Vertex>
        inline void addVertex(Id idx, const Vertex*)
        {
            addVertexIdx(idx);
        }
        inline void addVertexIdx(Id) {}
        inline void addEmptyVertex() {}
        
        constexpr bool constrainedDelaunayTriangulation() const { return false; }
        constexpr bool reverseContours() const { return false; }
        constexpr WindingRule windingRule() const { return TESS_WINDING_ODD; }

        // Retrieves an allocator memory pool that is shared by all the tesselators. Doesn't have any by default.
        template <typename _AllocatorPool>
        constexpr _AllocatorPool* getAllocatorPool() { return nullptr; }
    };
    
    // This is what you usually wish to have dynamically set up.
    struct DefaultOptions : public BaseOptions
    {
        WindingRule m_windingRule = BaseOptions::windingRule();
        
        WindingRule windingRule() const { return m_windingRule; }
    };
    
    // This is an example on how to use an internal Glm type
    template <typename GlmVecType>
    struct GlmOptions : public DefaultOptions
    {
        using Coord = typename GlmVecType::value_type;
        using Vec = GlmVecType;
    };
    
    // This is an example of a fully dynamic option code. The default libTess2 is mostly working
    // like this, but there's little reason to have it all dynamic. Usually, you might wish to retain
    // control of one or two options for yourself, but seldom will you do ask for all of them.
    struct DynamicOptions : public DefaultOptions
    {
        bool m_constrainedDelaunayTriangulation = BaseOptions::constrainedDelaunayTriangulation();
        bool m_reverseContours = BaseOptions::reverseContours();
        
        bool constrainedDelaunayTriangulation() const { return m_constrainedDelaunayTriangulation; }
        bool reverseContours() const { return m_reverseContours; }
    };
}
