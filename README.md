libtess3 C++
============
**Version 0.1.0**

**libtess3** is a good quality polygon tesselator and triangulator.

This is a refactoring of the **libtess2** C tesselator. The goal for this version is to have a C++-compliant and templated version of libtess2, where according to your external requirements, you get the properly optimized internal code. For example, choosing to use integer instead of float for inner and outer tesselation, different algorithms than straight lines for boundaries, a precise pattern for vectors alongside extra values, all of this while reducing or removing any copying of values internally.

License
-------
The code is released under SGI FREE SOFTWARE LICENSE B Version 2.0.
https://directory.fsf.org/wiki/License:SGIFreeBv2
This is the same license than the original libtess and the awesome libtess2 refactor.

Maintainers
-----------
> Original libtess in C: Eric Veach (SGI), as maintained by Mesa
> https://gitlab.freedesktop.org/mesa/glu/tree/master/src/libtess
> A lot of the GLU tesselator documentation applies to Libtess2 too (apart from the API), check out
> http://www.glprogramming.com/red/chapter11.html

> libtess2 in C: Mikko Mononen
> memon@inside.org
> https://github.com/memononen/libtess2

> libtess3 in C++: Michel Donais (this version)
> https://github.com/sakamura/libtess3
> Please look at the github project's contributors for individual contributors to this branch

Fair Warning
------------
This code is undergoing a LOT of changes, breaking and incompatible, potentially deal breaking. Use this for bleeding edge, don't use this if you aren't ready to have your hands dirty and losing precious hours to make it work! The orignal libtess2 is much more stable! I am but a poor soul trying to do a very hard task in his off-time. I feel like I have done most of the grunt work, making the once daunting task seem possible, and having helping hands be able to push new features, but you'll understand there is a major step to have this running in a production environment. If you feel like helping achieve the holy grail, please don't hesitate to propose patches and send bugs and feature requests.

To do
-----
- All parametrizable (through Options) STL containers for dictionaries, edge stack, priority queues (all while making sure we aren't losing performance)
- The evil EdgePair has to be fixed. There must be a better way to do this!
- More external geometry whenever possible. (Random probably unsuitable examples: having templated hooks to GLM and high speed libraries such as Eigen)
- Allowing multiple tesselation algorithms
- Compatibility with something else than iOS: make platforms ubiquitous
- Remove part of the requirement to output the results, doing it dynamically while tesselation occurs instead
- Add back 3D
- Add new examples
- Add unit tests

To do that will never be satisfied
----------------------------------
- Bug fixing, edge cases, optimizations
- Dusting off current code, which has a lot of inconsistencies and evil parts
- Rewriting code to support C++17 by default and new C++20 features when there's a speed gain and legibility to be had. Examples are new for loops; accessor functions; removing remaining defines; reducing short variable names and A()->B().C().d.E() paradigms with (or without) parenthesis that clutter the code uselessly; making code more legible; removing remaining static functions; usage of _ptrs and algorithms; coroutines; ...
