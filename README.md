A 3D physics engine that uses Separating Axis Test for collision detection, the clipping method for generating contact manifold, contact point reduction as outlined in [1] and a sequential impulses constraint solver as popularized by Erin Catto.

![](https://media1.giphy.com/media/SwP1QA8DZhvtij6VPy/giphy.gif)

There's still a lot of issues and improvements to be made for this. The biggest ones being
 * Stable stacking
 * Improved broadphase with dynamic AABB tree
 * Verlet integration instead of Euler
 * Implement quickhull to generate more interesting hulls than just boxes
 * Optimize SAT with edge pruning by using gauss maps
 * Linux support (currently only builds on Windows)
 * The code is a bit of a mess and could use some refactoring

## References
 1. Dirk Gregorius. 2015. "Robust Contact Creation for Physics Simulations" GDC
 2. Erin Catto. 2006. "Fast and Simple Physics using Sequential Impulses" GDC
 3. Gin van den Bergen and Dirk Gregorius. 2010. "Game Physics Pearls"
 4. Christer Ericson. 2005. "Real-Time Collision Detection"
 5. Dirk Gregorius. 2013. "The Separating Axis Test Between Convex Polyhedra" GDC
 6. Dirk Gregorius. 2014. "Implementing QuickHull" GDC
 7. Erin Catto. 2019. "Dynamic Bounding Volume Hierarchies" GDC
 8. Erin Catto. 2014. "Understanding Constraints" GDC

