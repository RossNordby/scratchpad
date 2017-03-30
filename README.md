# scratchpad

The three main objects of interest in this repo right now are:

1) The [prototype solver v2 implementation](SolverPrototype). Only one constraint type exists so far, but a great deal of the infrastructure is functional.

2) The [prototype tree implementation](SIMDPrototyping/SIMDPrototyping/Trees/SingleArray). The SIMDPrototyping application is currently configured to compare the performance of the old BEPUphysics broad phase with this tree. For more information, see the blog post: http://www.bepuphysics.com/blog/2015/9/19/blazing-fast-trees.html

3) [BEPUutilities v2](BEPUutilities%20v2). It's slowly being adapted for use with System.Numerics.Vectors and better resource management. Expect it to eventually house a better-packaged variant of the above tree (unless it ends up going in its own distinct project for some reason).

No documentation for this repo. Useful stuff will eventually get promoted out into cleaner repos.
