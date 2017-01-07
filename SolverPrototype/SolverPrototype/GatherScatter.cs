using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    class GatherScatter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void GatherVelocities(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //The user guarantees that the entity velocities are pointer safe.
            //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
            //we do a really gross hack where we manually stuff the memory backing of a bunch of vectors.
            //This logic is coupled with the layout of the EntityVelocities struct and makes assumptions about the memory layout of the type.
            //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
            //With any luck, it will be later enough that a proper solution exists.
            var csvLinearAX = (float*)Unsafe.AsPointer(ref velocitiesA);
            var csvLinearAY = csvLinearAX + Vector<float>.Count;
            var csvLinearAZ = csvLinearAX + 2 * Vector<float>.Count;
            var csvAngularAX = csvLinearAX + 3 * Vector<float>.Count;
            var csvAngularAY = csvLinearAX + 4 * Vector<float>.Count;
            var csvAngularAZ = csvLinearAX + 5 * Vector<float>.Count;
            var csvLinearBX = (float*)Unsafe.AsPointer(ref velocitiesB);
            var csvLinearBY = csvLinearBX + Vector<float>.Count;
            var csvLinearBZ = csvLinearBX + 2 * Vector<float>.Count;
            var csvAngularBX = csvLinearBX + 3 * Vector<float>.Count;
            var csvAngularBY = csvLinearBX + 4 * Vector<float>.Count;
            var csvAngularBZ = csvLinearBX + 5 * Vector<float>.Count;

            //TODO: Should check if a simpler approach that just inline-grabs the specific pointer is faster... I doubt we can avoid caching it, but we might be able to avoid pointer arithmetic.
            //Would be nice to have an indexable struct that is exactly the same size as vector<float>, but creating such a thing is tricky since the size is not known at compile time.
            //TODO: avoiding pointer arithmetic entirely so the user didn't have to guarantee GC safety would be nice- we could use Unsafe.Add if the codegen is good enough.

            for (int i = 0; i < references.Count; ++i)
            {
                //Null connections are inaccessible for reads.
                //Their velocities are always zero.
                //Since locals get initialized to zero, we don't need to write it explicitly.
                var bundleIndexA = references.BundleIndexB[i];
                if (bundleIndexA >= -1)
                {
                    ref var bundleA = ref velocities[bundleIndexA];
                    var innerIndexA = references.InnerIndexA[i];
                    csvLinearAX[i] = bundleA.LinearVelocity.X[innerIndexA];
                    csvLinearAY[i] = bundleA.LinearVelocity.Y[innerIndexA];
                    csvLinearAZ[i] = bundleA.LinearVelocity.Z[innerIndexA];
                    csvAngularAX[i] = bundleA.AngularVelocity.X[innerIndexA];
                    csvAngularAY[i] = bundleA.AngularVelocity.Y[innerIndexA];
                    csvAngularAZ[i] = bundleA.AngularVelocity.Z[innerIndexA];
                }

                ref var bundleB = ref velocities[references.BundleIndexB[i]];
                var innerIndexB = references.InnerIndexB[i];
                csvLinearBX[i] = bundleB.LinearVelocity.X[innerIndexB];
                csvLinearBY[i] = bundleB.LinearVelocity.Y[innerIndexB];
                csvLinearBZ[i] = bundleB.LinearVelocity.Z[innerIndexB];
                csvAngularBX[i] = bundleB.AngularVelocity.X[innerIndexB];
                csvAngularBY[i] = bundleB.AngularVelocity.Y[innerIndexB];
                csvAngularBZ[i] = bundleB.AngularVelocity.Z[innerIndexB];
            }

            //You may notice that this is a pretty dang heavy weight preamble to every solve iteration. Two points:
            //1) You should make sure to make this preamble as common as possible to all constraints, so that every time a new optimization becomes possible, you needn't
            //revisit every single constraint. This means the initial jacobian multiply can't be bundled with it. Good thing there's no reason for it to be.
            //2) When there's a possibility to increase the quality of a constraint solve by doing a little more effort within the solve iteration, chances are it's
            //worth it. For cheap low quality solves on low-DOF constraints, the pregather and postscatter may take significantly longer than the actual math!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void ScatterVelocities(BodyVelocities[] velocities, ref BodyReferences references, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            //Scatter is actually harder than gathering. For every single constraint, we have to find the associated entities independently.
            //We can't precache the source pointers or anything.
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                //If there is no constraint here, we don't care about the contents of this or any of the remaining slots in this bundle.
                var bundleIndexA = references.BundleIndexA[i];
                if (bundleIndexA < 0)
                    break;
                //We'll use the memory layout of the EntityVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                var linearAX = (float*)Unsafe.AsPointer(ref velocities[bundleIndexA].LinearVelocity.X) + references.InnerIndexA[i];
                *linearAX = velocitiesA.LinearVelocity.X[i];
                *(linearAX + Vector<float>.Count) = velocitiesA.LinearVelocity.Y[i];
                *(linearAX + 2 * Vector<float>.Count) = velocitiesA.LinearVelocity.Z[i];
                *(linearAX + 3 * Vector<float>.Count) = velocitiesA.AngularVelocity.X[i];
                *(linearAX + 4 * Vector<float>.Count) = velocitiesA.AngularVelocity.Y[i];
                *(linearAX + 5 * Vector<float>.Count) = velocitiesA.AngularVelocity.Z[i];

                var linearBX = (float*)Unsafe.AsPointer(ref velocities[references.BundleIndexB[i]].LinearVelocity.X) + references.InnerIndexB[i];
                *linearBX = velocitiesB.LinearVelocity.X[i];
                *(linearBX + Vector<float>.Count) = velocitiesB.LinearVelocity.Y[i];
                *(linearBX + 2 * Vector<float>.Count) = velocitiesB.LinearVelocity.Z[i];
                *(linearBX + 3 * Vector<float>.Count) = velocitiesB.AngularVelocity.X[i];
                *(linearBX + 4 * Vector<float>.Count) = velocitiesB.AngularVelocity.Y[i];
                *(linearBX + 5 * Vector<float>.Count) = velocitiesB.AngularVelocity.Z[i];
            }
        }
    }
}
