using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace SolverPrototype
{

    /// <summary>
    /// Integrates the velocity of mobile bodies over time into changes in position and orientation. Also applies gravitational acceleration to dynamic bodies.
    /// </summary>
    /// <remarks>
    /// This variant of the integrator uses a single global gravity. Other integrators that provide per-entity gravity could exist later.
    /// This integrator also assumes that the bodies positions are stored in terms of single precision floats. Later on, we will likely modify the Bodies
    /// storage to allow different representations for larger simulations. That will require changes in this integrator, the relative position calculation of collision detection,
    /// the bounding box calculation, and potentially even in the broadphase in extreme cases (64 bit per component positions).
    /// </remarks>
    public class PoseIntegrator
    {
        Bodies bodies;
        public PoseIntegrator(Bodies bodies)
        {
            this.bodies = bodies;
        }

        /// <summary>
        /// Acceleration of gravity to apply to all dynamic bodies in the simulation.
        /// </summary>
        public Vector3 Gravity;

        void IntegrateBundles(int startBundle, int exclusiveEndBundle, float dt)
        {
            ref var basePoses = ref bodies.Poses[0];
            ref var baseVelocities = ref bodies.Velocities[0];
            ref var baseLocalInertias = ref bodies.LocalInertias[0];
            ref var baseInertias = ref bodies.Inertias[0];
            var vectorDt = new Vector<float>(dt);
            var vectorHalfDt = new Vector<float>(dt * 0.5f);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                //Integrate position with the latest linear velocity. Note that gravity is integrated afterwards.
                ref var pose = ref Unsafe.Add(ref basePoses, i);
                ref var velocity = ref Unsafe.Add(ref baseVelocities, i);
                Vector3Wide.Scale(ref velocity.LinearVelocity, ref vectorDt, out var displacement);
                Vector3Wide.Add(ref pose.Position, ref displacement, out pose.Position);

                //Integrate orientation with the latest angular velocity.
                //Note that we don't bother with conservation of angular momentum or the gyroscopic term or anything else- 
                //it's not exactly correct, but it's stable, fast, and no one really notices. Unless they're trying to spin a multitool in space or something.
                //(But frankly, that just looks like reality has a bug.)
                QuaternionWide multiplier;
                multiplier.X = vectorHalfDt * velocity.AngularVelocity.X;
                multiplier.Y = vectorHalfDt * velocity.AngularVelocity.Y;
                multiplier.Z = vectorHalfDt * velocity.AngularVelocity.Z;
                multiplier.W = Vector<float>.Zero;
                QuaternionWide.ConcatenateWithoutOverlap(ref pose.Orientation, ref multiplier, out var increment);
                QuaternionWide.Add(ref pose.Orientation, ref increment, out pose.Orientation);
                QuaternionWide.Normalize(ref pose.Orientation, out pose.Orientation);

                //Update the inertia tensors for the new orientation.
                //TODO: If the pose integrator is positioned at the end of an update, the first frame after any out-of-timestep orientation change or local inertia change
                //has to get is inertia tensors calculated elsewhere. Either they would need to be computed on addition or something- which is a bit gross, but doable-
                //or we would need to move this calculation to the beginning of the frame to guarantee that all inertias are up to date. 
                //This would require a scan through all pose memory to support, but if you do it at the same time as AABB update, that's fine- that stage uses the pose too.
                ref var localInertias = ref Unsafe.Add(ref baseLocalInertias, i);
                ref var inertias = ref Unsafe.Add(ref baseInertias, i);
                Matrix3x3Wide.CreateFromQuaternion(ref pose.Orientation, out var orientationMatrix);
                //I^-1 = RT * Ilocal^-1 * R 
                //NOTE: If you were willing to confuse users a little bit, the local inertia could be required to be diagonal.
                //This would be totally fine for all the primitive types which happen to have diagonal inertias, but for more complex shapes (convex hulls, meshes), 
                //there would need to be a reorientation step. That could be confusing, and I'm not sure if it's worth it.
                Triangular3x3Wide.RotationSandwich(ref orientationMatrix, ref localInertias.InverseInertiaTensor, out inertias.InverseInertiaTensor);
                //While it's a bit goofy just to copy over the inverse mass every frame even if it doesn't change,
                //it's virtually always gathered together with the inertia tensor and it really isn't worth a whole extra external system to copy inverse masses only on demand.
                inertias.InverseMass = localInertias.InverseMass;

                //Note that we apply gravity during this phase. That means, if the integrator is put at the end of the frame, the velocity will be nonzero.
                //For now, we're assuming that the integrator runs at the beginning of the frame. This is a tradeoff, with some pros:
                //1) Contacts generated in a given frame will match the position. You could draw effects at the contact points without worrying about them being one frame offset.
                //However, if we detect contacts from a predicted transform, this mostly goes out the window.
                //2) There is no need for a second 'force application' stage before the AABB update. Realistically, the force application would be bundled into the AABB update.
                //3) Advanced users are free to trivially intervene in the velocity modifications caused by constraints before they are integrated. (A callback would be just as good.)
                //And one big con:
                //If you modify velocity outside of the update, it will be directly used to integrate position during the next frame, ignoring all constraints.
                //This WILL be a problem, especially since BEPUphysics v1 trained people to use velocity modifications to control motion.

                //This isn't an unsolvable problem- making it easy to handle velocity modifications mid-update by exposing a callback of some sort would work. But that's one step more
                //than the v1 'just set the velocity' style.

                //TODO:
                //I suspect that splitting the gravity application from this integrator will be the right choice in the end. Bundling it with AABB calculation would free
                //up the integrator to be placed on either end of execution without worrying about odd velocities being visible in resting objects, and there is basically no downside. 
                //(Note that the AABB calculation must sample both pose and velocity, so there's no additional bandwidth used.)
                //You could then provide different update orders. People could use the position-first or position-last variant as desired.
                //Whatever handles gravity will also probably end up handling drag.

                //Note that we avoid accelerating kinematics. Kinematics are any body with an inverse mass of zero (so a mass of ~infinity). No force can move them.
                Vector3Wide.Add(ref gravityDt, ref velocity.LinearVelocity, out var acceleratedLinearVelocity);
                var gravityMask = Vector.Equals(localInertias.InverseMass, Vector<float>.Zero);
                Vector3Wide.ConditionalSelect(ref gravityMask, ref velocity.LinearVelocity, ref acceleratedLinearVelocity, out velocity.LinearVelocity);
                //Implementation sidenote: Why aren't kinematics all bundled together separately from dynamics to avoid this condition?
                //Because kinematics can have a velocity- that is what distinguishes them from a static object. The solver must read velocities of all bodies involved in a constraint.
                //Under ideal conditions, those bodies will be near in memory to increase the chances of a cache hit. If kinematics are separately bundled, the the number of cache
                //misses necessarily increases. Slowing down the solver in order to speed up the pose integrator is a really, really bad trade, especially when the benefit is a few ALU ops.


            }
        }

        float cachedDt;
        Vector3Wide gravityDt;
        int bundlesPerJob;

        //Note that we aren't using a very cache-friendly work distribution here.
        //This is working on the assumption that the jobs will be large enough that border region cache misses won't be a big concern.
        //If this turns out to be false, this could be swapped over to a system similar to the solver-
        //preschedule offset regions for each worker to allow each one to consume a contiguous region before workstealing.
        int availableJobCount;
        void Worker(int workerIndex)
        {
            var bodyBundleCount = bodies.BodyBundleCount;
            while (true)
            {
                var jobIndex = Interlocked.Decrement(ref availableJobCount);
                if (jobIndex < 0)
                    return;
                var start = jobIndex * bundlesPerJob;
                var exclusiveEnd = start + bundlesPerJob;
                if (exclusiveEnd > bodyBundleCount)
                    exclusiveEnd = bodyBundleCount;
                Debug.Assert(exclusiveEnd > start, "Jobs that would involve bundles beyond the body count should not be created.");
                IntegrateBundles(start, exclusiveEnd, cachedDt);

            }

        }
        public void Update(float dt, IThreadDispatcher threadDispatcher = null)
        {
            var workerCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            var scalarGravityDt = Gravity * dt;
            Vector3Wide.CreateFrom(ref scalarGravityDt, out gravityDt);
            if (threadDispatcher != null)
            {
                //While we do technically support multithreading here, scaling is going to be really, really bad if the simulation gets kicked out of L3 cache in between frames.
                //The ratio of memory loads to actual compute work in this stage is extremely high, so getting scaling of 1.2x on a quad core is quite possible.
                //On the upside, it is a very short stage. With any luck, one or more of the following will hold:
                //1) the system has silly fast RAM,
                //2) the CPU supports octochannel memory and just brute forces the issue,
                //3) whatever the application is doing doesn't evict the entire L3 cache between frames.

                cachedDt = dt;
                const int jobsPerWorker = 4;
                var targetJobCount = workerCount * jobsPerWorker;
                var bodyBundleCount = bodies.BodyBundleCount;
                bundlesPerJob = bodyBundleCount / targetJobCount;
                if (bundlesPerJob == 0)
                    bundlesPerJob = 1;
                availableJobCount = bodyBundleCount / bundlesPerJob;
                if (bundlesPerJob * availableJobCount < bodyBundleCount)
                    ++availableJobCount;
                threadDispatcher.DispatchWorkers(Worker);
            }
            else
            {
                IntegrateBundles(0, bodies.BodyBundleCount, dt);
            }

        }
    }
}
