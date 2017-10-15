# Q&A

##### 1. How do I make an object that can't be moved by outside influences, like other colliding dynamic bodies, but can still have a velocity?

Use a kinematic body. To create one, set the inverse mass and all components of the inverse inertia to zero in the body description passed to Simulation.Add. Kinematic bodies have effectively infinite mass and cannot be moved by any force. You can still change their velocity directly, though.

Any nonzero component in the inverse mass or inverse inertia results in a dynamic body.

Be careful when using kinematics- they are both unstoppable forces and immovable objects. If a dynamic body gets caught between a kinematic body and the static environment, there's a good chance the dynamic body will get pushed out of the level!

Also, if two kinematic bodies collide, a constraint will not be generated. Kinematics cannot respond to collisions, not even with other infinitely massive objects. They will simply continue to move along the path defined by their velocity.

##### 2. I made a body with zero inverse mass and nonzero inverse inertia and the simulation exploded/crashed! Why?

While dynamic bodies with zero inverse mass and nonzero inverse inertia tensors are technically allowed, they require extreme care. It is possible for constraints to be configured such that there is no solution, resulting in a division by zero. NaN values will propagate through the simulation and make everything explode.

To avoid the NaN-explosion, any constraint involving two bodies must be able to compute some local solution. For example, if two bodies have zero inverse mass but nonzero inverse inertia, you could create a ball socket joint that avoids issues by ensuring that the anchor offsets are nonzero.

This is harder to guarantee in the general case. Consider collision constraints created between the same two bodies. If a contact is created with zero offset from the object's center to the contact position, there will be no angular contribution to the constraint. Since the inverse masses are both zero, no local solution exists and a portal to NaNland will open.

Generally, avoid creating dynamic bodies with zero inverse mass unless you can absolutely guarantee that the involved constraints will never become degenerate. For example, if collisions are disabled, you don't have to worry about the automatically generated contact constraints, and you can tightly control what other constraints exist.

(You can also just use a constraint to keep an object positioned in one spot rather than setting its inverse mass to zero!)

##### 3. How can I ensure that the results of a simulation are deterministic (given the same inputs, the simulation produces the same physical result) on a single machine?

Take great care to ensure that every interaction with the physics simulation is reproduced in exactly the same order on each execution. This even includes the order of adds and removes!

Assuming that all external interactions with the engine are deterministic, the simulation is deterministic on a single machine when one of two conditions is met:
1. The simulation runs with only a single thread (that is, no IThreadDispatcher is provided to the time step function).
2. The simulation is provided multiple threads and the Simulation.Deterministic property is set to true.

The Deterministic property defaults to false. Ensuring determinism has a slight performance impact. It should be trivial for most simulations, but large and extremely chaotic simulations may take a few hundred microseconds more per frame.

##### 4. What do I do if I want determinism across different computers?

Hope that they happen to have exactly the same architecture so that every single instruction produces bitwise identical results. :(

If the target hardware is known to be identical (maybe a networked console game where users only play against other users of the exact same hardware), you might be fine. Sometimes, you'll even get lucky and end up with two different desktop processors that produce identical results.

But, in general, the only way to guarantee cross platform determinism is to avoid generating any instructions which may differ between hardware. A common solution here is to use fixed point rather than floating point math.

At the moment, BEPUphysics v2 does not support fixed point math out of the box, and it would be a pretty enormous undertaking to port it all over without destroying performance.

I may look into conditionally compiled alternative scalar types in the future. I can't guarantee when or if I'll get around to it, though; don't wait for me!

##### 5. I updated to the latest version of the physics library and simulations are producing different results than before, even though I set the Simulation.Deterministic property to true! What do?

Different versions of the library are not guaranteed to produce identical simulation results. Guaranteeing cross-version determinism would constrain development to an unacceptable degree.

If you need determinism of results over long periods (for example, storing game replays for later viewing), it's typically easiest to just fall back to something like storing keyframed animation. You can still use the simulation to help fill out details if you'd like- similar to a networked game receiving sparse updates from a server and filling in the details with extrapolated local simulation. There just has to be a way to correct for the gradual drift.

Such a drift correcting mechanism also compensates for the differences between processor architectures, so you'd gain the ability to share the replay across different hardware as a bonus.