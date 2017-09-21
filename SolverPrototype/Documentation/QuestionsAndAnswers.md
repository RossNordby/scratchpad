# Q&A

1. How do I make an object that can't be moved by outside influences, like other colliding dynamic bodies, but can still have a velocity?
A: Use a kinematic body. To create one, set the inverse mass and all components of the inverse inertia to zero in the body description passed to Simulation.Add. Kinematic bodies have effectively infinite mass and cannot be moved by any force. You can still change their velocity directly, though.

Any nonzero component in the inverse mass or inverse inertia results in a dynamic body.

Be careful when using kinematics- they are both unstoppable forces and immovable objects. If a dynamic body gets caught between a kinematic body and the static environment, there's a good chance the dynamic body will get pushed out of the level!

Also, if two kinematic bodies collide, a constraint will not be generated. Kinematics cannot respond to collisions, not even with other infinitely massive objects. They will simply continue to move along the path defined by their velocity.

2. I made a body with zero inverse mass and nonzero and the simulation exploded/crashed! Why?
A: While dynamic bodies with zero inverse mass and nonzero inverse inertia tensors are technically allowed, they require extreme care. It is possible for constraints to be configured such that there is no solution, resulting in a division by zero. NaN values will propagate through the simulation and make everything explode.

To avoid the NaN-explosion, any constraint involving two bodies must be able to compute some local solution. For example, if two bodies have zero inverse mass but nonzero inverse inertia, you could create a ball socket joint that avoids issues by ensuring that the anchor offsets are nonzero.

This is harder to guarantee in the general case. Consider collision constraints created between the same two bodies. If a contact is created with zero offset from the object's center to the contact position, there will be no angular contribution to the constraint. Since the inverse masses are both zero, no local solution exists and a portal to NaNland will open.

Generally, avoid creating dynamic bodies with zero inverse mass unless you can absolutely guarantee that the involved constraints will never become degenerate. For example, if collisions are disabled, you don't have to worry about the automatically generated contact constraints, and you can tightly control what other constraints exist.

(You can also just use a constraint to keep an object positioned in one spot rather than setting its inverse mass to zero!)