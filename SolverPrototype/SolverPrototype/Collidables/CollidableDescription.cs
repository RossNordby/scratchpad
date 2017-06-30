using SolverPrototype.Collidables;

namespace SolverPrototype.Colldiables
{
    public struct CollidableDescription
    {
        public TypedIndex ShapeIndex;
        public float SpeculativeMargin;
        public ContinuousDetectionSettings Continuity;
    }
}
