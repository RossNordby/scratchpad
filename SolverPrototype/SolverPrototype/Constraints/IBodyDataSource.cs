namespace SolverPrototype.Constraints
{
    public interface IBodyDataSource
    {
        void GatherInertiaAndPose(ref UnpackedTwoBodyReferences bodyReferences, 
            out Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB, 
            out BodyInertias inertiaA, out BodyInertias inertiaB);

        void GatherInertia(ref UnpackedTwoBodyReferences bodyReferences,
            out BodyInertias inertiaA, out BodyInertias inertiaB);
    }
}