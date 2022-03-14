static class Program
{
    interface IHomogeneousCompoundShape
    {
        void RayTest<TRayHitHandler>();
    }

    struct Mesh : IHomogeneousCompoundShape
    {
        public unsafe void RayTest<TRayHitHandler>()
        {

        }
    }

    struct ShapeRayHitHandler<TRayHitHandler>
    {
    }


    static void RayTest<TShape, TRayHitHandler>() where TShape : unmanaged, IHomogeneousCompoundShape
    {
        default(TShape).RayTest<TRayHitHandler>();
    }
    static void Do<T>()
    {
        RayTest<Mesh, ShapeRayHitHandler<T>>();
    }

    public static void Main()
    {
        Do<int>();
    }
}

