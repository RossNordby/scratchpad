
//using BepuUtilities;
//using BepuUtilities.Memory;
//using System.Diagnostics;
//using System.Runtime.CompilerServices;

//public class Shapes
//{
//    ShapeBatch[] batches;

//    public ShapeBatch this[int typeIndex] => batches[typeIndex];


//    public Shapes()
//    {
//        //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
//        batches = new ShapeBatch[16];
//    }

//}