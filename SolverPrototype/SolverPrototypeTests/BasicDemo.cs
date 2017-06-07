using DemoRenderer;
using DemoUtilities;
using OpenTK.Input;
using System.Numerics;

namespace SolverPrototypeTests
{
    public struct Controls
    {
        public Key MoveForward;
        public Key MoveBackward;
        public Key MoveLeft;
        public Key MoveRight;
        public Key MoveUp;
        public Key MoveDown;
        public float MouseSensitivity;
        public float CameraMoveSpeed;

        public Key LockMouse;
        public Key Exit;

        public static Controls Default
        {
            get
            {
                return new Controls
                {
                    MoveForward = Key.W,
                    MoveBackward = Key.S,
                    MoveLeft = Key.A,
                    MoveRight = Key.D,
                    MoveDown = Key.ControlLeft,
                    MoveUp = Key.ShiftLeft,
                    MouseSensitivity = 1e-2f,
                    CameraMoveSpeed = 5,

                    LockMouse = Key.Tab,
                    Exit = Key.Escape
                };
            }

        }
    }

    public class BasicDemo
    {
        Window window;
        Input input;
        Camera camera;
        Controls controls;


        public BasicDemo(Window window, Input input, Camera camera, Controls? controls = null)
        {
            this.window = window;
            this.input = input;
            this.camera = camera;
            if (controls == null)
                this.controls = Controls.Default;
        }

        public void Update(float dt)
        {
            if(input.WasPushed(controls.Exit))
            {
                window.Close();
                return;
            }
            var cameraOffset = new Vector3();
            if (input.IsDown(controls.MoveForward))
                cameraOffset += camera.Forward;
            if (input.IsDown(controls.MoveBackward))
                cameraOffset += camera.Backward;
            if (input.IsDown(controls.MoveLeft))
                cameraOffset += camera.Left;
            if (input.IsDown(controls.MoveRight))
                cameraOffset += camera.Right;
            if (input.IsDown(controls.MoveUp))
                cameraOffset += camera.Up;
            if (input.IsDown(controls.MoveDown))
                cameraOffset += camera.Down;
            var length = cameraOffset.Length();
            if (length > 1e-7f)
                cameraOffset *= dt * controls.CameraMoveSpeed / length;
            else
                cameraOffset = new Vector3();
            camera.Position += cameraOffset;

            camera.Yaw += input.MouseDelta.X * controls.MouseSensitivity;
            camera.Pitch += input.MouseDelta.Y * controls.MouseSensitivity;

            if (input.WasPushed(controls.LockMouse))
            {
                input.MouseLocked = !input.MouseLocked;
            }

            

        }
    }
}
