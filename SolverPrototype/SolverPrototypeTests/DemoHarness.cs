using BEPUutilities2;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;
using System;
using System.Numerics;
using System.Text;
using System.Threading;

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
        public Key ShowControls;
        public Key ChangeTimingDisplayMode;

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
                    MouseSensitivity = 3e-3f,
                    CameraMoveSpeed = 5,

                    LockMouse = Key.Tab,
                    Exit = Key.Escape,
                    ShowControls = Key.F1,
                    ChangeTimingDisplayMode = Key.F2
                };
            }

        }
    }

    public class DemoHarness : IDisposable
    {
        Window window;
        Renderer renderer;
        Input input;
        Camera camera;
        Controls controls;
        Font font;

        bool showControls;

        enum TimingDisplayMode
        {
            FullScreen,
            Regular,
            Minimized
        }

        TimingDisplayMode timingDisplayMode;
        Graph timingGraph;

        DemoSet demoSet;
        Demo demo;
        void ChangeToDemo(int demoIndex)
        {
            demo.Dispose();
            demo = demoSet.Build(demoIndex);
        }

        SimulationTimeSamples timeSamples = new SimulationTimeSamples(512);

        public DemoHarness(Window window, Renderer renderer, Input input, Camera camera, Font font,
            Controls? controls = null)
        {
            this.window = window;
            this.renderer = renderer;
            this.input = input;
            this.camera = camera;
            if (controls == null)
                this.controls = Controls.Default;
            this.font = font;

            timingGraph = new Graph(new GraphDescription
            {
                BodyMinimum = new Vector2(150, 100),
                BodySpan = new Vector2(200, 200),
                BodyLineColor = new Vector3(1, 1, 1),
                AxisLabelHeight = 16,
                AxisLineRadius = 0.5f,
                HorizontalAxisLabel = "Frames",
                VerticalAxisLabel = "Time (ms)",
                VerticalIntervalLabelRounding = 2,
                VerticalIntervalValueScale = 1e3f,
                BackgroundLineRadius = 0.125f,
                IntervalTextHeight = 12,
                IntervalTickRadius = 0.25f,
                IntervalTickLength = 6f,
                HorizontalTickCount = 5,
                VerticalTickCount = 5,
                HorizontalTickTextPadding = 0,
                VerticalTickTextPadding = 3,

                LegendMinimum = new Vector2(20, 200),
                LegendNameHeight = 14,
                LegendLineLength = 10,

                TextColor = new Vector3(1, 1, 1),
                Font = font,

                LineSpacingMultiplier = 1f
            });
            timingGraph.AddSeries("Total", new Vector3(0, 0, 0), 0.75f, timeSamples.Simulation);
            timingGraph.AddSeries("Body Opt", new Vector3(1, 0, 0), 0.125f, timeSamples.BodyOptimizer);
            timingGraph.AddSeries("Constraint Opt", new Vector3(0, 0, 1), 0.125f, timeSamples.ConstraintOptimizer);
            timingGraph.AddSeries("Batch Compress", new Vector3(0, 1, 0), 0.125f, timeSamples.BatchCompressor);
            timingGraph.AddSeries("Pose Integrator", new Vector3(1, 1, 0), 0.25f, timeSamples.PoseIntegrator);
            timingGraph.AddSeries("Solver", new Vector3(1, 0, 1), 0.5f, timeSamples.Solver);

            demoSet = new DemoSet();
            demo = demoSet.Build(0);
        }

        public void OnResize(Int2 resolution)
        {
            switch (timingDisplayMode)
            {
                case TimingDisplayMode.FullScreen:

                    break;
                case TimingDisplayMode.Regular:
                    break;
            }

        }

        public void Update(float dt)
        {
            //Don't bother responding to input if the window isn't focused.
            if (window.Focused)
            {
                if (input.WasPushed(controls.Exit))
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
                if (input.MouseLocked)
                {
                    var delta = input.MouseDelta;
                    if (delta.X != 0 || delta.Y != 0)
                    {
                        camera.Yaw += delta.X * controls.MouseSensitivity;
                        camera.Pitch += delta.Y * controls.MouseSensitivity;
                    }
                }
                if (input.WasPushed(controls.LockMouse))
                {
                    input.MouseLocked = !input.MouseLocked;
                }

                if (input.WasPushed(controls.ShowControls))
                {
                    showControls = !showControls;
                }
            }
            else
            {
                input.MouseLocked = false;
            }

            demo.Update(dt);
            timeSamples.RecordFrame(demo.Simulation);
        }

        float t = 0;
        StringBuilder uiText = new StringBuilder();
        public void Render(Renderer renderer)
        {
            //Perform any demo-specific rendering first.
            demo.Render(renderer);

            timingGraph.Description.Font = font;
            timingGraph.Draw(uiText, renderer.UILineBatcher, renderer.TextBatcher);
            
            t += 0.01f;
            for (int i = 0; i < 128; ++i)
            {

                var lineCenter = new Vector2(384 + i * 3f, 256);
                var angle = t + i * 0.05f;
                var lineOffset = 100 * new Vector2((float)Math.Sin(angle), (float)Math.Cos(angle));
                renderer.UILineBatcher.Draw(lineCenter + lineOffset, lineCenter - lineOffset,
                    0.5f,//20 * Math.Max(0, 1f + (float)Math.Cos(t)), 
                    new Vector3(1, i * (1f / 127f), 0));

            }
            uiText.Clear();
            float textHeight = 24;
            float lineSpacing = textHeight * 1.5f;
            var textColor = new Vector3(1, 1, 1);
            var horizontalAxis = new Vector2((float)Math.Sin(t), (float)Math.Cos(t));
            var controlsStart = new Vector2(window.Resolution.X - 400 + 100 * (float)Math.Sin(t), window.Resolution.Y - 100 - 100 * (float)Math.Cos(t));
            //Conveniently, enum strings are cached. Every (Key).ToString() returns the same reference for the same key, so no garbage worries.
            if (showControls)
            {
                var controlNamePosition = controlsStart;
                controlNamePosition.Y -= 12 * lineSpacing;
                uiText.Append("Controls: ");
                renderer.TextBatcher.Write(uiText, controlNamePosition, textHeight, textColor, font);
                controlNamePosition.Y += lineSpacing;

                var controlPosition = controlNamePosition;
                controlPosition.X += 80;

                void WriteName(string controlName)
                {
                    uiText.Length = 0;
                    uiText.Append(controlName);
                    uiText.Append(":");
                    renderer.TextBatcher.Write(uiText, controlNamePosition, textHeight, textColor, font);
                    controlNamePosition.Y += lineSpacing;
                }

                WriteName(nameof(controls.MoveForward));
                WriteName(nameof(controls.MoveBackward));
                WriteName(nameof(controls.MoveLeft));
                WriteName(nameof(controls.MoveRight));
                WriteName(nameof(controls.MoveUp));
                WriteName(nameof(controls.MoveDown));
                WriteName(nameof(controls.LockMouse));
                WriteName(nameof(controls.Exit));
                WriteName(nameof(controls.ShowControls));

                void WriteControl(string control)
                {
                    uiText.Length = 0;
                    uiText.Append(control);
                    renderer.TextBatcher.Write(uiText, controlPosition, textHeight, textColor, font);
                    controlPosition.Y += lineSpacing;
                }
                WriteControl(controls.MoveForward.ToString());
                WriteControl(controls.MoveBackward.ToString());
                WriteControl(controls.MoveLeft.ToString());
                WriteControl(controls.MoveRight.ToString());
                WriteControl(controls.MoveUp.ToString());
                WriteControl(controls.MoveDown.ToString());
                WriteControl(controls.LockMouse.ToString());
                WriteControl(controls.Exit.ToString());
                WriteControl(controls.ShowControls.ToString());
            }
            else
            {
                uiText.Append("Press ");
                uiText.Append(controls.ShowControls.ToString());
                uiText.Append(" for controls.");
                renderer.TextBatcher.Write(uiText, controlsStart, textHeight, horizontalAxis, textColor, font);
            }


        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                demo?.Dispose();
            }
        }

#if DEBUG
        ~DemoHarness()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif


    }
}
