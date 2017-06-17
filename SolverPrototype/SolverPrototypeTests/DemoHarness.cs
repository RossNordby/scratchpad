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
        public Key ChangeTimingDisplayMode;
        public Key ShowControls;

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
                    ChangeTimingDisplayMode = Key.F2,
                    ShowControls = Key.F1,
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
            Regular,
            Big,
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
                BodyLineColor = new Vector3(1, 1, 1),
                AxisLabelHeight = 16,
                AxisLineRadius = 0.5f,
                HorizontalAxisLabel = "Frames",
                VerticalAxisLabel = "Time (ms)",
                VerticalIntervalValueScale = 1e3f,
                VerticalIntervalLabelRounding = 2,
                BackgroundLineRadius = 0.125f,
                IntervalTextHeight = 12,
                IntervalTickRadius = 0.25f,
                IntervalTickLength = 6f,
                TargetHorizontalTickCount = 5,
                HorizontalTickTextPadding = 0,
                VerticalTickTextPadding = 3,

                LegendMinimum = new Vector2(20, 200),
                LegendNameHeight = 12,
                LegendLineLength = 7,

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

            OnResize(window.Resolution);
        }

        private void UpdateTimingGraphForMode(TimingDisplayMode newDisplayMode)
        {
            timingDisplayMode = newDisplayMode;
            ref var description = ref timingGraph.Description;
            var resolution = window.Resolution;
            switch (timingDisplayMode)
            {
                case TimingDisplayMode.Big:
                    {
                        const float inset = 150;
                        description.BodyMinimum = new Vector2(inset);
                        description.BodySpan = new Vector2(resolution.X, resolution.Y) - description.BodyMinimum - new Vector2(inset);
                        description.LegendMinimum = description.BodyMinimum - new Vector2(110, 0);
                        description.TargetVerticalTickCount = 5;
                    }
                    break;
                case TimingDisplayMode.Regular:
                    {
                        const float inset = 50;
                        var targetSpan = new Vector2(400, 150);
                        description.BodyMinimum = new Vector2(resolution.X - targetSpan.X - inset, inset);
                        description.BodySpan = targetSpan;
                        description.LegendMinimum = description.BodyMinimum - new Vector2(130, 0);
                        description.TargetVerticalTickCount = 3;
                    }
                    break;
            }
            //In a minimized state, the graph is just not drawn.
        }

        public void OnResize(Int2 resolution)
        {
            UpdateTimingGraphForMode(timingDisplayMode);
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

                if (input.WasPushed(controls.ChangeTimingDisplayMode))
                {
                    var newDisplayMode = (int)timingDisplayMode + 1;
                    if (newDisplayMode > 2)
                        newDisplayMode = 0;
                    UpdateTimingGraphForMode((TimingDisplayMode)newDisplayMode);
                }
            }
            else
            {
                input.MouseLocked = false;
            }

            demo.Update(dt);
            timeSamples.RecordFrame(demo.Simulation);
        }

        StringBuilder uiText = new StringBuilder(128);
        public void Render(Renderer renderer)
        {
            //Perform any demo-specific rendering first.
            demo.Render(renderer);
#if DEBUG
            float warningHeight = 15f;
            renderer.TextBatcher.Write(uiText.Clear().Append("Running in Debug configuration. Compile in Release configuration for performance testing."),
                new Vector2((window.Resolution.X - GlyphBatch.MeasureLength(uiText, font, warningHeight)) * 0.5f, warningHeight), warningHeight, new Vector3(1, 0, 0), font);
#endif            
            float textHeight = 16;
            float lineSpacing = textHeight * 1.0f;
            var textColor = new Vector3(1, 1, 1);
            if (showControls)
            {
                var penPosition = new Vector2(window.Resolution.X - textHeight * 6 - 25, window.Resolution.Y - 25);
                penPosition.Y -= 10 * lineSpacing;
                uiText.Clear().Append("Controls: ");
                var headerHeight = textHeight * 1.2f;
                renderer.TextBatcher.Write(uiText, penPosition - new Vector2(0.5f * GlyphBatch.MeasureLength(uiText, font, headerHeight), 0), headerHeight, textColor, font);
                penPosition.Y += lineSpacing;

                var controlPosition = penPosition;
                controlPosition.X += textHeight * 0.5f;

                void WriteName(string controlName)
                {
                    uiText.Clear().Append(controlName).Append(":");
                    renderer.TextBatcher.Write(uiText, penPosition - new Vector2(GlyphBatch.MeasureLength(uiText, font, textHeight), 0), textHeight, textColor, font);
                    penPosition.Y += lineSpacing;
                }

                WriteName(nameof(controls.MoveForward));
                WriteName(nameof(controls.MoveBackward));
                WriteName(nameof(controls.MoveLeft));
                WriteName(nameof(controls.MoveRight));
                WriteName(nameof(controls.MoveUp));
                WriteName(nameof(controls.MoveDown));
                WriteName(nameof(controls.LockMouse));
                WriteName(nameof(controls.Exit));
                WriteName(nameof(controls.ChangeTimingDisplayMode));
                WriteName(nameof(controls.ShowControls));

                void WriteControl(string control)
                {
                    uiText.Clear().Append(control);
                    renderer.TextBatcher.Write(uiText, controlPosition, textHeight, textColor, font);
                    controlPosition.Y += lineSpacing;
                }
                //Conveniently, enum strings are cached. Every (Key).ToString() returns the same reference for the same key, so no garbage worries.
                WriteControl(controls.MoveForward.ToString());
                WriteControl(controls.MoveBackward.ToString());
                WriteControl(controls.MoveLeft.ToString());
                WriteControl(controls.MoveRight.ToString());
                WriteControl(controls.MoveUp.ToString());
                WriteControl(controls.MoveDown.ToString());
                WriteControl(controls.LockMouse.ToString());
                WriteControl(controls.Exit.ToString());
                WriteControl(controls.ChangeTimingDisplayMode.ToString());
                WriteControl(controls.ShowControls.ToString());
            }
            else
            {
                uiText.Clear().Append("Press ").Append(controls.ShowControls.ToString()).Append(" for controls.");
                const float inset = 25;
                renderer.TextBatcher.Write(uiText,
                    new Vector2(window.Resolution.X - inset - GlyphBatch.MeasureLength(uiText, font, textHeight), window.Resolution.Y - inset),
                    textHeight, textColor, font);
            }

            if (timingDisplayMode != TimingDisplayMode.Minimized)
            {
                timingGraph.Draw(uiText, renderer.UILineBatcher, renderer.TextBatcher);
            }
            else
            {
                const float timingTextSize = 14;
                const float inset = 25;
                renderer.TextBatcher.Write(
                    uiText.Clear().Append(Math.Round(1e3 * timeSamples.Simulation[timeSamples.Simulation.End - 1], timingGraph.Description.VerticalIntervalLabelRounding)).Append(" ms/step"),
                    new Vector2(window.Resolution.X - inset - GlyphBatch.MeasureLength(uiText, font, timingTextSize), inset), timingTextSize, timingGraph.Description.TextColor, font);
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
