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
                    ShowControls = Key.F1
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
        Font font;

        bool showControls;

        Graph graph;

        class TestSeries : IDataSeries
        {
            public float Value;
            public float Scale;
            public float Width;
            public TestSeries(float baseValue, float scale, float width)
            {
                Scale = scale;
                Value = baseValue;
                Width = width;
            }
            public float this[int index] => Scale * (1f + (float)Math.Sin(Value + index / Width));

            public int Start => 0;

            public int End => 256;
        }

        TestSeries testSeries0 = new TestSeries(0f, 1f, 200);
        TestSeries testSeries1 = new TestSeries(1f, 0.8f, 100);
        TestSeries testSeries2 = new TestSeries(2f, 1.2f, 400);

        public BasicDemo(Window window, Input input, Camera camera, Font font, Controls? controls = null)
        {
            this.window = window;
            this.input = input;
            this.camera = camera;
            if (controls == null)
                this.controls = Controls.Default;
            this.font = font;

            graph = new Graph(new GraphDescription
            {
                BodyMinimum = new Vector2(150, 100),
                BodySpan = new Vector2(200, 200),
                BodyLineColor = new Vector3(1, 1, 1),
                AxisLabelHeight = 16,
                AxisLineRadius = 0.5f,
                HorizontalAxisLabel = "Frames",
                VerticalAxisLabel = "Time (ms)",
                VerticalIntervalLabelRounding = 1,
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
            graph.AddSeries("yee", new Vector3(0, 0, 1), 0.25f, testSeries0);
            graph.AddSeries("laryngitis", new Vector3(0, 1, 1), 0.5f, testSeries1);
            graph.AddSeries("torb", new Vector3(1, 0, 1), 0.75f, testSeries2);
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
        }

        float t = 0;
        StringBuilder uiText = new StringBuilder();
        public void Render(Renderer renderer)
        {
            graph.Description.Font = font;
            graph.Draw(uiText, renderer.UILineBatcher, renderer.TextBatcher);
            testSeries0.Value += 0.02f;
            testSeries1.Value += 0.03f;
            testSeries2.Value += 0.04f;
            testSeries0.Scale = 1 + 0.5f * (float)Math.Sin(t);
            testSeries1.Scale = 1 + 0.3f * (float)Math.Sin(t);
            testSeries2.Scale = 1 + 0.1f * (float)Math.Sin(t);
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
    }
}
