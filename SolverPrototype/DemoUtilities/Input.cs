using BEPUutilities2;
using OpenTK;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemoUtilities
{
    public class Input : IDisposable
    {
        NativeWindow window;
        //You could use GetState-like stuff to avoid the need to explicitly grab these, but shrug. This keeps events localized to just the window, and we can do a little logic of our own.
        HashSet<Key> anyDownedKeys = new HashSet<Key>();
        HashSet<Key> downedKeys = new HashSet<Key>();
        HashSet<Key> previousDownedKeys = new HashSet<Key>();
        HashSet<MouseButton> anyDownedButtons = new HashSet<MouseButton>();
        HashSet<MouseButton> downedButtons = new HashSet<MouseButton>();
        HashSet<MouseButton> previousDownedButtons = new HashSet<MouseButton>();
        
        /// <summary>
        /// Forces the mouse to stay at the center of the screen by recentering it on every flush.
        /// </summary>
        public bool MouseLocked
        {
            get; set;
        }
        Int2 WindowCenter { get { return new Int2(window.Width / 2, window.Height / 2); } }

        /// <summary>
        /// Gets or sets the mouse position in window coordinates without changing the net mouse delta.
        /// </summary>
        public Int2 MousePosition
        {
            get
            {
                var state = Mouse.GetCursorState();
                return new Int2(state.X, state.Y);
            }
            set
            {
                //Note that changing the cursor position does not change the raw mouse x/y.
                var screen = window.PointToScreen(new Point(value.X, value.Y));
                Mouse.SetPosition(screen.X, screen.Y);
            }
        }

        /// <summary>
        /// Gets the change in mouse position since the previous flush.
        /// </summary>
        public Int2 MouseDelta
        {
            get
            {
                return mouseDelta;
            }
        }

        /// <summary>
        /// Gets the amount of upward mouse wheel scrolling since the last flush regardless of how much downward scrolling occurred.
        /// </summary>
        public float ScrolledUp { get; private set; }
        /// <summary>
        /// Gets the amount of downward mouse wheel scrolling since the last flush regardless of how much upward scrolling occurred.
        /// </summary>
        public float ScrolledDown { get; private set; }
        /// <summary>
        /// Gets the mouse wheel scroll delta since the last flush.
        /// </summary>
        public float ScrollDelta {  get { return ScrolledUp + ScrolledDown; } }

        public Input(Window window)
        {
            this.window = window.window;
            this.window.KeyDown += KeyDown;
            this.window.KeyUp += KeyUp;
            this.window.MouseDown += MouseDown;
            this.window.MouseUp += MouseUp;
            this.window.MouseWheel += MouseWheel;
        }

        private void MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (e.DeltaPrecise > 0)
                ScrolledUp += e.DeltaPrecise;
            else
                ScrolledDown += e.DeltaPrecise;
        }

        private void MouseDown(object sender, MouseButtonEventArgs e)
        {
            anyDownedButtons.Add(e.Button);
            downedButtons.Add(e.Button);
        }
        private void MouseUp(object sender, MouseButtonEventArgs e)
        {
            downedButtons.Remove(e.Button);
        }

        private void KeyDown(object sender, KeyboardKeyEventArgs e)
        {
            anyDownedKeys.Add(e.Key);
            downedKeys.Add(e.Key);
        }
        private void KeyUp(object sender, KeyboardKeyEventArgs e)
        {
            downedKeys.Remove(e.Key);
        }



        /// <summary>
        /// Gets whether a key is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(Key key)
        {
            return downedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a key was down at the time of the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(Key key)
        {
            return previousDownedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a key that was not down in the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(Key key)
        {
            return !previousDownedKeys.Contains(key) && anyDownedKeys.Contains(key);
        }


        /// <summary>
        /// Gets whether a button is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(MouseButton button)
        {
            return downedButtons.Contains(button);
        }

        /// <summary>
        /// Gets whether a button was down at the time of the previous flush.
        /// </summary>
        /// <param name="mouseButton">Button to check.</param>
        /// <returns>True if the button was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(MouseButton mouseButton)
        {
            return previousDownedButtons.Contains(mouseButton);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a button that was not down in the previous flush.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(MouseButton button)
        {
            return !previousDownedButtons.Contains(button) && anyDownedButtons.Contains(button);
        }

        Int2 mouseDelta;
        Int2 previousRawMouse;
        public void Start()
        {
            var currentState = Mouse.GetState();
            //Given a long enough time, this could theoretically hit overflow.
            //But that would require hours of effort with a high DPI mouse, and this is a demo application...
            mouseDelta.X = currentState.X - previousRawMouse.X;
            mouseDelta.Y = currentState.Y - previousRawMouse.Y;
            previousRawMouse = new Int2(currentState.X, currentState.Y);
            if (MouseLocked)
            {
                //This is pretty doofy, but it works reasonably well and we don't have easy access to the windows-provided capture stuff through opentk (that I'm aware of?).
                //Could change it later if it matters, but realistically it won't matter.
                MousePosition = WindowCenter;
                window.CursorVisible = false;
            }
            else
            {
                window.CursorVisible = true;
            }
        }
        public void End()
        {
            anyDownedKeys.Clear();
            anyDownedButtons.Clear();
            previousDownedKeys.Clear();
            previousDownedButtons.Clear();
            previousDownedKeys.UnionWith(downedKeys);
            previousDownedButtons.UnionWith(downedButtons);
            ScrolledDown = 0;
            ScrolledUp = 0;
        }

        /// <summary>
        /// Unhooks the input management from the window.
        /// </summary>
        public void Dispose()
        {
            window.KeyDown -= KeyDown;
            window.KeyUp -= KeyUp;
            window.MouseDown -= MouseDown;
            window.MouseUp -= MouseUp;
        }
    }
}
