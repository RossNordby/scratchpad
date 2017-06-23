using DemoUtilities;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototypeTests
{
    public enum HoldableControlType
    {
        Key,
        MouseButton,
    }
    /// <summary>
    /// A control binding which can be held for multiple frames.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct HoldableBind
    {
        [FieldOffset(0)]
        public Key Key;
        [FieldOffset(0)]
        public MouseButton Button;
        [FieldOffset(4)]
        public HoldableControlType Type;

        public HoldableBind(Key key)
            : this()
        {
            Key = key;
            Type = HoldableControlType.Key;
        }
        public HoldableBind(MouseButton button)
            : this()
        {
            Button = button;
            Type = HoldableControlType.MouseButton;
        }

        public static implicit operator HoldableBind(Key key)
        {
            return new HoldableBind(key);
        }
        public static implicit operator HoldableBind(MouseButton button)
        {
            return new HoldableBind(button);
        }
        public bool IsDown(Input input)
        {
            if (Type == HoldableControlType.Key)
                return input.IsDown(Key);
            return input.IsDown(Button);
        }

        public bool WasPushed(Input input)
        {
            if (Type == HoldableControlType.Key)
                return input.WasPushed(Key);
            return input.WasPushed(Button);
        }

        public override string ToString()
        {
            if (Type == HoldableControlType.Key)
                return Key.ToString();
            return Button.ToString();
        }
    }

    public enum InstantControlType
    {
        Key,
        MouseButton,
        MouseWheel,
    }
    public enum MouseWheelAction
    {
        ScrollUp,
        ScrollDown
    }
    /// <summary>
    /// A control binding that supports any form of instant action, but may or may not support being held.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct InstantBind
    {
        [FieldOffset(0)]
        public Key Key;
        [FieldOffset(0)]
        public MouseButton Button;
        [FieldOffset(0)]
        public MouseWheelAction Wheel;
        [FieldOffset(4)]
        public InstantControlType Type;

        public InstantBind(Key key)
            : this()
        {
            Key = key;
            Type = InstantControlType.Key;
        }
        public InstantBind(MouseButton button)
            : this()
        {
            Button = button;
            Type = InstantControlType.MouseButton;
        }
        public InstantBind(MouseWheelAction wheelAction)
            : this()
        {
            Wheel = wheelAction;
            Type = InstantControlType.MouseWheel;
        }

        public static implicit operator InstantBind(Key key)
        {
            return new InstantBind(key);
        }
        public static implicit operator InstantBind(MouseButton button)
        {
            return new InstantBind(button);
        }
        public static implicit operator InstantBind(MouseWheelAction wheelAction)
        {
            return new InstantBind(wheelAction);
        }

        public bool WasTriggered(Input input)
        {
            switch (Type)
            {
                case InstantControlType.Key:
                    return input.WasPushed(Key);
                case InstantControlType.MouseButton:
                    return input.WasPushed(Button);
                case InstantControlType.MouseWheel:
                    return Wheel == MouseWheelAction.ScrollUp ? input.ScrolledUp > 0 : input.ScrolledDown < 0;
            }
            return false;
        }

        public override string ToString()
        {
            switch (Type)
            {
                case InstantControlType.Key:
                    return Key.ToString();
                case InstantControlType.MouseButton:
                    return Button.ToString();
                case InstantControlType.MouseWheel:
                    return Wheel.ToString();
            }
            return "";
        }
    }

    public struct Controls
    {
        public HoldableBind MoveForward;
        public HoldableBind MoveBackward;
        public HoldableBind MoveLeft;
        public HoldableBind MoveRight;
        public HoldableBind MoveUp;
        public HoldableBind MoveDown;
        public InstantBind MoveSlower;
        public InstantBind MoveFaster;
        public float MouseSensitivity;
        public float CameraSlowMoveSpeed;
        public float CameraMoveSpeed;
        public float CameraFastMoveSpeed;

        public InstantBind LockMouse;
        public InstantBind Exit;
        public InstantBind ChangeTimingDisplayMode;
        public InstantBind ShowControls;

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
                    MoveSlower = MouseWheelAction.ScrollDown,
                    MoveFaster = MouseWheelAction.ScrollUp,
                    MouseSensitivity = 3e-3f,
                    CameraSlowMoveSpeed = 0.5f,
                    CameraMoveSpeed = 5,
                    CameraFastMoveSpeed = 50,

                    LockMouse = Key.Tab,
                    Exit = Key.Escape,
                    ChangeTimingDisplayMode = Key.F2,
                    ShowControls = Key.F1,
                };
            }

        }
    }
}
