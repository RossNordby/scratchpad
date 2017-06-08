using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace DemoRenderer
{
    /// <summary>
    /// Simple yaw/pitch up-locked camera.
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Gets or sets the position of the camera.
        /// </summary>
        public Vector3 Position { get; set; }

        float yaw;
        /// <summary>
        /// Gets or sets the yaw of the camera as a value from -PI to PI. At 0, Forward is aligned with -z. At PI/2, Forward is aligned with +x. In other words, higher values turn right.
        /// </summary>
        public float Yaw
        {
            get { return yaw; }
            set
            {
                var revolution = (value + Math.PI) / (2 * Math.PI);
                revolution -= Math.Floor(revolution);
                yaw = (float)(revolution * (Math.PI * 2) - Math.PI);
            }
        }
        float pitch;
        /// <summary>
        /// Gets or sets the pitch of the camera, clamped to a value from -MaximumPitch to MaximumPitch. Higher values look downward, lower values look upward.
        /// </summary>
        public float Pitch
        {
            get { return pitch; }
            set { pitch = Math.Clamp(value, -maximumPitch, maximumPitch); }
        }

        float maximumPitch;
        /// <summary>
        /// Gets or sets the maximum pitch of the camera, a value from 0 to PI / 2.
        /// </summary>
        public float MaximumPitch
        {
            get { return maximumPitch; }
            set { maximumPitch = (float)Math.Clamp(value, 0, Math.PI / 2); }
        }

        /// <summary>
        /// Gets or sets the aspect ratio of the camera.
        /// </summary>
        public float AspectRatio { get; set; }

        /// <summary>
        /// Gets or sets the field of view of the camera.
        /// </summary>
        public float FieldOfView { get; set; }

        /// <summary>
        /// Gets or sets the near plane of the camera.
        /// </summary>
        public float NearClip { get; set; }

        /// <summary>
        /// Gets or sets the far plane of the camera.
        /// </summary>
        public float FarClip { get; set; }

        //All of this could be quite a bit faster, but wasting a few thousand cycles per frame isn't exactly a concern.

        /// <summary>
        /// Gets the orientation transform of the camera.
        /// </summary>
        public Matrix4x4 Orientation
        {
            get
            {
                return Matrix4x4.CreateFromYawPitchRoll(-yaw, -pitch, 0);
            }
        }


        /// <summary>
        /// Gets the right direction of the camera. Equivalent to transforming (1,0,0) by Orientation.
        /// </summary>
        public Vector3 Right
        {
            get
            {
                var orientation = Orientation;
                return new Vector3(orientation.M11, orientation.M12, orientation.M13);
            }
        }
        /// <summary>
        /// Gets the left direction of the camera. Equivalent to transforming (-1,0,0) by Orientation.
        /// </summary>
        public Vector3 Left
        {
            get
            {
                return -Right;
            }
        }
        /// <summary>
        /// Gets the up direction of the camera. Equivalent to transforming (0,1,0) by Orientation.
        /// </summary>
        public Vector3 Up
        {
            get
            {
                var orientation = Orientation;
                return new Vector3(orientation.M21, orientation.M22, orientation.M23);
            }
        }
        /// <summary>
        /// Gets the down direction of the camera. Equivalent to transforming (0,-1,0) by Orientation.
        /// </summary>
        public Vector3 Down
        {
            get
            {
                return -Up;
            }
        }
        /// <summary>
        /// Gets the backward direction of the camera. Equivalent to transforming (0,0,1) by Orientation.
        /// </summary>
        public Vector3 Backward
        {
            get
            {
                var orientation = Orientation;
                return new Vector3(orientation.M31, orientation.M32, orientation.M33);
            }
        }
        /// <summary>
        /// Gets the forward direction of the camera. Equivalent to transforming (0,0,-1) by Orientation.
        /// </summary>
        public Vector3 Forward
        {
            get
            {
                return -Backward;
            }
        }

        /// <summary>
        /// Gets the world transform of the camera.
        /// </summary>
        public Matrix4x4 World
        {
            get
            {
                var world = Orientation;
                world.Translation = Position;
                return world;
            }
        }

        /// <summary>
        /// Gets the view transform of the camera.
        /// </summary>
        public Matrix4x4 View
        {
            get
            {
                Matrix4x4.Invert(World, out var result);
                return result;
            }
        }


        /// <summary>
        /// Gets the projection transform of the camera using reversed depth.
        /// </summary>
        public Matrix4x4 Projection
        {
            get
            {
                //Note the flipped near/far! Reversed depth. Better precision distribution. Unlikely that we'll take advantage of it in the demos, but hey, it's free real estate.
                //The Matrix4x4 built in perspective function throws an argument exception for flipped near/far, unfortunately.
                float h = 1f / ((float)Math.Tan(FieldOfView * 0.5f));
                float w = h / AspectRatio;
                Matrix4x4 perspective;
                perspective.M11 = w;
                perspective.M12 = 0;
                perspective.M13 = 0;
                perspective.M14 = 0;

                perspective.M21 = 0;
                perspective.M22 = h;
                perspective.M23 = 0;
                perspective.M24 = 0;

                perspective.M31 = 0;
                perspective.M32 = 0;
                perspective.M33 = NearClip / (FarClip - NearClip);
                perspective.M34 = -1;

                perspective.M41 = 0;
                perspective.M42 = 0;
                perspective.M44 = 0;
                perspective.M43 = FarClip * perspective.M33;
                return perspective;
            }
        }

        /// <summary>
        /// Gets the combined view * projection of the camera.
        /// </summary>
        public Matrix4x4 ViewProjection
        {
            get
            {
                return View * Projection;
            }
        }


        public Camera(float aspectRatio, float fieldOfView, float nearClip, float farClip, float maximumPitch = (float)(Math.PI / 2))
        {
            AspectRatio = aspectRatio;
            FieldOfView = fieldOfView;
            MaximumPitch = maximumPitch;
            NearClip = nearClip;
            FarClip = farClip;
        }


    }
}
