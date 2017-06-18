﻿using BEPUutilities2;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Quaternion = BEPUutilities2.Quaternion;

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
        /// Gets the orientation quaternion of the camera.
        /// </summary>
        public Quaternion OrientationQuaternion
        {
            get
            {
                Quaternion.CreateFromYawPitchRoll(-yaw, -pitch, 0, out var orientationQuaternion);
                return orientationQuaternion;
            }
        }

        /// <summary>
        /// Gets the orientation transform of the camera.
        /// </summary>
        public Matrix Orientation
        {
            get
            {
                return Matrix.CreateFromQuaternion(OrientationQuaternion);
            }
        }

        /// <summary>
        /// Gets the right direction of the camera. Equivalent to transforming (1,0,0) by Orientation.
        /// </summary>
        public Vector3 Right
        {
            get
            {
                var orientation = OrientationQuaternion;
                Quaternion.TransformX(1, ref orientation, out var right);
                return right;
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
                var orientation = OrientationQuaternion;
                Quaternion.TransformY(1, ref orientation, out var up);
                return up;
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
                var orientation = OrientationQuaternion;
                Quaternion.TransformZ(1, ref orientation, out var backward);
                return backward;
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
        public Matrix World
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
        public Matrix View
        {
            get
            {
                return Matrix.Invert(World);
            }
        }

        /// <summary>
        /// Gets the projection transform of the camera using reversed depth.
        /// </summary>
        public Matrix Projection
        {
            get
            {
                //Note the flipped near/far! Reversed depth. Better precision distribution. Unlikely that we'll take advantage of it in the demos, but hey, it's free real estate.
                return Matrix.CreatePerspectiveFieldOfView(FieldOfView, AspectRatio, FarClip, NearClip);
            }
        }

        /// <summary>
        /// Gets the combined view * projection of the camera.
        /// </summary>
        public Matrix ViewProjection
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
