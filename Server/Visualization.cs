using Microsoft.Kinect;
using OpenTK;
using OpenTK.Graphics.OpenGL;
using SharedData;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;
using System.Windows.Forms.Integration;
using System.Windows.Interop;
using System.Windows.Threading;

namespace ServerApp
{
    class Visualization
    {
        private Server server;
        private GLControl glControl;
        private DispatcherTimer timer;
        private bool idle;
        private double angle;
        private double xMovement;
        private double yMovement;
        private const float InferredZPositionClamp = 0.1f;
        private List<Tuple<JointType, JointType>> bones;

        private Color[] colors = {
            Color.Red, Color.DarkRed,
            Color.Green, Color.DarkGreen,
            Color.Blue, Color.DarkBlue,
            Color.Cyan, Color.DarkCyan,
            Color.Magenta, Color.DarkMagenta,
            Color.Yellow, Color.Orange
        };

        public Visualization(WindowsFormsHost winFormsHost, Server server)
        {
            this.server = server;

            this.DefineBones();

            //vytvorenie kontrolky                
            glControl = new GLControl();

            //pridanie eventov
            glControl.Load += glControl_Load;
            glControl.Paint += glControl_Paint;
            glControl.KeyDown += glControl_KeyDown; //reakcia na stlacanie klavesov            

            //automaticke prekreslenie pomocou timera (25FPS)
            timer = new DispatcherTimer();
            timer.Interval = new TimeSpan(0, 0, 0, 0, 40);
            timer.Tick += timer_onTick;

            //existuje event, ze ak formular nic nerobi, tak generuje event, ze nic nerobi :D vie sa to zijst
            ComponentDispatcher.ThreadIdle += (sender, e) => idle = true;

            OpenTK.Toolkit.Init();
            glControl.CreateControl();
            winFormsHost.Child = glControl;
        }
        //automaticke prekreslenie
        void timer_onTick(object sender, EventArgs e)
        {
            glControl.Invalidate();
        }
        //priprava gl kontrolky
        void glControl_Load(object sender, EventArgs e)
        {
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            angle = 180;
            yMovement = 3.5;

            timer.Start();
        }
        //prekreslenie kontrolky
        void glControl_Paint(object sender, PaintEventArgs e)
        {
            if (!idle)
                return;

            idle = false;
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            draw();
            glControl.SwapBuffers();
        }
        //ovladanie klavesami
        void glControl_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyData == Keys.W)
            {
                xMovement += -0.1 * Math.Sin(angle / 180 * Math.PI);
                yMovement += 0.1 * Math.Cos(angle / 180 * Math.PI);
            }
            if (e.KeyData == Keys.S)
            {
                xMovement += 0.1 * Math.Sin(angle / 180 * Math.PI);
                yMovement += -0.1 * Math.Cos(angle / 180 * Math.PI);
            }
            if (e.KeyData == Keys.A)
            {
                angle -= 0.5;
                angle %= 360;
            }
            if (e.KeyData == Keys.D)
            {
                angle += 0.5;
                angle %= 360;
            }
        }
        //metoda kreslenia
        void draw()
        {
            // load
            GL.ClearColor(Color.Black);
            GL.Viewport(0, 0, glControl.Width, glControl.Height);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();
            Matrix4 perspective = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, glControl.Width / glControl.Height, 0.1f, 100.0f);
            GL.LoadMatrix(ref perspective);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadIdentity();

            // rotacia a posun
            GL.LoadIdentity();
            GL.Rotate(angle, 0, 1, 0);
            GL.Translate(xMovement, 0, yMovement);

            // kreslenie bodov
            this.DrawBodies(server.BodiesDictionary.Values, colors);
        }

        public void DrawBodies(ICollection<SerializableBody[]> bodies, Color[] colors)
        {
            int i = 0;
            List<Joint> trackedBodiesCenters = new List<Joint>();

            foreach (SerializableBody[] bodyList in bodies)
            {
                foreach (SerializableBody body in bodyList)
                {
                    if (body == null || !body.IsTracked)
                    {
                        continue;
                    }

                    IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                    Dictionary<JointType, CameraSpacePoint> jointPoints3D = new Dictionary<JointType, CameraSpacePoint>();

                    foreach (JointType jointType in joints.Keys)
                    {
                        CameraSpacePoint position = joints[jointType].Position;
                        if (position.Z < 0)
                        {
                            position.Z = InferredZPositionClamp;
                        }
                        jointPoints3D[jointType] = position;
                    }

                    this.DrawBody(joints, jointPoints3D, colors[(2 * i) % colors.Length], colors[(2 * i + 1) % colors.Length]);
                    i++;

                    bool alreadyTracked = false;
                    foreach (Joint center in trackedBodiesCenters)
                    {
                        if (Server.DEBUG >= 2)
                            Console.WriteLine("Distance: " + server.EuclideanSquareDistance(center, body.Joints[JointType.SpineMid]));
                        if (server.EuclideanSquareDistance(center, body.Joints[JointType.SpineMid]) <= Server.MAX_SQUARE_DISTANCE)
                        {
                            alreadyTracked = true;
                            break;
                        }
                    }
                    if (!alreadyTracked)
                    {
                        trackedBodiesCenters.Add(body.Joints[JointType.SpineMid]);
                    }
                }
            }

            if (Server.DEBUG >= 2)
                Console.WriteLine("i: " + i + " TBC: " + trackedBodiesCenters.Count);
            server.UpdateTrackedNumber(trackedBodiesCenters.Count);
        }

        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, Dictionary<JointType, CameraSpacePoint> jointPoints3D, Color color, Color darkColor)
        {
            GL.PointSize(10);
            GL.Begin(PrimitiveType.Points);

            foreach (JointType jointType in joints.Keys)
            {
                if (joints[jointType].TrackingState == TrackingState.NotTracked)
                {
                    GL.Color3(Color.White);
                }
                else
                {
                    if (joints[jointType].TrackingState == TrackingState.Tracked)
                    {
                        GL.Color3(color);
                    }
                    else
                    {
                        GL.Color3(darkColor);
                    }
                }
                GL.Vertex3(CSPtoV3(jointPoints3D[jointType]));
            }

            GL.End();

            GL.Begin(PrimitiveType.Lines);
            GL.Color3(color);

            foreach (Tuple<JointType, JointType> bone in bones)
            {
                GL.Vertex3(CSPtoV3(jointPoints3D[bone.Item1]));
                GL.Vertex3(CSPtoV3(jointPoints3D[bone.Item2]));
            }

            GL.End();
        }

        private static float[] CSPtoV3(CameraSpacePoint csp)
        {
            float[] res = { csp.X, csp.Y, csp.Z };
            return res;
        }

        private void DefineBones()
        {
            // a bone defined as a line between two joints
            bones = new List<Tuple<JointType, JointType>>();

            // Torso
            bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
        }
    }
}
