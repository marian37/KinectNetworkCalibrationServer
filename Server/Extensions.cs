using Microsoft.Kinect;
using SharedData;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace ServerApp
{
    public static class Extensions
    {
        public const float InferredZPosition = 0.1f;

        public const int DEPTH_SPACE_MAX_WIDTH = 512;

        public const int DEPTH_SPACE_MAX_HEIGHT = 424;

        public const int COLOR_SPACE_MAX_WIDTH = 1920;

        public const int COLOR_SPACE_MAX_HEIGHT = 1080;

        private static Color[] color = {
            Colors.Red, Colors.DarkRed,
            Colors.Green, Colors.DarkGreen,
            Colors.Blue, Colors.DarkBlue,
            Colors.Cyan, Colors.DarkCyan,
            Colors.Magenta, Colors.DarkMagenta,
            Colors.Yellow, Colors.Orange
        };

        private static List<Tuple<JointType, JointType>> InitializeBones()
        {
            List<Tuple<JointType, JointType>> bones = new List<Tuple<JointType, JointType>>();

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

            return bones;
        }

        public static void DrawBodies(this Canvas canvas, SerializableBody[] bodies)
        {
            canvas.Children.Clear();
            int bodyColor = 0;
            for (int i = 0; i < bodies.Length; i++)
            {
                if (bodies[i] != null)
                {
                    if (bodies[i].IsTracked)
                    {
                        IReadOnlyDictionary<JointType, Joint> joints = bodies[i].Joints;
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                        foreach (JointType jointType in joints.Keys)
                        {
                            CameraSpacePoint position = joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPosition;
                            }

                            double x = position.X * canvas.ActualWidth;
                            double y = position.Y * canvas.ActualHeight;
                            jointPoints[jointType] = new Point(x, y);
                        }

                        canvas.DrawSkeleton(joints, jointPoints, color[2 * bodyColor], color[2 * bodyColor + 1]);
                    }
                }
                bodyColor++;
            }
        }

        public static void DrawSkeleton(this Canvas canvas, IReadOnlyDictionary<JointType, Joint> joints, Dictionary<JointType, Point> jointPoints, Color primaryColor, Color secondaryColor)
        {
            foreach (JointType jointType in joints.Keys)
            {
                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    canvas.DrawPoint(jointPoints, jointType, primaryColor);
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    canvas.DrawPoint(jointPoints, jointType, secondaryColor);
                }
            }

            foreach (Tuple<JointType, JointType> bone in InitializeBones())
            {
                canvas.DrawLine(joints, jointPoints, bone.Item1, bone.Item2, primaryColor, secondaryColor);
            }
        }

        public static void DrawPoint(this Canvas canvas, Dictionary<JointType, Point> jointPoints, JointType jointType, Color color)
        {
            Ellipse ellipse = new Ellipse
            {
                Width = 14,
                Height = 14,
                Fill = new SolidColorBrush(color)
            };

            Canvas.SetLeft(ellipse, jointPoints[jointType].X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, jointPoints[jointType].Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        public static void DrawLine(this Canvas canvas, IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, Color primaryColor, Color secondaryColor)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            SolidColorBrush brush = new SolidColorBrush(secondaryColor);
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                brush = new SolidColorBrush(primaryColor);
            }

            Line line = new Line
            {
                X1 = jointPoints[jointType0].X,
                Y1 = jointPoints[jointType0].Y,
                X2 = jointPoints[jointType1].X,
                Y2 = jointPoints[jointType1].Y,
                StrokeThickness = 5,
                Stroke = brush
            };

            canvas.Children.Add(line);
        }
    }
}
