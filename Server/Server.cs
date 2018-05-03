using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.IO;
using Newtonsoft.Json.Bson;
using Newtonsoft.Json;
using SharedData;
using System.Collections.Concurrent;
using Microsoft.Kinect;
using Accord.Math;
using Accord.Math.Decompositions;

namespace ServerApp
{
    public class ChangeStatusEventArgs : EventArgs
    {
        public string Status { get; set; }
    }

    public class UpdateConnectedDevicesEventArgs : EventArgs
    {
        public string Add { get; set; }
        public string Remove { get; set; }
    }

    public class UpdateTrackedNumberEventArgs : EventArgs
    {
        public int TrackedNumber { get; set; }
    }

    class Server
    {
        public const int SERVER_PORT = 8000;
        private const int MAX_SOCKETS = 10;
        private const int BUFFER_SIZE = 16384;
        private static readonly TimeSpan MIN_TIME_DIFF = new TimeSpan(0, 0, 0, 0, 1000 / 3);
        public const float MAX_SQUARE_DISTANCE = 0.2f;

        private const bool ADD = true;
        private const bool REMOVE = false;

        public const int DEBUG = 2;

        private Socket listener;
        private IPEndPoint ipEndPoint;

        private bool IsListening;
        private Object myLock = new Object();

        public event EventHandler<EventArgs> ShowNotificationEventHandler;

        public event EventHandler<ChangeStatusEventArgs> ChangeStatusEventHandler;

        public event EventHandler<UpdateConnectedDevicesEventArgs> UpdateConnectedDevicesEventHandler;

        public event EventHandler<UpdateTrackedNumberEventArgs> UpdateTrackedNumberHandler;

        public ConcurrentDictionary<string, SerializableBody[]> BodiesDictionary = new ConcurrentDictionary<string, SerializableBody[]>();

        private ConcurrentDictionary<string, Tuple<SerializableBody, DateTime>> savedPoses = new ConcurrentDictionary<string, Tuple<SerializableBody, DateTime>>();

        public List<TopologyEdge> Topology = new List<TopologyEdge>();

        private HashSet<string> ConnectedDevices = new HashSet<string>();

        public string GraphCenter;

        private Dictionary<string, int> vertices;

        private Dictionary<Tuple<int, int>, List<int>> paths;

        protected virtual void ShowNotification()
        {
            ShowNotificationEventHandler?.Invoke(this, null);
        }

        protected virtual void ChangeStatus(string status)
        {
            if (DEBUG >= 1)
                Console.WriteLine(status);
            ChangeStatusEventArgs args = new ChangeStatusEventArgs();
            args.Status = status;
            ChangeStatusEventHandler?.Invoke(this, args);
        }

        protected virtual void UpdateConnectedDevices(string deviceAddress, bool add)
        {
            UpdateConnectedDevicesEventArgs args = new UpdateConnectedDevicesEventArgs();
            if (add)
            {
                args.Add = deviceAddress;
                this.ConnectedDevices.Add(deviceAddress);
            }
            else
            {
                args.Remove = deviceAddress;
                this.ConnectedDevices.Remove(deviceAddress);
            }
            UpdateConnectedDevicesEventHandler?.Invoke(this, args);
        }

        public virtual void UpdateTrackedNumber(int trackedNumber)
        {
            UpdateTrackedNumberEventArgs args = new UpdateTrackedNumberEventArgs();
            args.TrackedNumber = trackedNumber;
            UpdateTrackedNumberHandler?.Invoke(this, args);
        }

        public List<IPAddress> getPossibleIpAddresses()
        {
            IPHostEntry ipHost = Dns.GetHostEntry(Dns.GetHostName());
            return ipHost.AddressList.Where(ip => ip.AddressFamily == AddressFamily.InterNetwork).ToList();
        }

        public void StartListening(IPAddress ipAddress, int port)
        {
            ChangeStatus("Start listening...");
            ipEndPoint = new IPEndPoint(ipAddress, port);

            listener = new Socket(ipAddress.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
            listener.Bind(ipEndPoint);
            listener.Listen(MAX_SOCKETS);

            IsListening = true;

            AsyncCallback asyncCallback = new AsyncCallback(AcceptCallback);
            if (IsListening)
            {
                listener.BeginAccept(asyncCallback, listener);
            }

            ChangeStatus("Listening on IP address " + ipEndPoint.Address + ", port " + ipEndPoint.Port + ".");
        }

        private void AcceptCallback(IAsyncResult ar)
        {
            if (!IsListening) return;

            Socket listener = (Socket)ar.AsyncState;
            Socket handler = listener.EndAccept(ar);
            handler.NoDelay = false;
            byte[] buffer = new byte[BUFFER_SIZE];
            List<byte> mainBuffer = new List<byte>();

            object[] obj = new object[3];
            obj[0] = buffer;
            obj[1] = handler;
            obj[2] = mainBuffer;

            handler.BeginReceive(buffer, 0, buffer.Length, SocketFlags.None, new AsyncCallback(ReceiveCallback), obj);

            AsyncCallback aCallback = new AsyncCallback(AcceptCallback);
            listener.BeginAccept(aCallback, listener);
        }

        private void ReceiveCallback(IAsyncResult ar)
        {
            object[] obj = new object[3];
            obj = (object[])ar.AsyncState;

            byte[] buffer = (byte[])obj[0];
            Socket handler = (Socket)obj[1];
            List<byte> mainBuffer = (List<byte>)obj[2];

            SocketError errorCode;
            int bytesRead = handler.EndReceive(ar, out errorCode);
            if (errorCode != SocketError.Success)
            {
                bytesRead = 0;
            }
            if (bytesRead > 0)
            {
                mainBuffer.AddRange(buffer);

                if (DEBUG >= 1)
                    Console.WriteLine(buffer.Length + " " + mainBuffer.Count + " " + handler.Available);

                if (handler.Available == 0)
                {
                    MemoryStream ms = new MemoryStream(mainBuffer.ToArray());
                    MessageData message;
                    JsonSerializer jsonSerializer = new JsonSerializer();
                    using (BsonDataReader reader = new BsonDataReader(ms))
                    {
                        try
                        {
                            message = jsonSerializer.Deserialize<MessageData>(reader);
                            if (DEBUG >= 1)
                                if (message.MessageType == SharedData.Type.Data)
                                {
                                    Console.WriteLine(handler.RemoteEndPoint.ToString() + " " + message);
                                }
                                else
                                {
                                    Console.WriteLine(handler.RemoteEndPoint.ToString() + " TimeSync");
                                }
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine("Error: " + e);
                            return;
                        }
                    }

                    if (message.MessageType == SharedData.Type.Data)
                    {
                        int oldCount = this.BodiesDictionary.Count;

                        // this.BodiesDictionary.AddOrUpdate(message.SrcIPAddress, message.Bodies, (k, v) => message.Bodies);

                        foreach (SerializableBody body in message.Bodies)
                        {
                            if (body.GestureDetected)
                            {
                                if (DEBUG >= 2)
                                    Console.WriteLine("-----------GESTURE------");

                                this.savedPoses.AddOrUpdate(message.SrcIPAddress, new Tuple<SerializableBody, DateTime>(body, message.Timestamp), (k, v) => new Tuple<SerializableBody, DateTime>(body, message.Timestamp));

                                foreach (KeyValuePair<string, Tuple<SerializableBody, DateTime>> entry in this.savedPoses)
                                {
                                    if (!entry.Key.Equals(message.SrcIPAddress) && (message.Timestamp - entry.Value.Item2) < MIN_TIME_DIFF)
                                    {
                                        // show notification
                                        this.ShowNotification();

                                        // ComputeTransformation
                                        double[,] coeffTemp = this.PrepareTransform(message.SrcIPAddress, body, entry.Key, entry.Value.Item1);

                                        if (DEBUG >= 2)
                                        {
                                            Console.WriteLine("COEFF BEGIN");
                                            for (int i = 0; i < coeffTemp.GetLength(0); i++)
                                            {
                                                for (int j = 0; j < coeffTemp.GetLength(1); j++)
                                                {
                                                    Console.Write(coeffTemp[i, j] + " ");
                                                }
                                                Console.WriteLine();
                                            }
                                            Console.WriteLine("COEFF END");
                                        }
                                    }
                                }
                            }
                        }

                        this.Transform(message);
                    }

                    if (message.MessageType == SharedData.Type.TimeSync)
                    {
                        int bytesSend = SendTimeSyncResponse(handler, jsonSerializer);
                        if (DEBUG >= 1)
                            Console.WriteLine("Sent TS: " + bytesSend);

                        this.UpdateConnectedDevices(message.SrcIPAddress, ADD);
                    }

                    mainBuffer.Clear();
                }

                handler.BeginReceive(buffer, 0, buffer.Length, SocketFlags.None, new AsyncCallback(ReceiveCallback), obj);
            }
            else
            {
                //handler.Shutdown();
                Console.WriteLine("Closing " + mainBuffer.Count);
                this.UpdateConnectedDevices(handler.RemoteEndPoint.ToString(), REMOVE);
                SerializableBody[] bodies;
                this.BodiesDictionary.TryRemove(handler.RemoteEndPoint.ToString(), out bodies);
                handler.Close();
            }
        }

        private void Transform(MessageData message)
        {
            double[,] coeff = null;

            // Graph Center - do not have to trasnform
            if (GraphCenter == null || message.SrcIPAddress.Equals(GraphCenter))
            {
                if (DEBUG >= 2)
                    Console.WriteLine("Transform: Graph Center Body " + message.SrcIPAddress);
                this.BodiesDictionary.AddOrUpdate(message.SrcIPAddress, message.Bodies, (k, v) => message.Bodies);
                return;
            }

            if (!vertices.ContainsKey(message.SrcIPAddress) || !vertices.ContainsKey(GraphCenter))
            {
                if (DEBUG >= 2)
                    Console.WriteLine("Not transforming (isolated) " + message.SrcIPAddress);
                this.BodiesDictionary.AddOrUpdate(message.SrcIPAddress, message.Bodies, (k, v) => message.Bodies);
                return;
            }

            int pathStart = vertices[message.SrcIPAddress];
            int pathEnd = vertices[GraphCenter];
            Tuple<int, int> pathEnds = new Tuple<int, int>(pathStart, pathEnd);

            // Isolated vertex - can not transform
            if (!paths.ContainsKey(pathEnds))
            {
                if (DEBUG >= 2)
                    Console.WriteLine("Not transforming (because of no path) " + message.SrcIPAddress);
                this.BodiesDictionary.AddOrUpdate(message.SrcIPAddress, message.Bodies, (k, v) => message.Bodies);
                return;
            }

            List<int> path = paths[pathEnds];

            if (DEBUG >= 2)
                Console.WriteLine("Path: " + String.Join(", ", path));

            for (int i = 1; i < path.Count; i++)
            {
                string oldAddress = null;
                string address = null;

                foreach (string vertex in vertices.Keys)
                {
                    if (vertices[vertex] == path[i])
                    {
                        address = vertex;
                    }
                    if (vertices[vertex] == path[i - 1])
                    {
                        oldAddress = vertex;
                    }
                }

                foreach (TopologyEdge edge in this.Topology)
                {
                    if (edge.SourceIPAddress.Equals(oldAddress) && edge.TargetIPAddress.Equals(address))
                    {
                        if (coeff == null)
                        {
                            coeff = edge.TransformMatrixCoefficients;
                        }
                        else
                        {
                            coeff = Matrix.Dot(coeff, edge.TransformMatrixCoefficients);
                        }
                        break;
                    }

                    if (edge.SourceIPAddress.Equals(address) && edge.TargetIPAddress.Equals(oldAddress))
                    {
                        if (coeff == null)
                        {
                            coeff = Matrix.Inverse(edge.TransformMatrixCoefficients);
                        }
                        else
                        {
                            coeff = Matrix.Dot(coeff, Matrix.Inverse(edge.TransformMatrixCoefficients));
                        }
                        break;
                    }
                }
            }

            if (coeff != null)
            {
                if (DEBUG >= 2)
                    Console.WriteLine("Transforming " + message.SrcIPAddress);
                this.TransformBodies(message.Bodies, coeff, message.SrcIPAddress);
                return;
            }

            if (DEBUG >= 2)
                Console.WriteLine("Did not transform " + message.SrcIPAddress);
        }

        public float ComputeErrorDistance(SerializableBody[] bodies1, SerializableBody[] bodies2)
        {
            float error = 0;

            for (int i = 0; i < bodies1.Length; i++)
            {
                if (bodies1[i] == null || bodies2[i] == null)
                {
                    continue;
                }

                foreach (JointType jointType in bodies1[i].Joints.Keys)
                {
                    float squareDistance = this.EuclideanSquareDistance(bodies1[i].Joints[jointType], bodies2[i].Joints[jointType]);
                    error += squareDistance;
                }
            }

            return error;
        }

        public float EuclideanSquareDistance(Joint joint1, Joint joint2)
        {
            float distance = 0;

            distance += (joint1.Position.X - joint2.Position.X) * (joint1.Position.X - joint2.Position.X);
            distance += (joint1.Position.Y - joint2.Position.Y) * (joint1.Position.Y - joint2.Position.Y);
            distance += (joint1.Position.Z - joint2.Position.Z) * (joint1.Position.Z - joint2.Position.Z);

            return distance;
        }

        // TODO Set just for gesture body! - Done
        private double[,] PrepareTransform(string srcAddress, SerializableBody srcBody, string tgtAddress, SerializableBody tgtBody)
        {

            double[,] matrixCoeff;

            foreach (TopologyEdge edge in Topology)
            {
                if (edge.SourceIPAddress.Equals(srcAddress) && edge.TargetIPAddress.Equals(tgtAddress))
                {
                    matrixCoeff = this.ComputeTransformation(srcBody, tgtBody, edge.Matching);
                    edge.TransformMatrixCoefficients = matrixCoeff;
                    return matrixCoeff;
                }
                else if (edge.SourceIPAddress.Equals(tgtAddress) && edge.TargetIPAddress.Equals(srcAddress))
                {
                    matrixCoeff = this.ComputeTransformation(tgtBody, srcBody, edge.Matching);
                    edge.TransformMatrixCoefficients = matrixCoeff;
                    return matrixCoeff;
                }
            }

            Dictionary<Joint, Joint> matching = new Dictionary<Joint, Joint>();
            matrixCoeff = this.ComputeTransformation(srcBody, tgtBody, matching);
            Topology.Add(new TopologyEdge(srcAddress, tgtAddress, matching, matrixCoeff));

            // find graph center
            this.FindGraphCenter();

            return matrixCoeff;
        }

        private void FindGraphCenter()
        {
            int MAX_DIST = int.MaxValue / 2 - 1;

            int verticesCount = this.ConnectedDevices.Count;
            vertices = new Dictionary<string, int>();
            paths = new Dictionary<Tuple<int, int>, List<int>>();
            int count = 0;
            foreach (string address in this.ConnectedDevices)
            {
                vertices.Add(address, count);
                count++;
            }
            int[,] distanceMatrix = new int[verticesCount, verticesCount];
            for (int i = 0; i < verticesCount; i++)
            {
                for (int j = 0; j < verticesCount; j++)
                {
                    distanceMatrix[i, j] = MAX_DIST;
                }
            }

            // create edges
            foreach (TopologyEdge edge in this.Topology)
            {
                distanceMatrix[vertices[edge.SourceIPAddress], vertices[edge.TargetIPAddress]] = 1;
                distanceMatrix[vertices[edge.TargetIPAddress], vertices[edge.SourceIPAddress]] = 1;

                Tuple<int, int> t = new Tuple<int, int>(vertices[edge.SourceIPAddress], vertices[edge.TargetIPAddress]);
                if (!paths.ContainsKey(t))
                {
                    paths.Add(t, new List<int> { vertices[edge.SourceIPAddress] });
                }
                Tuple<int, int> t2 = new Tuple<int, int>(vertices[edge.TargetIPAddress], vertices[edge.SourceIPAddress]);
                if (!paths.ContainsKey(t2))
                {
                    paths.Add(t2, new List<int> { vertices[edge.TargetIPAddress] });
                }
            }

            // Floyd-Warshall
            for (int k = 0; k < verticesCount; k++)
            {
                for (int i = 0; i < verticesCount; i++)
                {
                    for (int j = 0; j < verticesCount; j++)
                    {
                        if (distanceMatrix[i, k] + distanceMatrix[k, j] < distanceMatrix[i, j])
                        {
                            distanceMatrix[i, j] = distanceMatrix[i, k] + distanceMatrix[k, j];

                            // add edge to path
                            Tuple<int, int> t = new Tuple<int, int>(i, j);
                            if (!paths.ContainsKey(t))
                            {
                                paths.Add(t, new List<int> { i });
                            }
                            paths[t].Add(k);
                        }
                    }
                }
            }

            foreach (Tuple<int, int> t in paths.Keys)
            {
                paths[t].Add(t.Item2);
            }

            int graphCenterMaxDistance = int.MaxValue;
            for (int i = 0; i < verticesCount; i++)
            {
                int maxDistance = 0;
                for (int j = 0; j < verticesCount; j++)
                {
                    if (i != j && distanceMatrix[i, j] != MAX_DIST && distanceMatrix[i, j] > maxDistance)
                    {
                        maxDistance = distanceMatrix[i, j];
                    }
                }
                if (maxDistance != 0 && maxDistance < graphCenterMaxDistance)
                {
                    graphCenterMaxDistance = maxDistance;
                    foreach (string address in vertices.Keys)
                    {
                        if (vertices[address] == i)
                        {
                            this.GraphCenter = address;
                        }
                    }
                }
            }

            if (DEBUG >= 2)
            {
                for (int i = 0; i < verticesCount; i++)
                {
                    for (int j = 0; j < verticesCount; j++)
                    {
                        Console.Write(distanceMatrix[i, j] + " ");
                    }
                    Console.WriteLine();
                }
            }

            if (DEBUG >= 1)
                Console.WriteLine("Center: " + this.GraphCenter + " " + graphCenterMaxDistance);
        }

        private double[,] ComputeTransformation(SerializableBody messageBody, SerializableBody entryBody, Dictionary<Joint, Joint> matching)
        {
            if (entryBody == null || messageBody == null)
            {
                if (DEBUG >= 1)
                    Console.WriteLine("Transform unsuccessful - null value");
                return null;
            }

            JointType[] jointTypes = new JointType[] { JointType.Neck, JointType.SpineMid, JointType.ShoulderLeft, JointType.ShoulderRight };

            foreach (JointType jointType in jointTypes)
            {
                if (!matching.ContainsKey(messageBody.Joints[jointType]))
                    matching.Add(messageBody.Joints[jointType], entryBody.Joints[jointType]);
            }

            double[,] matrixCoeff = SolveMatrix(matching);

            return matrixCoeff;
        }

        private double[,] SolveMatrix(List<Joint> points, List<Joint> imagePoints)
        {
            double[,] matrix = new double[3 * points.Count, 12];
            double[] rightSide = new double[3 * points.Count];

            for (int j = 0; j < points.Count; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    matrix[3 * j + k, 4 * k] = points[j].Position.X;
                    matrix[3 * j + k, 4 * k + 1] = points[j].Position.Y;
                    matrix[3 * j + k, 4 * k + 2] = points[j].Position.Z;
                    matrix[3 * j + k, 4 * k + 3] = 1;
                }
                rightSide[3 * j] = imagePoints[j].Position.X;
                rightSide[3 * j + 1] = imagePoints[j].Position.Y;
                rightSide[3 * j + 2] = imagePoints[j].Position.Z;
            }

            double[] coefficientsVector = Matrix.Solve(matrix, rightSide, true);

            double[,] coefficients = new double[4, 4];
            for (int j = 0; j < coefficients.GetLength(0) - 1; j++)
            {
                for (int k = 0; k < coefficients.GetLength(1); k++)
                {
                    coefficients[j, k] = coefficientsVector[j * coefficients.GetLength(1) + k];
                }
            }
            for (int k = 0; k < coefficients.GetLength(1); k++)
            {
                coefficients[3, k] = (k == 3) ? 1 : 0;
            }

            return coefficients;
        }

        private double[,] SolveMatrix(Dictionary<Joint, Joint> matching)
        {
            double[,] matrix = new double[3 * matching.Count, 12];
            double[] rightSide = new double[3 * matching.Count];

            int i = 0;
            foreach (Joint key in matching.Keys)
            {
                for (int k = 0; k < 3; k++)
                {
                    matrix[3 * i + k, 4 * k] = key.Position.X;
                    matrix[3 * i + k, 4 * k + 1] = key.Position.Y;
                    matrix[3 * i + k, 4 * k + 2] = key.Position.Z;
                    matrix[3 * i + k, 4 * k + 3] = 1;
                }
                rightSide[3 * i] = matching[key].Position.X;
                rightSide[3 * i + 1] = matching[key].Position.Y;
                rightSide[3 * i + 2] = matching[key].Position.Z;
                i++;
            }

            QrDecomposition decomposition = new QrDecomposition(matrix);
            double[] coefficientsVector = decomposition.Solve(rightSide);

            double[,] coefficients = new double[4, 4];
            for (int j = 0; j < coefficients.GetLength(0) - 1; j++)
            {
                for (int k = 0; k < coefficients.GetLength(1); k++)
                {
                    coefficients[j, k] = coefficientsVector[j * coefficients.GetLength(1) + k];
                }
            }
            for (int k = 0; k < coefficients.GetLength(1); k++)
            {
                coefficients[3, k] = (k == 3) ? 1 : 0;
            }

            return coefficients;
        }


        private void TransformBodies(SerializableBody[] bodies, double[,] matrixCoeff, string sourceAddress)
        {
            SerializableBody[] transformedBodies = new SerializableBody[bodies.Length];

            for (int i = 0; i < transformedBodies.Length; i++)
            {
                if (bodies[i].IsTracked)
                {
                    transformedBodies[i] = TransformBody(bodies[i], matrixCoeff);
                }
            }

            this.BodiesDictionary.AddOrUpdate(sourceAddress, transformedBodies, (k, v) => transformedBodies);
        }

        public SerializableBody TransformBody(SerializableBody body, double[,] coefficients)
        {
            SerializableBody transformedBody = body.Clone();

            Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();

            foreach (KeyValuePair<JointType, Joint> entry in body.Joints)
            {
                joints.Add(entry.Key, TransformJoint(entry.Value, coefficients));
            }

            IReadOnlyDictionary<JointType, Joint> transformedJoints = joints;

            transformedBody.Joints = transformedJoints;

            return transformedBody;
        }

        private Joint TransformJoint(Joint point, double[,] coefficients)
        {
            Joint output = new Joint();

            output.JointType = point.JointType;
            CameraSpacePoint csp = new CameraSpacePoint();
            csp.X = (float)(coefficients[0, 0] * point.Position.X + coefficients[0, 1] * point.Position.Y + coefficients[0, 2] * point.Position.Z + coefficients[0, 3]);
            csp.Y = (float)(coefficients[1, 0] * point.Position.X + coefficients[1, 1] * point.Position.Y + coefficients[1, 2] * point.Position.Z + coefficients[1, 3]);
            csp.Z = (float)(coefficients[2, 0] * point.Position.X + coefficients[2, 1] * point.Position.Y + coefficients[2, 2] * point.Position.Z + coefficients[2, 3]);
            output.Position = csp;
            output.TrackingState = point.TrackingState;

            return output;
        }

        private static int SendTimeSyncResponse(Socket handler, JsonSerializer jsonSerializer)
        {
            MemoryStream ms;
            MessageData response = new MessageData();
            response.SrcIPAddress = handler.LocalEndPoint.ToString();
            response.MessageType = SharedData.Type.TimeSync;
            response.Timestamp = DateTime.Now;
            ms = new MemoryStream();
            using (BsonDataWriter writer = new BsonDataWriter(ms))
            {
                jsonSerializer.Serialize(writer, response);
            }
            byte[] msg = ms.ToArray();
            if (DEBUG >= 1)
                Console.WriteLine("Sending TS: " + response.Timestamp);
            return handler.Send(msg);
        }

        public void Close()
        {
            if (listener != null)
            {
                ChangeStatus("Disconnecting...");
                if (listener.Connected)
                {
                    listener.Shutdown(SocketShutdown.Receive);
                }
                //listener.Disconnect(true);
                IsListening = false;
                listener.Close();
                ChangeStatus("Disconnected.");
            }
        }
    }
}
