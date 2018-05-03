using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading.Tasks;

namespace SharedData
{
    public enum Type { TimeSync, Data };

    public enum MovingState { Moving, Stand };

    public enum CalibrationState { Uncalibrated, RaisedHand, SavedPose };

    public enum RaisedHand { None, Right, Left };

    public class MessageData
    {
        public Type MessageType { get; set; }

        public string SrcIPAddress { get; set; }

        public DateTime Timestamp { get; set; }

        public SerializableBody[] Bodies { get; set; }

        public override string ToString()
        {
            string tracked = "";
            for (int i = 0; i < Bodies.Length; i++)
            {
                tracked += Bodies[i].IsTracked ? "T" : "N";
                tracked += Bodies[i].GestureDetected ? "G" : "O";
            }
            return "From: " + SrcIPAddress + " Type: " + MessageType + " at: " + Timestamp + "\n" +
                Bodies.Length + " " + tracked;
        }
    }

    public class SerializableBody
    {
        public bool IsTracked { get; set; }

        public IReadOnlyDictionary<JointType, Joint> Joints { get; set; }

        public MovingState MovingState { get; set; }

        public CalibrationState CalibrationState { get; set; }

        public RaisedHand RaisedHand { get; set; }

        public bool GestureDetected { get; set; }

        public ulong TrackingID { get; set; }

        public SerializableBody()
        {

        }

        public SerializableBody(Body body, MovingState movingState, CalibrationState calibrationState, RaisedHand raisedHand, bool gestureDetected)
        {
            if (body != null)
            {
                this.IsTracked = body.IsTracked;
                this.Joints = body.Joints;
                this.MovingState = movingState;
                this.CalibrationState = calibrationState;
                this.RaisedHand = raisedHand;
                this.GestureDetected = gestureDetected;
                this.TrackingID = body.TrackingId;
            }
        }

        private SerializableBody(SerializableBody body)
        {
            this.IsTracked = body.IsTracked;
            this.Joints = body.Joints;
            this.MovingState = body.MovingState;
            this.CalibrationState = body.CalibrationState;
            this.RaisedHand = body.RaisedHand;
            this.GestureDetected = body.GestureDetected;
            this.TrackingID = body.TrackingID;
        }

        public SerializableBody Clone()
        {
            return new SerializableBody(this);
        }
    }
}
