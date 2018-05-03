using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ServerApp
{
    public class TopologyEdge
    {
        public string SourceIPAddress { get; set; }

        public string TargetIPAddress { get; set; }

        public Dictionary<Joint, Joint> Matching { get; set; }

        public double[,] TransformMatrixCoefficients { get; set; }

        public TopologyEdge(string SrcIPAddress, string TargetIPAddress, Dictionary<Joint, Joint> Matching, double[,] TransformMatrixCoefficients)
        {
            this.SourceIPAddress = SrcIPAddress;
            this.TargetIPAddress = TargetIPAddress;
            this.Matching = Matching;
            this.TransformMatrixCoefficients = TransformMatrixCoefficients;
        }
    }
}
