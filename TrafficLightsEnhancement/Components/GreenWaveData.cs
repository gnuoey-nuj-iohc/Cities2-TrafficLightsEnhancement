using Colossal.Serialization.Entities;
using Unity.Entities;
using Unity.Collections;

namespace C2VM.TrafficLightsEnhancement.Components
{
    /// <summary>
    /// Green Wave coordination data for a traffic light intersection.
    /// Stores information about adjacent intersections for green wave synchronization.
    /// </summary>
    public struct GreenWaveData : IComponentData, IQueryTypeParameter, ISerializable
    {
        private ushort m_SchemaVersion;

        /// <summary>
        /// Whether green wave coordination is enabled for this intersection.
        /// </summary>
        public bool m_Enabled;

        /// <summary>
        /// Maximum distance to consider adjacent intersections (in meters).
        /// Default: 200 meters
        /// </summary>
        public float m_MaxDistance;

        /// <summary>
        /// Average vehicle speed for delay calculation (in m/s).
        /// Default: 13.89 m/s (50 km/h)
        /// </summary>
        public float m_AverageSpeed;

        public void Serialize<TWriter>(TWriter writer) where TWriter : IWriter
        {
            m_SchemaVersion = 1;
            writer.Write(m_SchemaVersion);
            writer.Write(m_Enabled);
            writer.Write(m_MaxDistance);
            writer.Write(m_AverageSpeed);
        }

        public void Deserialize<TReader>(TReader reader) where TReader : IReader
        {
            reader.Read(out m_SchemaVersion);
            if (m_SchemaVersion >= 1)
            {
                reader.Read(out m_Enabled);
                reader.Read(out m_MaxDistance);
                reader.Read(out m_AverageSpeed);
            }
            else
            {
                m_Enabled = false;
                m_MaxDistance = 200f;
                m_AverageSpeed = 13.89f; // 50 km/h
            }
        }

        public GreenWaveData()
        {
            m_SchemaVersion = 1;
            m_Enabled = false;
            m_MaxDistance = 200f;
            m_AverageSpeed = 13.89f; // 50 km/h
        }
    }

    /// <summary>
    /// Stores information about an adjacent intersection for green wave coordination.
    /// This is a dynamic buffer component, allowing multiple adjacent intersections per node.
    /// </summary>
    public struct AdjacentIntersection : IBufferElementData
    {
        /// <summary>
        /// Entity of the adjacent intersection node.
        /// </summary>
        public Entity m_NodeEntity;

        /// <summary>
        /// Distance to the adjacent intersection (in meters).
        /// </summary>
        public float m_Distance;

        /// <summary>
        /// Edge connecting this intersection to the adjacent one.
        /// </summary>
        public Entity m_EdgeEntity;

        /// <summary>
        /// Calculated delay time in traffic light ticks (64 ticks = 1 second).
        /// This is the time it takes for a vehicle to travel from this intersection to the adjacent one.
        /// </summary>
        public int m_DelayTicks;

        public AdjacentIntersection(Entity nodeEntity, float distance, Entity edgeEntity, int delayTicks)
        {
            m_NodeEntity = nodeEntity;
            m_Distance = distance;
            m_EdgeEntity = edgeEntity;
            m_DelayTicks = delayTicks;
        }
    }
}




















