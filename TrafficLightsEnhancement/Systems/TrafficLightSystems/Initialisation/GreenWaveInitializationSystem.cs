using C2VM.TrafficLightsEnhancement.Components;
using Game.Net;
using Game.Simulation;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace C2VM.TrafficLightsEnhancement.Systems.TrafficLightSystems.Initialisation
{
    /// <summary>
    /// System that finds and initializes adjacent intersections for Green Wave coordination.
    /// This system runs periodically to update adjacent intersection information.
    /// </summary>
    [BurstCompile]
    public partial class GreenWaveInitializationSystem : Game.GameSystemBase
    {
        private EntityQuery m_TrafficLightsQuery;

        protected override void OnCreate()
        {
            base.OnCreate();
            
            // Query for all nodes with traffic lights
            m_TrafficLightsQuery = GetEntityQuery(
                ComponentType.ReadWrite<TrafficLights>(),
                ComponentType.ReadOnly<Node>(),
                ComponentType.ReadOnly<ConnectedEdge>()
            );
            
            RequireForUpdate(m_TrafficLightsQuery);
        }

        protected override void OnUpdate()
        {
            // Only update every 5 seconds to avoid performance issues
            if (SystemAPI.Time.ElapsedTime % 5.0 < 0.016) // ~60 FPS check
            {
                return;
            }

            var job = new FindAdjacentIntersectionsJob
            {
                m_EntityType = GetEntityTypeHandle(),
                m_NodeData = GetComponentLookup<Node>(true),
                m_EdgeData = GetComponentLookup<Edge>(true),
                m_CurveData = GetComponentLookup<Curve>(true),
                m_TrafficLightsType = GetComponentTypeHandle<TrafficLights>(true),
                m_ConnectedEdgeType = GetBufferTypeHandle<ConnectedEdge>(true),
                m_GreenWaveData = GetComponentLookup<GreenWaveData>(true),
                m_AdjacentIntersections = GetBufferLookup<AdjacentIntersection>(false),
                m_CommandBuffer = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(World.Unmanaged).AsParallelWriter()
            };

            Dependency = job.ScheduleParallel(m_TrafficLightsQuery, Dependency);
        }

        [BurstCompile]
        private struct FindAdjacentIntersectionsJob : IJobChunk
        {
            [ReadOnly]
            public EntityTypeHandle m_EntityType;

            [ReadOnly]
            public ComponentLookup<Node> m_NodeData;

            [ReadOnly]
            public ComponentLookup<Edge> m_EdgeData;

            [ReadOnly]
            public ComponentLookup<Curve> m_CurveData;

            [ReadOnly]
            public ComponentTypeHandle<TrafficLights> m_TrafficLightsType;

            [ReadOnly]
            public BufferTypeHandle<ConnectedEdge> m_ConnectedEdgeType;

            [ReadOnly]
            public ComponentLookup<GreenWaveData> m_GreenWaveData;

            public BufferLookup<AdjacentIntersection> m_AdjacentIntersections;

            public EntityCommandBuffer.ParallelWriter m_CommandBuffer;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in Unity.Burst.Intrinsics.v128 chunkEnabledMask)
            {
                var nodeEntities = chunk.GetNativeArray(m_EntityType);
                var trafficLightsArray = chunk.GetNativeArray(ref m_TrafficLightsType);
                var connectedEdgesAccessor = chunk.GetBufferAccessor(ref m_ConnectedEdgeType);

                for (int i = 0; i < chunk.Count; i++)
                {
                    Entity currentNode = nodeEntities[i];
                    TrafficLights trafficLights = trafficLightsArray[i];
                    
                    // Skip sub-nodes
                    if ((trafficLights.m_Flags & TrafficLightFlags.IsSubNode) != 0)
                    {
                        continue;
                    }

                    // Check if Green Wave is enabled for this intersection
                    if (!m_GreenWaveData.TryGetComponent(currentNode, out var greenWaveData) || !greenWaveData.m_Enabled)
                    {
                        // Clear adjacent intersections if Green Wave is disabled
                        if (m_AdjacentIntersections.HasBuffer(currentNode))
                        {
                            var buffer = m_AdjacentIntersections[currentNode];
                            buffer.Clear();
                        }
                        continue;
                    }

                    // Get or create AdjacentIntersection buffer
                    DynamicBuffer<AdjacentIntersection> adjacentBuffer;
                    if (!m_AdjacentIntersections.HasBuffer(currentNode))
                    {
                        m_CommandBuffer.AddBuffer<AdjacentIntersection>(unfilteredChunkIndex, currentNode);
                        continue; // Will be processed in next update
                    }
                    else
                    {
                        adjacentBuffer = m_AdjacentIntersections[currentNode];
                    }

                    // Clear existing adjacent intersections
                    adjacentBuffer.Clear();

                    if (!m_NodeData.TryGetComponent(currentNode, out var currentNodeData))
                    {
                        continue;
                    }

                    var connectedEdges = connectedEdgesAccessor[i];
                    NativeHashSet<Entity> foundAdjacentNodes = new NativeHashSet<Entity>(8, Allocator.Temp);

                    // Find adjacent intersections through connected edges
                    for (int edgeIdx = 0; edgeIdx < connectedEdges.Length; edgeIdx++)
                    {
                        Entity edge = connectedEdges[edgeIdx].m_Edge;
                        if (!m_EdgeData.TryGetComponent(edge, out var edgeData))
                        {
                            continue;
                        }

                        // Get the other node connected to this edge
                        Entity otherNode = edgeData.m_Start == currentNode ? edgeData.m_End : edgeData.m_Start;
                        if (otherNode == Entity.Null || otherNode == currentNode)
                        {
                            continue;
                        }

                        // Check if we've already found this adjacent node
                        if (foundAdjacentNodes.Contains(otherNode))
                        {
                            continue;
                        }

                        // Note: We can't check if the other node has traffic lights in IJobChunk
                        // The Green Wave logic in CustomStateMachine will handle missing traffic lights gracefully

                        // Calculate distance
                        if (!m_NodeData.TryGetComponent(otherNode, out var otherNodeData))
                        {
                            continue;
                        }

                        if (!m_CurveData.TryGetComponent(edge, out var curveData))
                        {
                            continue;
                        }

                        // Calculate distance along the curve (approximate)
                        float distance = math.distance(currentNodeData.m_Position, otherNodeData.m_Position);
                        
                        // Use curve length if available for more accurate distance
                        if (curveData.m_Bezier.a.x != 0 || curveData.m_Bezier.a.y != 0 || curveData.m_Bezier.a.z != 0)
                        {
                            // Approximate curve length using control points
                            float3 p0 = currentNodeData.m_Position;
                            float3 p1 = curveData.m_Bezier.b;
                            float3 p2 = curveData.m_Bezier.c;
                            float3 p3 = otherNodeData.m_Position;
                            
                            // Approximate using chord length
                            distance = math.distance(p0, p1) + math.distance(p1, p2) + math.distance(p2, p3);
                        }

                        // Check if within max distance
                        if (distance > greenWaveData.m_MaxDistance)
                        {
                            continue;
                        }

                        // Calculate delay in traffic light ticks (64 ticks = 1 second)
                        // delay = distance / speed * 64
                        float delaySeconds = distance / greenWaveData.m_AverageSpeed;
                        int delayTicks = (int)(delaySeconds * 64f);

                        // Add to adjacent intersections
                        adjacentBuffer.Add(new AdjacentIntersection(otherNode, distance, edge, delayTicks));
                        foundAdjacentNodes.Add(otherNode);
                        
                        // Debug: Log when adjacent intersections are found
                        // Uncomment the line below to enable debug logging
                        // System.Console.WriteLine($"[Green Wave] Node {currentNode.Index} found adjacent intersection at {distance:F1}m (delay: {delayTicks} ticks)");
                    }

                    foundAdjacentNodes.Dispose();
                    
                    // Debug: Log summary
                    // Uncomment the line below to enable debug logging
                    // if (adjacentBuffer.Length > 0) System.Console.WriteLine($"[Green Wave] Node {currentNode.Index} has {adjacentBuffer.Length} adjacent intersection(s)");
                }
            }
        }
    }
}

