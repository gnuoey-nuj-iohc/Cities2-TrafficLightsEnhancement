using C2VM.TrafficLightsEnhancement.Components;
using C2VM.TrafficLightsEnhancement.Utils;
using Game.Net;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using static C2VM.TrafficLightsEnhancement.Systems.TrafficLightSystems.Initialisation.PatchedTrafficLightInitializationSystem;

namespace C2VM.TrafficLightsEnhancement.Systems.TrafficLightSystems.Initialisation;

public struct CustomPhaseProcessor
{
    public static void ProcessLanes(ref InitializeTrafficLightsJob job, int unfilteredChunkIndex, Entity nodeEntity, DynamicBuffer<ConnectedEdge> connectedEdges, DynamicBuffer<SubLane> subLanes, out int groupCount, ref TrafficLights trafficLights, ref CustomTrafficLights customTrafficLights, DynamicBuffer<EdgeGroupMask> edgeGroupMasks, DynamicBuffer<SubLaneGroupMask> subLaneGroupMasks, DynamicBuffer<CustomPhaseData> customPhaseDatas)
    {
        NativeHashMap<Entity, NodeUtils.LaneConnection> laneConnectionMap = NodeUtils.GetLaneConnectionMap(Allocator.Temp, subLanes, connectedEdges, job.m_ExtraTypeHandle.m_SubLane, job.m_ExtraTypeHandle.m_Lane);
        
        // If CustomPhaseData buffer is empty OR if this is Advanced Split Phasing pattern, initialize with automatic groups
        // For existing traffic lights, we need to reinitialize EdgeGroupMask even if CustomPhaseData exists
        bool isAdvancedSplitPhasing = customTrafficLights.GetPatternOnly() == CustomTrafficLights.Patterns.SplitPhasingAdvancedObsolete;
        if (customPhaseDatas.Length == 0 || isAdvancedSplitPhasing)
        {
            // Validate buffers first to ensure EdgeGroupMask and SubLaneGroupMask are initialized
            CustomPhaseUtils.ValidateBuffer(ref job, nodeEntity, subLanes, connectedEdges, edgeGroupMasks, subLaneGroupMasks, job.m_ExtraTypeHandle.m_SubLane);
            
            // Use SplitPhasing logic to create default groups automatically
            PredefinedPatternsProcessor.SetupSplitPhasing(ref job, connectedEdges, subLanes, out groupCount, ref trafficLights);
            
            // Store original group masks AFTER SetupSplitPhasing (since SetupSplitPhasing modifies them)
            // We need to read them after SetupSplitPhasing has assigned groups
            NativeHashMap<Entity, ushort> originalGroupMasks = new NativeHashMap<Entity, ushort>(subLanes.Length, Allocator.Temp);
            int lanesWithGroupMask = 0;
            for (int i = 0; i < subLanes.Length; i++)
            {
                Entity subLane = subLanes[i].m_SubLane;
                if (job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
                {
                    originalGroupMasks[subLane] = laneSignal.m_GroupMask;
                    if (laneSignal.m_GroupMask != 0)
                    {
                        lanesWithGroupMask++;
                    }
                }
            }
            
            // Debug: Log if we have group masks
            // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index}: After SetupSplitPhasing, {lanesWithGroupMask}/{subLanes.Length} lanes have non-zero group masks");
            
            // SetupSplitPhasing assigns groups sequentially: one group per edge for car lanes, then one group per edge for track lanes
            // Map each edge to its assigned group
            NativeHashMap<Entity, ushort> edgeToGroupMap = new NativeHashMap<Entity, ushort>(connectedEdges.Length, Allocator.Temp);
            int currentGroupIndex = 0;
            
            // First pass: map car lane groups for each edge (same order as SetupSplitPhasing)
            for (int i = 0; i < connectedEdges.Length; i++)
            {
                Entity edge = connectedEdges[i].m_Edge;
                bool hasCarLanes = false;
                
                for (int j = 0; j < subLanes.Length; j++)
                {
                    Entity subLane = subLanes[j].m_SubLane;
                    if (!laneConnectionMap.TryGetValue(subLane, out var laneConnection))
                    {
                        continue;
                    }
                    if (laneConnection.m_SourceEdge != edge)
                    {
                        continue;
                    }
                    if (!job.m_CarLaneData.HasComponent(laneConnection.m_SourceSubLane))
                    {
                        continue;
                    }
                    hasCarLanes = true;
                    break;
                }
                
                if (hasCarLanes)
                {
                    edgeToGroupMap[edge] = (ushort)(1 << currentGroupIndex);
                    currentGroupIndex++;
                }
            }
            
            // Second pass: map track lane groups for edges without car lanes
            for (int i = 0; i < connectedEdges.Length; i++)
            {
                Entity edge = connectedEdges[i].m_Edge;
                if (edgeToGroupMap.ContainsKey(edge))
                {
                    continue; // Already has a group from car lanes
                }
                
                bool hasTrackLanes = false;
                
                for (int j = 0; j < subLanes.Length; j++)
                {
                    Entity subLane = subLanes[j].m_SubLane;
                    if (!laneConnectionMap.TryGetValue(subLane, out var laneConnection))
                    {
                        continue;
                    }
                    if (laneConnection.m_SourceEdge != edge)
                    {
                        continue;
                    }
                    if (!job.m_ExtraTypeHandle.m_TrackLane.HasComponent(subLane))
                    {
                        continue;
                    }
                    if (job.m_CarLaneData.HasComponent(laneConnection.m_SourceSubLane))
                    {
                        continue;
                    }
                    hasTrackLanes = true;
                    break;
                }
                
                if (hasTrackLanes)
                {
                    edgeToGroupMap[edge] = (ushort)(1 << currentGroupIndex);
                    currentGroupIndex++;
                }
            }
            
            // Update EdgeGroupMask buffers with the groups assigned by SetupSplitPhasing
            // First, collect the actual groups assigned to each edge from originalGroupMasks
            NativeHashMap<Entity, ushort> edgeActualGroupMap = new NativeHashMap<Entity, ushort>(connectedEdges.Length, Allocator.Temp);
            int lanesWithGroups = 0;
            int edgesWithGroups = 0;
            
            // If originalGroupMasks is empty or has no valid groups, we need to use edgeToGroupMap
            bool useEdgeToGroupMap = false;
            if (lanesWithGroupMask == 0)
            {
                // SetupSplitPhasing didn't assign groups properly, use edgeToGroupMap instead
                useEdgeToGroupMap = true;
                // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index}: WARNING - No lanes have group masks after SetupSplitPhasing, using edgeToGroupMap");
            }
            else
            {
                for (int i = 0; i < subLanes.Length; i++)
                {
                    Entity subLane = subLanes[i].m_SubLane;
                    if (!originalGroupMasks.TryGetValue(subLane, out ushort originalMask))
                    {
                        continue;
                    }
                    if (originalMask == 0)
                    {
                        continue;
                    }
                    if (!laneConnectionMap.TryGetValue(subLane, out var laneConnection))
                    {
                        continue;
                    }
                    Entity sourceEdge = laneConnection.m_SourceEdge;
                    if (sourceEdge == Entity.Null)
                    {
                        continue;
                    }
                    
                    lanesWithGroups++;
                    
                    // Find the primary group (lowest bit set) for this lane
                    ushort primaryGroup = (ushort)(originalMask & (~(originalMask - 1)));
                    
                    // If this edge doesn't have a group yet, or if this is a lower group, use it
                    if (!edgeActualGroupMap.TryGetValue(sourceEdge, out ushort existingGroup))
                    {
                        edgeActualGroupMap[sourceEdge] = primaryGroup;
                        edgesWithGroups++;
                    }
                    else if (primaryGroup < existingGroup)
                    {
                        edgeActualGroupMap[sourceEdge] = primaryGroup;
                    }
                }
            }
            
            // Debug: Log if we found groups
            // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index}: Found {lanesWithGroups} lanes with groups, {edgesWithGroups} edges with groups, groupCount={groupCount}, useEdgeToGroupMap={useEdgeToGroupMap}");
            
            for (int edgeIndex = 0; edgeIndex < connectedEdges.Length; edgeIndex++)
            {
                Entity edge = connectedEdges[edgeIndex].m_Edge;
                float3 edgePosition = NodeUtils.GetEdgePosition(ref job, nodeEntity, edge);
                int groupMaskIndex = CustomPhaseUtils.TryGet(edgeGroupMasks, edge, edgePosition, out EdgeGroupMask groupMask);
                
                if (groupMaskIndex < 0)
                {
                    // Add new EdgeGroupMask if it doesn't exist
                    groupMask = new EdgeGroupMask(edge, edgePosition);
                    edgeGroupMasks.Add(groupMask);
                    groupMaskIndex = edgeGroupMasks.Length - 1;
                }
                
                // Get the group assigned to this edge by SetupSplitPhasing
                // Priority: 1) edgeActualGroupMap (from originalGroupMasks), 2) edgeToGroupMap, 3) fallback
                ushort edgeGroup = 0;
                if (!useEdgeToGroupMap && edgeActualGroupMap.TryGetValue(edge, out ushort actualGroup))
                {
                    edgeGroup = actualGroup;
                }
                else if (edgeToGroupMap.TryGetValue(edge, out ushort assignedGroup))
                {
                    edgeGroup = assignedGroup;
                }
                else
                {
                    // Fallback: find any group from subLanes connected to this edge
                    for (int j = 0; j < subLanes.Length; j++)
                    {
                        Entity subLane = subLanes[j].m_SubLane;
                        if (!laneConnectionMap.TryGetValue(subLane, out var laneConnection))
                        {
                            continue;
                        }
                        if (laneConnection.m_SourceEdge != edge)
                        {
                            continue;
                        }
                        if (originalGroupMasks.TryGetValue(subLane, out ushort mask) && mask != 0)
                        {
                            edgeGroup = (ushort)(mask & (~(mask - 1)));
                            break;
                        }
                    }
                    
                    // If still no group found, use default based on edge index (but ensure it's within groupCount)
                    if (edgeGroup == 0 && edgeIndex < groupCount)
                    {
                        edgeGroup = (ushort)(1 << edgeIndex);
                    }
                    else if (edgeGroup == 0 && groupCount > 0)
                    {
                        // Last resort: use the first group
                        edgeGroup = 1;
                    }
                }
                
                // Ensure edgeGroup is valid (non-zero and within groupCount range)
                if (edgeGroup == 0 || edgeGroup >= (1 << groupCount))
                {
                    // Use first group as fallback
                    edgeGroup = 1;
                    // Debug: Log fallback usage
                    // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index} Edge {edgeIndex}: Using fallback group 1 (was {edgeGroup})");
                }
                
                // Debug: Log edge group assignment
                // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index} Edge {edgeIndex}: Assigned group {edgeGroup} (groupCount={groupCount})");
                
                // Set all turn directions to use the same group mask (for Advanced Split Phasing)
                // This ensures signals are displayed correctly
                groupMask.m_Car.m_Left.m_GoGroupMask = edgeGroup;
                groupMask.m_Car.m_Right.m_GoGroupMask = edgeGroup;
                groupMask.m_Car.m_UTurn.m_GoGroupMask = edgeGroup;
                groupMask.m_Car.m_Straight.m_GoGroupMask = edgeGroup;
                
                groupMask.m_PublicCar.m_Left.m_GoGroupMask = edgeGroup;
                groupMask.m_PublicCar.m_Right.m_GoGroupMask = edgeGroup;
                groupMask.m_PublicCar.m_UTurn.m_GoGroupMask = edgeGroup;
                groupMask.m_PublicCar.m_Straight.m_GoGroupMask = edgeGroup;
                
                groupMask.m_Track.m_Left.m_GoGroupMask = edgeGroup;
                groupMask.m_Track.m_Right.m_GoGroupMask = edgeGroup;
                groupMask.m_Track.m_Straight.m_GoGroupMask = edgeGroup;
                
                // Pedestrian signals use all groups except the one used by this edge
                ushort pedestrianGroupMask = (ushort)(((1 << groupCount) - 1) & ~edgeGroup);
                groupMask.m_PedestrianStopLine.m_GoGroupMask = pedestrianGroupMask;
                groupMask.m_PedestrianNonStopLine.m_GoGroupMask = pedestrianGroupMask;
                
                edgeGroupMasks[groupMaskIndex] = groupMask;
            }
            
            // Debug: Log summary
            // System.Console.WriteLine($"[CustomPhaseProcessor] Node {nodeEntity.Index}: Processed {connectedEdges.Length} edges, groupCount={groupCount}");
            
            // For Advanced Split Phasing, remove permissive left turns
            // This prevents vehicles from turning left when oncoming traffic has green light
            // Each lane should only use its own group, not all groups
            // Do this AFTER EdgeGroupMask is set up so signals are displayed correctly
            // This applies to both car lanes and track lanes (trams)
            for (int i = 0; i < subLanes.Length; i++)
            {
                Entity subLane = subLanes[i].m_SubLane;
                if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
                {
                    continue;
                }
                
                // Check if this is a car lane or track lane
                bool isCarLane = job.m_CarLaneData.TryGetComponent(subLane, out var carLane);
                bool isTrackLane = job.m_ExtraTypeHandle.m_TrackLane.HasComponent(subLane);
                
                // Skip if neither car lane nor track lane
                if (!isCarLane && !isTrackLane)
                {
                    continue;
                }
                
                // Skip U-turn lanes (only applies to car lanes)
                if (isCarLane && (carLane.m_Flags & (CarLaneFlags.UTurnLeft | CarLaneFlags.UTurnRight)) != 0)
                {
                    continue;
                }
                
                // Find the primary group for this lane from the original group mask
                // This ensures each lane (car, track, bicycle) only gets green when its own group is active
                ushort primaryGroup = 0;
                if (originalGroupMasks.TryGetValue(subLane, out ushort originalMask) && originalMask != 0)
                {
                    primaryGroup = (ushort)(originalMask & (~(originalMask - 1)));
                }
                
                // Restrict to only the primary group to prevent permissive turns
                // This ensures each lane only gets green when its own group is active
                // Works for car lanes, track lanes (trams), and bicycle lanes (treated as car lanes)
                laneSignal.m_GroupMask = primaryGroup;
                
                job.m_LaneSignalData[subLane] = laneSignal;
            }
            
            // Update master lanes after restricting group masks
            for (int i = 0; i < subLanes.Length; i++)
            {
                Entity subLane = subLanes[i].m_SubLane;
                if (!job.m_MasterLaneData.TryGetComponent(subLane, out MasterLane masterLane))
                {
                    continue;
                }
                if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
                {
                    continue;
                }

                laneSignal.m_GroupMask = 0;
                for (int j = masterLane.m_MinIndex; j <= masterLane.m_MaxIndex; j++)
                {
                    Entity slaveSubLane = subLanes[j].m_SubLane;
                    if (!job.m_LaneSignalData.TryGetComponent(slaveSubLane, out LaneSignal slaveLaneSignal))
                    {
                        continue;
                    }
                    laneSignal.m_GroupMask |= slaveLaneSignal.m_GroupMask;
                }

                job.m_LaneSignalData[subLane] = laneSignal;
            }
            
            // Create default CustomPhaseData for each group with adaptive settings
            // If CustomPhaseData already exists (for existing traffic lights), clear and recreate it
            if (isAdvancedSplitPhasing && customPhaseDatas.Length > 0)
            {
                customPhaseDatas.Clear();
            }
            
            for (int i = 0; i < groupCount; i++)
            {
                CustomPhaseData defaultPhase = new CustomPhaseData();
                // Default adaptive settings
                defaultPhase.m_Options = Components.CustomPhaseData.Options.PrioritiseTrack;
                defaultPhase.m_MinimumDuration = 2;
                defaultPhase.m_MaximumDuration = 300;
                defaultPhase.m_TargetDurationMultiplier = 1f;
                defaultPhase.m_LaneOccupiedMultiplier = 1f;
                defaultPhase.m_IntervalExponent = 2f;
                customPhaseDatas.Add(defaultPhase);
            }
            
            // For Advanced Split Phasing, always enable "Allow Turning on Red" for kerbside turns
            // This improves traffic flow by allowing right turns (or left turns in LHT) on red when safe
            if (isAdvancedSplitPhasing)
            {
                // Calculate pedestrian group mask (all groups that have pedestrian signals)
                // Pedestrian groups should not allow turning on red for safety
                ushort pedestrianGroupMask = 0;
                for (int i = 0; i < subLanes.Length; i++)
                {
                    Entity subLane = subLanes[i].m_SubLane;
                    if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
                    {
                        continue;
                    }
                    if (job.m_PedestrianLaneData.HasComponent(subLane))
                    {
                        pedestrianGroupMask |= laneSignal.m_GroupMask;
                    }
                }
                
                // Apply "Allow Turning on Red" to kerbside turn lanes
                // Kerbside turns are safer as they don't cross oncoming traffic
                for (int i = 0; i < subLanes.Length; i++)
                {
                    Entity subLane = subLanes[i].m_SubLane;
                    if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
                    {
                        continue;
                    }
                    if (!job.m_CarLaneData.TryGetComponent(subLane, out var carLane))
                    {
                        continue;
                    }
                    
                    // Check if this is a kerbside turn lane (left turn in LHT, right turn in RHT)
                    bool isKerbsideTurn = false;
                    if (job.m_LeftHandTraffic && (carLane.m_Flags & (CarLaneFlags.TurnLeft | CarLaneFlags.GentleTurnLeft)) != 0)
                    {
                        isKerbsideTurn = true;
                    }
                    else if (!job.m_LeftHandTraffic && (carLane.m_Flags & (CarLaneFlags.TurnRight | CarLaneFlags.GentleTurnRight)) != 0)
                    {
                        isKerbsideTurn = true;
                    }
                    
                    if (!isKerbsideTurn)
                    {
                        continue;
                    }
                    
                    // Calculate which groups allow turning on red
                    // Allow turning on red when:
                    // 1. Another group (not pedestrian) is active
                    // 2. Not the same group as this lane (to avoid conflicts)
                    ushort allGroupsMask = (ushort)((1 << groupCount) - 1);
                    ushort allowTurningOnRedGroupMask = (ushort)(allGroupsMask & ~pedestrianGroupMask & ~laneSignal.m_GroupMask);
                    
                    // Only apply if there are other groups to allow turning on red
                    if (allowTurningOnRedGroupMask == 0)
                    {
                        continue;
                    }
                    
                    // Get or create ExtraLaneSignal
                    ExtraLaneSignal extraLaneSignal = new ExtraLaneSignal();
                    if (job.m_ExtraTypeHandle.m_ExtraLaneSignal.HasComponent(subLane))
                    {
                        extraLaneSignal = job.m_ExtraTypeHandle.m_ExtraLaneSignal[subLane];
                    }
                    else
                    {
                        // Set source subLane for new ExtraLaneSignal
                        extraLaneSignal.m_SourceSubLane = subLane;
                    }
                    
                    // Set YieldGroupMask to allow turning on red when other groups are active
                    // This enables yield signal (turn on red) when another group has green
                    extraLaneSignal.m_YieldGroupMask = allowTurningOnRedGroupMask;
                    extraLaneSignal.m_IgnorePriorityGroupMask = allowTurningOnRedGroupMask;
                    
                    // Update the ExtraLaneSignal component
                    if (job.m_ExtraTypeHandle.m_ExtraLaneSignal.HasComponent(subLane))
                    {
                        job.m_CommandBuffer.SetComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
                    }
                    else
                    {
                        job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
                    }
                }
            }
            
            // SetupSplitPhasing already configured all signals, and EdgeGroupMask is now synchronized
            return;
        }
        
        groupCount = customPhaseDatas.Length;

        for (int i = 0; i < subLanes.Length; i++)
        {
            Entity subLane = subLanes[i].m_SubLane;
            if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
            {
                continue;
            }
            laneSignal.m_GroupMask = 0;
            job.m_LaneSignalData[subLane] = laneSignal;
        }

        for (int i = 0; i < subLanes.Length; i++)
        {
            Entity subLane = subLanes[i].m_SubLane;
            bool isPedestrian = job.m_PedestrianLaneData.TryGetComponent(subLane, out var pedestrianLane);
            if (!job.m_LaneSignalData.HasComponent(subLane) && (pedestrianLane.m_Flags & PedestrianLaneFlags.Crosswalk) == 0)
            {
                continue;
            }
            if ((pedestrianLane.m_Flags & (PedestrianLaneFlags.Crosswalk | PedestrianLaneFlags.Unsafe)) == (PedestrianLaneFlags.Crosswalk | PedestrianLaneFlags.Unsafe))
            {
                continue;
            }
            if (job.m_MasterLaneData.HasComponent(subLane))
            {
                continue;
            }
            var laneConnection = NodeUtils.GetLaneConnectionFromNodeSubLane(subLane, laneConnectionMap, (pedestrianLane.m_Flags & PedestrianLaneFlags.Crosswalk) != 0);
            var sourceEdge = laneConnection.m_SourceEdge == Entity.Null && isPedestrian ? laneConnection.m_DestEdge : laneConnection.m_SourceEdge;
            var edgePosition = NodeUtils.GetEdgePosition(ref job, nodeEntity, sourceEdge);
            LaneSignal laneSignal = new LaneSignal();
            if (job.m_LaneSignalData.HasComponent(subLane))
            {
                laneSignal = job.m_LaneSignalData[subLane];
            }
            laneSignal.m_GroupMask = ushort.MaxValue;
            laneSignal.m_Default = 0;
            ExtraLaneSignal extraLaneSignal = new ExtraLaneSignal();
            extraLaneSignal.m_SourceSubLane = laneConnection.m_SourceSubLane;
            if (CustomPhaseUtils.TryGet(edgeGroupMasks, sourceEdge, edgePosition, out EdgeGroupMask groupMask) >= 0)
            {
                if ((groupMask.m_Options & EdgeGroupMask.Options.PerLaneSignal) != 0)
                {
                    Entity searchKey = isPedestrian ? subLane : laneConnection.m_SourceSubLane;
                    float3 subLanePosition = NodeUtils.GetSubLanePosition(searchKey, job.m_CurveData);
                    CustomPhaseUtils.TryGet(subLaneGroupMasks, searchKey, subLanePosition, out SubLaneGroupMask subLaneGroupMask);
                    groupMask.m_Car = subLaneGroupMask.m_Car;
                    groupMask.m_PublicCar = subLaneGroupMask.m_Car;
                    groupMask.m_Track = subLaneGroupMask.m_Track;
                    groupMask.m_PedestrianStopLine = subLaneGroupMask.m_Pedestrian;
                    groupMask.m_PedestrianNonStopLine = subLaneGroupMask.m_Pedestrian;
                }
                if (job.m_CarLaneData.TryGetComponent(subLane, out var nodeCarLane))
                {
                    job.m_CarLaneData.TryGetComponent(laneConnection.m_SourceSubLane, out var edgeCarLane);
                    var turn = (edgeCarLane.m_Flags & CarLaneFlags.PublicOnly) != 0 ? groupMask.m_PublicCar : groupMask.m_Car;
                    if ((nodeCarLane.m_Flags & (CarLaneFlags.TurnLeft | CarLaneFlags.GentleTurnLeft)) != 0)
                    {
                        laneSignal.m_GroupMask = turn.m_Left.m_GoGroupMask;
                        extraLaneSignal.m_YieldGroupMask = turn.m_Left.m_YieldGroupMask;
                        extraLaneSignal.m_IgnorePriorityGroupMask = turn.m_Left.m_YieldGroupMask;
                    }
                    else if ((nodeCarLane.m_Flags & (CarLaneFlags.TurnRight | CarLaneFlags.GentleTurnRight)) != 0)
                    {
                        laneSignal.m_GroupMask = turn.m_Right.m_GoGroupMask;
                        extraLaneSignal.m_YieldGroupMask = turn.m_Right.m_YieldGroupMask;
                        extraLaneSignal.m_IgnorePriorityGroupMask = turn.m_Right.m_YieldGroupMask;
                    }
                    else
                    {
                        laneSignal.m_GroupMask = turn.m_Straight.m_GoGroupMask;
                        extraLaneSignal.m_YieldGroupMask = turn.m_Straight.m_YieldGroupMask;
                        extraLaneSignal.m_IgnorePriorityGroupMask = turn.m_Straight.m_YieldGroupMask;
                    }
                    if ((nodeCarLane.m_Flags & (CarLaneFlags.UTurnLeft | CarLaneFlags.UTurnRight)) != 0)
                    {
                        laneSignal.m_GroupMask = turn.m_UTurn.m_GoGroupMask;
                        extraLaneSignal.m_YieldGroupMask = turn.m_UTurn.m_YieldGroupMask;
                        extraLaneSignal.m_IgnorePriorityGroupMask = turn.m_UTurn.m_YieldGroupMask;
                    }
                    laneSignal.m_Flags |= LaneSignalFlags.CanExtend;
                }
                if (job.m_ExtraTypeHandle.m_TrackLane.TryGetComponent(subLane, out var trackLane))
                {
                    if ((trackLane.m_Flags & TrackLaneFlags.TurnLeft) != 0)
                    {
                        laneSignal.m_GroupMask = groupMask.m_Track.m_Left.m_GoGroupMask;
                    }
                    else if ((trackLane.m_Flags & TrackLaneFlags.TurnRight) != 0)
                    {
                        laneSignal.m_GroupMask = groupMask.m_Track.m_Right.m_GoGroupMask;
                    }
                    else
                    {
                        laneSignal.m_GroupMask = groupMask.m_Track.m_Straight.m_GoGroupMask;
                    }
                }
                if ((pedestrianLane.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
                {
                    if (NodeUtils.IsCrossingStopLine(ref job, subLane, sourceEdge))
                    {
                        laneSignal.m_GroupMask = groupMask.m_PedestrianStopLine.m_GoGroupMask;
                    }
                    else
                    {
                        laneSignal.m_GroupMask = groupMask.m_PedestrianNonStopLine.m_GoGroupMask;
                    }
                }
            }

            Simulation.PatchedTrafficLightSystem.UpdateLaneSignal(trafficLights, ref laneSignal, ref extraLaneSignal);
            if (job.m_LaneSignalData.HasComponent(subLane))
            {
                job.m_LaneSignalData[subLane] = laneSignal;
            }
            else
            {
                job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLane, laneSignal);
            }
            if (job.m_ExtraTypeHandle.m_ExtraLaneSignal.HasComponent(subLane))
            {
                job.m_CommandBuffer.SetComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
            }
            else
            {
                job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
            }
        }

        for (int i = 0; i < subLanes.Length; i++)
        {
            Entity subLane = subLanes[i].m_SubLane;
            if (!job.m_MasterLaneData.TryGetComponent(subLane, out MasterLane masterLane))
            {
                continue;
            }
            if (!job.m_LaneSignalData.TryGetComponent(subLane, out LaneSignal laneSignal))
            {
                continue;
            }

            laneSignal.m_GroupMask = 0;
            for (int j = masterLane.m_MinIndex; j <= masterLane.m_MaxIndex; j++)
            {
                Entity slaveSubLane = subLanes[j].m_SubLane;
                if (!job.m_LaneSignalData.TryGetComponent(slaveSubLane, out LaneSignal slaveLaneSignal))
                {
                    continue;
                }
                laneSignal.m_GroupMask |= slaveLaneSignal.m_GroupMask;
            }

            ExtraLaneSignal extraLaneSignal = new();
            Simulation.PatchedTrafficLightSystem.UpdateLaneSignal(trafficLights, ref laneSignal, ref extraLaneSignal);
            job.m_LaneSignalData[subLane] = laneSignal;
        }

        // Set up pedestrian crossings at tracks
        for (int i = 0; i < subLanes.Length; i++)
        {
            Entity subLane = subLanes[i].m_SubLane;
            bool isPedestrian = job.m_PedestrianLaneData.TryGetComponent(subLane, out var pedestrianLane);
            if (!isPedestrian)
            {
                continue;
            }
            if ((pedestrianLane.m_Flags & (PedestrianLaneFlags.Crosswalk | PedestrianLaneFlags.Unsafe)) == (PedestrianLaneFlags.Crosswalk | PedestrianLaneFlags.Unsafe))
            {
                continue;
            }
            if (job.m_MasterLaneData.HasComponent(subLane))
            {
                continue;
            }
            var laneConnection = NodeUtils.GetLaneConnectionFromNodeSubLane(subLane, laneConnectionMap, true);
            var sourceEdge = laneConnection.m_SourceEdge == Entity.Null ? laneConnection.m_DestEdge : laneConnection.m_SourceEdge;
            var edgePosition = NodeUtils.GetEdgePosition(ref job, nodeEntity, sourceEdge);
            if (CustomPhaseUtils.TryGet(edgeGroupMasks, sourceEdge, edgePosition, out EdgeGroupMask groupMask) >= 0)
            {
                if ((groupMask.m_Options & EdgeGroupMask.Options.PerLaneSignal) != 0)
                {
                    continue;
                }
            }
            LaneSignal laneSignal = new LaneSignal();
            if (job.m_LaneSignalData.HasComponent(subLane))
            {
                laneSignal = job.m_LaneSignalData[subLane];
            }
            ExtraLaneSignal extraLaneSignal = new ExtraLaneSignal();
            laneSignal.m_GroupMask = ushort.MaxValue;
            laneSignal.m_Default = 0;
            if (job.m_Overlaps.HasBuffer(subLane))
            {
                bool hasCarLane = false;
                foreach (var overlap in job.m_Overlaps[subLane])
                {
                    if (job.m_CarLaneData.HasComponent(overlap.m_Other))
                    {
                        hasCarLane = true;
                        break;
                    }
                    if (!job.m_ExtraTypeHandle.m_TrackLane.HasComponent(overlap.m_Other))
                    {
                        continue;
                    }
                    if (job.m_LaneSignalData.TryGetComponent(overlap.m_Other, out var overlapSignal))
                    {
                        laneSignal.m_GroupMask &= (ushort)~overlapSignal.m_GroupMask;
                    }
                }
                if (hasCarLane)
                {
                    continue;
                }
            }

            Simulation.PatchedTrafficLightSystem.UpdateLaneSignal(trafficLights, ref laneSignal, ref extraLaneSignal);
            if (!job.m_LaneSignalData.HasComponent(subLane))
            {
                job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLane, laneSignal);
            }
            if (!job.m_ExtraTypeHandle.m_ExtraLaneSignal.HasComponent(subLane))
            {
                job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
            }
            job.m_CommandBuffer.SetComponent(unfilteredChunkIndex, subLane, laneSignal);
            job.m_CommandBuffer.SetComponent(unfilteredChunkIndex, subLane, extraLaneSignal);
        }
    }
}