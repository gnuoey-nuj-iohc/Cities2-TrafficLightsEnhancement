using C2VM.TrafficLightsEnhancement.Components;
using Game.Net;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace C2VM.TrafficLightsEnhancement.Systems.TrafficLightSystems.Simulation
{
    public struct CustomStateMachine
    {
        public static bool UpdateTrafficLightState(ref TrafficLights trafficLights, ref CustomTrafficLights customTrafficLights, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer)
        {
            return UpdateTrafficLightState(ref trafficLights, ref customTrafficLights, customPhaseDataBuffer, Entity.Null, null);
        }

        public static bool UpdateTrafficLightState(ref TrafficLights trafficLights, ref CustomTrafficLights customTrafficLights, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer, Entity currentNode, PatchedTrafficLightSystem.UpdateTrafficLightsJob? job)
        {
            if (trafficLights.m_State == TrafficLightState.None || trafficLights.m_State == TrafficLightState.Extending || trafficLights.m_State == TrafficLightState.Extended)
            {
                // Safety check: ensure buffer is not empty
                if (customPhaseDataBuffer.Length == 0)
                {
                    // Buffer is empty, cannot proceed - return false to let vanilla system handle it
                    return false;
                }
                
                trafficLights.m_State = TrafficLightState.Beginning;
                trafficLights.m_CurrentSignalGroup = 0;
                trafficLights.m_NextSignalGroup = GetNextSignalGroup(trafficLights.m_CurrentSignalGroup, customPhaseDataBuffer, customTrafficLights, currentNode, job, out _);
                trafficLights.m_Timer = 0;
                customTrafficLights.m_Timer = 0;
                
                // Safety check: if GetNextSignalGroup returned 0, try again with default fallback
                if (trafficLights.m_NextSignalGroup <= 0)
                {
                    trafficLights.m_NextSignalGroup = 1; // Default to first phase as fallback
                }
                return true;
            }
            else if (trafficLights.m_State == TrafficLightState.Beginning)
            {
                // Safety check: ensure buffer is not empty
                if (customPhaseDataBuffer.Length == 0)
                {
                    trafficLights.m_State = TrafficLightState.None;
                    return false; // Let vanilla system handle it
                }
                
                if (trafficLights.m_NextSignalGroup <= 0 || trafficLights.m_NextSignalGroup > customPhaseDataBuffer.Length)
                {
                    // Invalid group, try to get a valid one
                    trafficLights.m_NextSignalGroup = GetNextSignalGroup(trafficLights.m_CurrentSignalGroup, customPhaseDataBuffer, customTrafficLights, currentNode, job, out _);
                    if (trafficLights.m_NextSignalGroup <= 0)
                    {
                        trafficLights.m_State = TrafficLightState.None; // roll a new group
                        return true;
                    }
                }
                trafficLights.m_State = TrafficLightState.Ongoing;
                trafficLights.m_CurrentSignalGroup = trafficLights.m_NextSignalGroup;
                trafficLights.m_NextSignalGroup = 0;
                trafficLights.m_Timer = 0;
                customTrafficLights.m_Timer = 0;
                for (int i = 0; i < customPhaseDataBuffer.Length; i++)
                {
                    CustomPhaseData phase = customPhaseDataBuffer[i];
                    if (trafficLights.m_CurrentSignalGroup == i + 1)
                    {
                        phase.m_TurnsSinceLastRun = 0;
                        phase.m_LowFlowTimer = 0;
                        phase.m_LowPriorityTimer = 0;
                    }
                    else
                    {
                        phase.m_TurnsSinceLastRun++;
                    }
                    phase.m_Options &= ~CustomPhaseData.Options.EndPhasePrematurely;
                    customPhaseDataBuffer[i] = phase;
                }
                return true;
            }
            else if (trafficLights.m_State == TrafficLightState.Ongoing)
            {
                int currentSignalIndex = trafficLights.m_CurrentSignalGroup - 1;
                if (currentSignalIndex < 0 || currentSignalIndex >= customPhaseDataBuffer.Length)
                {
                    trafficLights.m_State = TrafficLightState.None; // roll a new group
                    return true;
                }
                customTrafficLights.m_Timer++;
                CustomPhaseData phase = customPhaseDataBuffer[currentSignalIndex];
                float targetDuration = 10f * (phase.AverageCarFlow() + (float)(phase.m_TrackLaneOccupied * 0.5)) * phase.m_TargetDurationMultiplier;
                bool preferChange = false;
                phase.m_TargetDuration = targetDuration;
                if (customTrafficLights.m_Timer <= phase.m_MinimumDuration)
                {
                    phase.m_LowFlowTimer = 0;
                    phase.m_LowPriorityTimer = 0;
                }
                else if (phase.m_Priority > 0 && phase.m_Priority >= MaxPriority(customPhaseDataBuffer))
                {
                    if (customTrafficLights.m_Timer >= phase.m_MaximumDuration)
                    {
                        preferChange = true;
                    }
                    else if (customTrafficLights.m_Timer <= targetDuration)
                    {
                        phase.m_LowFlowTimer = 0;
                    }
                    else if (phase.m_LowFlowTimer < 3)
                    {
                        phase.m_LowFlowTimer++;
                    }
                    else
                    {
                        preferChange = true;
                    }
                    phase.m_LowPriorityTimer = 0;
                }
                else if (phase.m_Priority < MaxPriority(customPhaseDataBuffer))
                {
                    if (phase.m_LowPriorityTimer >= 1)
                    {
                        preferChange = true;
                    }
                    phase.m_LowPriorityTimer++;
                }
                else
                {
                    preferChange = true;
                }
                if ((phase.m_Options & CustomPhaseData.Options.EndPhasePrematurely) != 0)
                {
                    preferChange = true;
                }
                if (customTrafficLights.m_ManualSignalGroup > 0 && customTrafficLights.m_ManualSignalGroup != trafficLights.m_CurrentSignalGroup)
                {
                    preferChange = true;
                }
                customPhaseDataBuffer[currentSignalIndex] = phase;
                byte nextGroup = GetNextSignalGroup(trafficLights.m_CurrentSignalGroup, customPhaseDataBuffer, customTrafficLights, currentNode, job, out var linked);
                
                // Force change if maximum duration exceeded, even if nextGroup is the same
                // This prevents a single direction from blocking others indefinitely
                // This is critical for Advanced Split Phasing to ensure fair signal rotation
                bool forceChange = false;
                if (customTrafficLights.m_Timer >= phase.m_MaximumDuration)
                {
                    forceChange = true;
                    // If nextGroup is the same, find the next available group
                    // This ensures that even if the current group has highest priority,
                    // it will be forced to change after maximum duration
                    if (nextGroup == trafficLights.m_CurrentSignalGroup)
                    {
                        // Find the next group with highest priority or longest waiting time
                        // Prioritize groups that haven't run recently (higher m_TurnsSinceLastRun)
                        int bestGroup = 0;
                        int bestPriority = -1;
                        float bestWaiting = -1;
                        int bestTurnsSinceLastRun = -1;
                        
                        for (int i = 0; i < customPhaseDataBuffer.Length; i++)
                        {
                            if (i + 1 == trafficLights.m_CurrentSignalGroup)
                            {
                                continue; // Skip current group
                            }
                            CustomPhaseData otherPhase = customPhaseDataBuffer[i];
                            float weightedWaiting = otherPhase.m_WeightedWaiting;
                            
                            // Prioritize groups that haven't run in a while
                            // This ensures fair rotation even when priorities are similar
                            bool isBetter = false;
                            if (otherPhase.m_TurnsSinceLastRun > bestTurnsSinceLastRun)
                            {
                                isBetter = true;
                            }
                            else if (otherPhase.m_TurnsSinceLastRun == bestTurnsSinceLastRun)
                            {
                                if (otherPhase.m_Priority > bestPriority)
                                {
                                    isBetter = true;
                                }
                                else if (otherPhase.m_Priority == bestPriority && weightedWaiting > bestWaiting)
                                {
                                    isBetter = true;
                                }
                            }
                            
                            if (isBetter)
                            {
                                bestGroup = i + 1;
                                bestPriority = otherPhase.m_Priority;
                                bestWaiting = weightedWaiting;
                                bestTurnsSinceLastRun = otherPhase.m_TurnsSinceLastRun;
                            }
                        }
                        
                        if (bestGroup > 0)
                        {
                            nextGroup = (byte)bestGroup;
                        }
                        else if (customPhaseDataBuffer.Length > 1)
                        {
                            // Fallback: cycle to next group to ensure rotation
                            // This guarantees that signals will change even if all groups have same priority
                            nextGroup = (byte)((trafficLights.m_CurrentSignalGroup % customPhaseDataBuffer.Length) + 1);
                        }
                    }
                }
                
                if ((preferChange || forceChange) && nextGroup != trafficLights.m_CurrentSignalGroup)
                {
                    trafficLights.m_State = TrafficLightState.Ending;
                    trafficLights.m_NextSignalGroup = nextGroup;
                    if (linked)
                    {
                        for (int i = trafficLights.m_CurrentSignalGroup; i < trafficLights.m_NextSignalGroup - 1; i++)
                        {
                            CustomPhaseData nextPhase = customPhaseDataBuffer[i];
                            if (nextPhase.m_Priority <= 0)
                            {
                                nextPhase.m_TurnsSinceLastRun = 0;
                                customPhaseDataBuffer[i] = nextPhase;
                            }
                        }
                    }
                    return true;
                }
                return false;
            }
            else if (trafficLights.m_State == TrafficLightState.Ending)
            {
                trafficLights.m_State = TrafficLightState.Changing;
                return true;
            }
            else if (trafficLights.m_State == TrafficLightState.Changing)
            {
                trafficLights.m_State = TrafficLightState.Beginning;
                return true;
            }
            return false;
        }

        public static void CalculateFlow(PatchedTrafficLightSystem.UpdateTrafficLightsJob job, int unfilteredChunkIndex, DynamicBuffer<SubLane> subLaneBuffer, TrafficLights trafficLights, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer)
        {
            // PERFORMANCE: Early exit if no active signal group
            if (trafficLights.m_CurrentSignalGroup == 0 || trafficLights.m_CurrentSignalGroup > customPhaseDataBuffer.Length)
            {
                return;
            }
            
            float4 timeFactors = job.m_ExtraData.m_TimeFactors * 0.125f;
            for (int i = 0; i < customPhaseDataBuffer.Length; i++)
            {
                CustomPhaseData customPhaseData = customPhaseDataBuffer[i];
                customPhaseData.m_CarFlow.z = customPhaseData.m_CarFlow.y;
                customPhaseData.m_CarFlow.y = customPhaseData.m_CarFlow.x;
                customPhaseData.m_CarFlow.x = 0f;
                customPhaseDataBuffer[i] = customPhaseData;
            }
            
            // PERFORMANCE: Only process lanes that belong to current active group
            int currentGroupMask = 1 << (trafficLights.m_CurrentSignalGroup - 1);
            foreach (var subLane in subLaneBuffer)
            {
                Entity subLaneEntity = subLane.m_SubLane;
                float4 newDistance = 0f;
                float4 newDuration = 0f;
                float4 oldDistance = 0f;
                float4 oldDuration = 0f;
                float4 diffDistance = 0f;
                float4 diffDuration = 0f;
                uint newFrame = job.m_ExtraData.m_Frame;
                uint oldFrame = 0;
                uint diffFrame = 0;

                if (!job.m_LaneSignalData.TryGetComponent(subLaneEntity, out var laneSignal))
                {
                    continue;
                }
                // PERFORMANCE: Check group mask early before expensive component lookups
                if ((laneSignal.m_GroupMask & currentGroupMask) == 0)
                {
                    continue;
                }
                
                if (!job.m_ExtraTypeHandle.m_LaneFlow.TryGetComponent(subLaneEntity, out var laneFlow))
                {
                    continue;
                }

                newDistance = math.lerp(laneFlow.m_Distance, laneFlow.m_Next.y, timeFactors);
                newDuration = math.lerp(laneFlow.m_Duration, laneFlow.m_Next.x, timeFactors);

                LaneFlowHistory laneFlowHistory = new LaneFlowHistory();
                if (job.m_ExtraTypeHandle.m_LaneFlowHistory.TryGetComponent(subLaneEntity, out laneFlowHistory))
                {
                    oldDistance = laneFlowHistory.m_Distance;
                    oldDuration = laneFlowHistory.m_Duration;
                    oldFrame = laneFlowHistory.m_Frame;
                }
                else
                {
                    job.m_CommandBuffer.AddComponent(unfilteredChunkIndex, subLaneEntity, laneFlowHistory);
                }

                diffDistance = newDistance - oldDistance;
                diffDuration = newDuration - oldDuration;
                diffFrame = newFrame - oldFrame;

                laneFlowHistory.m_Distance = newDistance;
                laneFlowHistory.m_Duration = newDuration;
                laneFlowHistory.m_Frame = newFrame;

                job.m_CommandBuffer.SetComponent(unfilteredChunkIndex, subLaneEntity, laneFlowHistory);

                int group = trafficLights.m_CurrentSignalGroup - 1;
                if (group < customPhaseDataBuffer.Length && diffFrame > 0)
                {
                    CustomPhaseData customPhaseData = customPhaseDataBuffer[group];
                    float totalDiff = math.abs(Max(diffDistance)) + math.abs(Max(diffDuration));
                    customPhaseData.m_CarFlow.x += totalDiff * (64f / (float)diffFrame); // 64 frames per traffic light tick
                    customPhaseDataBuffer[group] = customPhaseData;
                }
            }
        }

        public static void CalculatePriority(PatchedTrafficLightSystem.UpdateTrafficLightsJob job, DynamicBuffer<SubLane> subLaneBuffer, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer)
        {
            for (int i = 0; i < customPhaseDataBuffer.Length; i++)
            {
                CustomPhaseData customPhaseData = customPhaseDataBuffer[i];
                customPhaseData.m_CarLaneOccupied = 0;
                customPhaseData.m_PublicCarLaneOccupied = 0;
                customPhaseData.m_TrackLaneOccupied = 0;
                customPhaseData.m_PedestrianLaneOccupied = 0;
                customPhaseData.m_Priority = 0;
                customPhaseDataBuffer[i] = customPhaseData;
            }
            foreach (var subLane in subLaneBuffer)
            {
                Entity subLaneEntity = subLane.m_SubLane;

                if (!job.m_LaneSignalData.TryGetComponent(subLaneEntity, out var laneSignal))
                {
                    continue;
                }

                Entity lanePetitioner = laneSignal.m_Petitioner;
                int lanePriority = laneSignal.m_Priority;

                laneSignal.m_Petitioner = Entity.Null;
                laneSignal.m_Priority = laneSignal.m_Default;
                job.m_LaneSignalData[subLaneEntity] = laneSignal;

                if (job.m_ExtraTypeHandle.m_MasterLane.HasComponent(subLaneEntity))
                {
                    continue;
                }
                if (lanePetitioner == Entity.Null)
                {
                    continue;
                }

                for (int i = 0; i < customPhaseDataBuffer.Length; i++)
                {
                    if ((laneSignal.m_GroupMask & (1 << i)) == 0)
                    {
                        continue;
                    }

                    CustomPhaseData customPhaseData = customPhaseDataBuffer[i];

                    if (job.m_ExtraTypeHandle.m_CarLane.HasComponent(subLaneEntity))
                    {
                        customPhaseData.m_CarLaneOccupied++;
                        if (job.m_ExtraTypeHandle.m_ExtraLaneSignal.TryGetComponent(subLaneEntity, out var extraLaneSignal))
                        {
                            if (extraLaneSignal.m_SourceSubLane != Entity.Null && job.m_ExtraTypeHandle.m_CarLane.TryGetComponent(extraLaneSignal.m_SourceSubLane, out var sourceCarLane))
                            {
                                if ((sourceCarLane.m_Flags & CarLaneFlags.PublicOnly) != 0)
                                {
                                    customPhaseData.m_PublicCarLaneOccupied++;
                                    if ((customPhaseData.m_Options & CustomPhaseData.Options.PrioritisePublicCar) != 0)
                                    {
                                        lanePriority = math.max(lanePriority, 104); // 104 is the priority for trams
                                    }
                                    else
                                    {
                                        lanePriority = math.min(lanePriority, 100); // 100 is the default priority
                                    }
                                }
                            }
                        }
                    }
                    if (job.m_ExtraTypeHandle.m_TrackLane.HasComponent(subLaneEntity))
                    {
                        customPhaseData.m_TrackLaneOccupied++;
                        if ((customPhaseData.m_Options & CustomPhaseData.Options.PrioritiseTrack) == 0)
                        {
                            // Do not lower priority for trains, as they do not stop for signals
                            // 110 is the priority for trains
                            if (lanePriority < 110)
                            {
                                lanePriority = math.min(lanePriority, 100); // 100 is the default priority
                            }
                        }
                    }
                    if (job.m_ExtraTypeHandle.m_PedestrianLane.TryGetComponent(subLaneEntity, out var pedestrianLane))
                    {
                        if ((pedestrianLane.m_Flags & PedestrianLaneFlags.Crosswalk) != 0)
                        {
                            customPhaseData.m_PedestrianLaneOccupied++;
                            if ((customPhaseData.m_Options & CustomPhaseData.Options.PrioritisePedestrian) != 0)
                            {
                                lanePriority = math.max(lanePriority, 104); // 104 is the priority for trams
                            }
                        }
                    }

                    customPhaseData.m_Priority = math.max(customPhaseData.m_Priority, lanePriority);

                    customPhaseDataBuffer[i] = customPhaseData;
                }
            }
        }

        public static byte GetNextSignalGroup(byte currentGroup, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer, CustomTrafficLights customTrafficLights, out bool linked)
        {
            return GetNextSignalGroup(currentGroup, customPhaseDataBuffer, customTrafficLights, Entity.Null, null, out linked);
        }

        public static byte GetNextSignalGroup(byte currentGroup, DynamicBuffer<CustomPhaseData> customPhaseDataBuffer, CustomTrafficLights customTrafficLights, Entity currentNode, PatchedTrafficLightSystem.UpdateTrafficLightsJob? job, out bool linked)
        {
            linked = false;
            byte nextGroup = 0;
            int maxPriority = -1;
            float maxWaiting = -1;
            
            // Safety check: if buffer is empty, return 0 (this should be handled by caller)
            if (customPhaseDataBuffer.Length == 0)
            {
                return 0;
            }
            
            if (customTrafficLights.m_ManualSignalGroup > 0 && customTrafficLights.m_ManualSignalGroup - 1 < customPhaseDataBuffer.Length)
            {
                return customTrafficLights.m_ManualSignalGroup;
            }
            
            // Green Wave coordination: check adjacent intersections
            // PERFORMANCE: Skip green wave calculation if no manual override and buffer is small
            // Green wave calculation can be expensive, so skip it when not needed
            NativeHashMap<byte, float> greenWaveBonuses = default;
            bool hasGreenWaveData = false;
            int greenWaveActiveCount = 0;
            
            // PERFORMANCE OPTIMIZATION: Only check green wave if there are multiple phases
            // Single phase intersections don't benefit from green wave coordination
            bool shouldCheckGreenWave = customPhaseDataBuffer.Length > 1 && job.HasValue && currentNode != Entity.Null;
            
            if (shouldCheckGreenWave)
            {
                var jobValue = job.Value;
                if (jobValue.m_ExtraTypeHandle.m_GreenWaveData.TryGetComponent(currentNode, out var greenWaveData) && greenWaveData.m_Enabled)
                {
                    if (jobValue.m_ExtraTypeHandle.m_AdjacentIntersections.HasBuffer(currentNode))
                    {
                        var adjacentIntersections = jobValue.m_ExtraTypeHandle.m_AdjacentIntersections[currentNode];
                        // Only allocate if we have adjacent intersections
                        if (adjacentIntersections.Length > 0)
                        {
                            greenWaveBonuses = new NativeHashMap<byte, float>(customPhaseDataBuffer.Length, Allocator.Temp);
                            hasGreenWaveData = true;
                            
                            for (int adjIdx = 0; adjIdx < adjacentIntersections.Length; adjIdx++)
                            {
                                var adjacent = adjacentIntersections[adjIdx];
                                if (adjacent.m_NodeEntity == Entity.Null)
                                {
                                    continue;
                                }
                                
                                // Check if adjacent intersection has traffic lights
                                if (!jobValue.m_ExtraTypeHandle.m_TrafficLights.TryGetComponent(adjacent.m_NodeEntity, out var adjacentTrafficLights))
                                {
                                    continue;
                                }
                                
                                // Check if adjacent intersection is in a state that would benefit from coordination
                                // If adjacent is about to turn green (in Beginning state or just started), give bonus to matching phase
                                if (adjacentTrafficLights.m_State == TrafficLightState.Beginning || 
                                    (adjacentTrafficLights.m_State == TrafficLightState.Ongoing && adjacentTrafficLights.m_Timer < adjacent.m_DelayTicks))
                                {
                                    byte adjacentGroup = adjacentTrafficLights.m_CurrentSignalGroup;
                                    if (adjacentGroup > 0 && adjacentGroup <= customPhaseDataBuffer.Length)
                                    {
                                        // Calculate bonus based on how close the timing is
                                        float timingBonus = 1.0f;
                                        if (adjacentTrafficLights.m_State == TrafficLightState.Ongoing)
                                        {
                                            // The closer we are to the delay time, the higher the bonus
                                            float timeDiff = math.abs(adjacentTrafficLights.m_Timer - adjacent.m_DelayTicks);
                                            timingBonus = math.max(0.1f, 1.0f - (timeDiff / (float)adjacent.m_DelayTicks));
                                        }
                                        
                                        // Add bonus to the matching phase group
                                        if (greenWaveBonuses.TryGetValue(adjacentGroup, out float existingBonus))
                                        {
                                            greenWaveBonuses[adjacentGroup] = math.max(existingBonus, timingBonus);
                                        }
                                        else
                                        {
                                            greenWaveBonuses[adjacentGroup] = timingBonus;
                                        }
                                        greenWaveActiveCount++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            // Debug: Log Green Wave activity (only occasionally to avoid spam)
            // Uncomment the line below to enable debug logging
            // if (greenWaveActiveCount > 0) System.Console.WriteLine($"[Green Wave] Node {currentNode.Index} coordinating with {greenWaveActiveCount} adjacent intersection(s)");
            
            // Note: Current phase duration checking is handled in the main update loop
            // This section is for phase selection optimization
            
            for (int i = 0; i < customPhaseDataBuffer.Length; i++)
            {
                CustomPhaseData phase = customPhaseDataBuffer[i];
                byte phaseGroup = (byte)(i + 1);
                
                // Penalize current group if it's been active (indicated by m_TurnsSinceLastRun == 0)
                // This prevents the same group from being selected again immediately
                // For Advanced Split Phasing, this is critical to ensure fair rotation
                float priorityPenalty = 0f;
                if (currentGroup > 0 && phaseGroup == currentGroup && phase.m_TurnsSinceLastRun == 0)
                {
                    // Reduce priority for current group to allow others to have a chance
                    // This helps prevent one direction from blocking others
                    // Use a more significant penalty to ensure rotation
                    priorityPenalty = 2f; // Subtract 2 from priority (increased from 1f)
                }
                
                // Calculate weighted waiting time
                // This considers lane occupancy, waiting time, and how long since last run
                float weightedWaiting = ((float)phase.TotalLaneOccupied()) * phase.m_LaneOccupiedMultiplier * math.pow((float)phase.m_TurnsSinceLastRun / (float)customPhaseDataBuffer.Length, phase.m_IntervalExponent);
                
                // Apply Green Wave bonus if available
                // Green Wave coordination helps synchronize adjacent intersections
                if (hasGreenWaveData && greenWaveBonuses.TryGetValue(phaseGroup, out float bonus))
                {
                    weightedWaiting *= (1.0f + bonus * 0.5f); // 50% bonus for green wave coordination
                }
                
                // Apply priority with penalty
                // The penalty ensures that even high-priority groups will eventually yield
                int effectivePriority = phase.m_Priority - (int)priorityPenalty;
                
                if (effectivePriority > maxPriority)
                {
                    nextGroup = phaseGroup;
                    maxPriority = effectivePriority;
                    maxWaiting = weightedWaiting;
                }
                else if (effectivePriority == maxPriority && weightedWaiting > maxWaiting)
                {
                    nextGroup = phaseGroup;
                    maxWaiting = weightedWaiting;
                }
                phase.m_WeightedWaiting = weightedWaiting;
                customPhaseDataBuffer[i] = phase;
            }
            
            // Only dispose if we actually allocated it
            if (hasGreenWaveData)
            {
                greenWaveBonuses.Dispose();
            }

            int linkedPriority = -1;
            byte linkedNextGroup = 0;
            for (int i = currentGroup - 1; i >= 0 && i < customPhaseDataBuffer.Length - 1; i++)
            {
                CustomPhaseData phase = customPhaseDataBuffer[i];
                if ((phase.m_Options & CustomPhaseData.Options.LinkedWithNextPhase) == 0)
                {
                    break;
                }

                CustomPhaseData nextPhase = customPhaseDataBuffer[i + 1];
                if (linkedNextGroup == 0 && nextPhase.m_Priority > 0)
                {
                    linkedNextGroup = (byte)(i + 2);
                }
                linkedPriority = math.max(linkedPriority, nextPhase.m_Priority);
            }
            if (linkedNextGroup > 0 && linkedPriority >= maxPriority)
            {
                linked = true;
                return linkedNextGroup;
            }

            for (int i = nextGroup - 2; i >= 0; i--)
            {
                CustomPhaseData phase = customPhaseDataBuffer[i];
                if ((phase.m_Options & CustomPhaseData.Options.LinkedWithNextPhase) == 0)
                {
                    break;
                }
                if (phase.m_Priority > 0)
                {
                    nextGroup = (byte)(i + 1);
                }
            }
            
            // Safety fallback: if no group was selected (nextGroup is 0), select the first available phase
            // This prevents the traffic light from getting stuck in None state
            if (nextGroup == 0 && customPhaseDataBuffer.Length > 0)
            {
                // Find the first phase with any priority or lane occupancy
                for (int i = 0; i < customPhaseDataBuffer.Length; i++)
                {
                    CustomPhaseData phase = customPhaseDataBuffer[i];
                    if (phase.m_Priority > 0 || phase.TotalLaneOccupied() > 0)
                    {
                        nextGroup = (byte)(i + 1);
                        break;
                    }
                }
                // If still no group found, default to first phase
                if (nextGroup == 0)
                {
                    nextGroup = 1;
                }
            }
            
            return nextGroup;
        }

        private static int MaxPriority(DynamicBuffer<CustomPhaseData> customPhaseDataBuffer)
        {
            int max = int.MinValue;
            foreach (var phase in customPhaseDataBuffer)
            {
                max = math.max(max, phase.m_Priority);
            }
            return max;
        }

        private static float Max(float4 f)
        {
            return math.max(f.w, math.max(f.x, math.max(f.y, f.z)));
        }
    }
}