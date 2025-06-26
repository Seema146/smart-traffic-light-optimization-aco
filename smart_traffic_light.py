import os
import sys
import traci
import random
import numpy as np

# Set SUMO_HOME path 
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# ACO Parameters
NUM_ANTS = 8
PHEROMONE_INIT = 1.0
EVAPORATION_RATE = 0.4
ALPHA = 1.0
BETA = 4.0
PHASE_OPTIONS = [5, 10, 15, 20, 25, 30, 35]  # Added longer durations
WAITING_TIME_THRESHOLD = 10
HIGHLIGHTED_EDGE = "e1_out_east"  # Eastbound from n1 to n2_out_east

# Start SUMO simulation
sumoBinary = "sumo-gui"
sumoCmd = [sumoBinary, "-c", "sumo_config.sumocfg", "--start"]
traci.start(sumoCmd)

try:
    # Get traffic light IDs
    tl_ids = traci.trafficlight.getIDList()
    print(f"Traffic lights found: {tl_ids}")

    if not tl_ids:
        print("No traffic lights found. Check network.net.xml.")
        traci.close()
        sys.exit(1)

    # Initialize pheromone trails
    pheromones = {tl: {dur: PHEROMONE_INIT for dur in PHASE_OPTIONS} for tl in tl_ids}

    # Controlled lanes aur edge-to-lane mappings ko cache karo
    tl_lanes = {tl: traci.trafficlight.getControlledLanes(tl) for tl in tl_ids}
    edge_to_lanes = {}
    for tl, lanes in tl_lanes.items():
        for lane in lanes:
            edge = lane.rsplit("_", 1)[0]  # e.g., e1_out_east_0 -> e1_out_east
            if edge not in edge_to_lanes:
                edge_to_lanes[edge] = []
            edge_to_lanes[edge].append(lane)

    # Direction mappings define karo (jo ab bidirectional edges ke liye updated hain).
    direction_map = {
        "n1": {
            "north-south": ["e1_in_north", "e1_out_north", "e1_in_south", "e1_out_south"],
            "east-west": ["e1_in_west", "e1_out_west", "e1_in_east", "e1_out_east"]
        },
        "n2": {
            "north-south": ["e2_in_north", "e2_out_north", "e2_in_south", "e2_out_south"],
            "east-west": ["e2_in_west", "e2_out_west", "e2_in_east", "e2_out_east"]
        },
        "n3": {
            "north-south": ["e3_in_north", "e3_out_north", "e3_in_south", "e3_out_south"],
            "east-west": ["e3_in_west", "e3_out_west", "e3_in_east", "e3_out_east"]
        },
        "n4": {
            "north-south": ["e4_in_north", "e4_out_north", "e4_in_south", "e4_out_south"],
            "east-west": ["e4_in_west", "e4_out_west", "e4_in_east", "e4_out_east"]
        }
    }

    # Pehle ke queue sizes ko track karo taaki persistent queues detect ki ja sake.
    previous_queues = {tl: {"ns_in": 0, "ew_in": 0} for tl in tl_ids}

    def get_traffic_metrics(tl_id):
        """Calculate queue length and waiting time, with focus on highlighted edge."""
        lanes = tl_lanes[tl_id]
        queue_length = sum(traci.lane.getLastStepVehicleNumber(lane) for lane in lanes)
        waiting_time = sum(traci.lane.getWaitingTime(lane) for lane in lanes)

        # Highlighted edge (n1 se n2_out_east tak eastbound) ke liye metrics check karo.
        highlighted_lanes = edge_to_lanes.get(HIGHLIGHTED_EDGE, [])
        highlighted_queue = sum(traci.lane.getLastStepVehicleNumber(lane) for lane in highlighted_lanes)
        highlighted_waiting = sum(traci.lane.getWaitingTime(lane) for lane in highlighted_lanes)

        # Har travel direction ke hisaab se alag-alag queues (directional queues) banayi.
        ns_edges = direction_map[tl_id]["north-south"]
        ew_edges = direction_map[tl_id]["east-west"]
        ns_in_edges = [e for e in ns_edges if "in" in e]
        ns_out_edges = [e for e in ns_edges if "out" in e]
        ew_in_edges = [e for e in ew_edges if "in" in e]
        ew_out_edges = [e for e in ew_edges if "out" in e]

        ns_in_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                          for edge in ns_in_edges for lane in edge_to_lanes.get(edge, []))
        ns_out_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                           for edge in ns_out_edges for lane in edge_to_lanes.get(edge, []))
        ew_in_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                          for edge in ew_in_edges for lane in edge_to_lanes.get(edge, []))
        ew_out_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                           for edge in ew_out_edges for lane in edge_to_lanes.get(edge, []))

        print(f"Step {step}: TL {tl_id} - NS In Queue: {ns_in_queue}, NS Out Queue: {ns_out_queue}, "
              f"EW In Queue: {ew_in_queue}, EW Out Queue: {ew_out_queue}, "
              f"Highlighted Queue: {highlighted_queue}, Highlighted Wait: {highlighted_waiting:.2f}")

        return queue_length, waiting_time, highlighted_queue, highlighted_waiting, ns_in_queue, ns_out_queue, ew_in_queue, ew_out_queue

    def get_downstream_queue(tl_id):
        """Measure downstream queue for coordination."""
        downstream_edges = {
            "n1": ["e1_out_south", "e1_out_east"],
            "n2": ["e2_out_south", "e2_out_east"],
            "n3": ["e3_out_south", "e3_out_east"],
            "n4": ["e4_out_south", "e4_out_east"]
        }.get(tl_id, [])
        downstream_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                               for edge in downstream_edges for lane in edge_to_lanes.get(edge, []))
        print(f"Step {step}: TL {tl_id} - Downstream Queue: {downstream_queue}")
        return downstream_queue

    def coordinate_traffic_lights(tl_id, chosen_duration):
        """Coordinate downstream traffic lights for smooth flow."""
        downstream_map = {
            "n1": ["n3", "n2"],
            "n2": ["n4", "n1"],
            "n3": ["n4", "n1"],
            "n4": ["n3", "n2"]
        }
        for downstream_tl in downstream_map.get(tl_id, []):
            if downstream_tl in tl_ids:
                downstream_state = traci.trafficlight.getRedYellowGreenState(downstream_tl)
                downstream_queue = sum(traci.lane.getLastStepVehicleNumber(lane)
                                       for lane in tl_lanes[downstream_tl])
                if "r" in downstream_state.lower() and downstream_queue > 0:
                    current_phase = traci.trafficlight.getPhase(tl_id)
                    target_phase = 0 if current_phase == 0 else 2
                    # Ensure at least 15 seconds of green time to clear downstream queue
                    downstream_duration = max(chosen_duration, 15)
                    traci.trafficlight.setPhase(downstream_tl, target_phase)
                    traci.trafficlight.setPhaseDuration(downstream_tl, downstream_duration)
                    print(f"Step {step}: Coordinating {downstream_tl} to phase {target_phase} "
                          f"for {downstream_duration}s to clear queue ({downstream_queue}) after {tl_id}")

    def choose_duration(tl_id, pheromones, ns_in_queue, ew_in_queue):
        """Ant chooses duration with priority on largest queue."""
        queue_length, waiting_time, highlighted_queue, highlighted_waiting, _, _, _, _ = get_traffic_metrics(tl_id)
        downstream_queue = get_downstream_queue(tl_id)

        max_in_queue = max(ns_in_queue, ew_in_queue)
        queue_priority = 2.5 if max_in_queue > 3 else 1.0  # Increased priority and lowered threshold
        ns_priority = queue_priority if ns_in_queue == max_in_queue else 1.0
        ew_priority = queue_priority if ew_in_queue == max_in_queue else 1.0

        heuristic = {}
        for dur in PHASE_OPTIONS:
            queue_pressure = queue_length + 1
            wait_pressure = waiting_time + 1
            downstream_pressure = downstream_queue + 1
            highlighted_pressure = (highlighted_queue + 1) * (highlighted_waiting + 1) * 1.2  # Reduced weighting
            efficiency = (queue_pressure + wait_pressure + highlighted_pressure) / downstream_pressure

            if ns_in_queue > ew_in_queue:
                heuristic[dur] = efficiency * (dur * ns_priority if waiting_time > WAITING_TIME_THRESHOLD else 1 / (dur + 1))
            else:
                heuristic[dur] = efficiency * (dur * ew_priority if waiting_time > WAITING_TIME_THRESHOLD else 1 / (dur + 1))

        probs = {}
        for dur in PHASE_OPTIONS:
            probs[dur] = (pheromones[tl_id][dur] ** ALPHA) * (heuristic[dur] ** BETA)
        total = sum(probs.values())
        if total == 0:
            return random.choice(PHASE_OPTIONS)
        probs = {dur: p / total for dur, p in probs.items()}
        return random.choices(PHASE_OPTIONS, weights=probs.values(), k=1)[0]

    def evaluate_solution(tl_id, duration):
        """Evaluate duration dynamically, focusing on queue reduction."""
        traci.trafficlight.setPhaseDuration(tl_id, duration)
        total_queue_length = 0
        total_waiting_time = 0
        highlighted_queue = 0
        highlighted_waiting = 0
        steps = max(5, min(15, duration))
        for _ in range(steps):
            if traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                total_queue_length += sum(traci.lane.getLastStepVehicleNumber(lane) for lane in tl_lanes[tl_id])
                total_waiting_time += sum(traci.lane.getWaitingTime(lane) for lane in tl_lanes[tl_id])
                highlighted_lanes = edge_to_lanes.get(HIGHLIGHTED_EDGE, [])
                highlighted_queue += sum(traci.lane.getLastStepVehicleNumber(lane) for lane in highlighted_lanes)
                highlighted_waiting += sum(traci.lane.getWaitingTime(lane) for lane in highlighted_lanes)
            else:
                break
        avg_queue = total_queue_length / max(1, steps)
        avg_wait = total_waiting_time / max(1, steps)
        avg_hl_queue = highlighted_queue / max(1, steps)
        avg_hl_wait = highlighted_waiting / max(1, steps)
        return avg_queue, avg_wait, avg_hl_queue, avg_hl_wait

    def update_pheromones(tl_id, duration, avg_queue_length, avg_waiting_time, avg_highlighted_queue, avg_highlighted_waiting):
        """Update pheromones with emphasis on queue reduction."""
        queue_penalty = 1.0 + (avg_queue_length * 0.5)
        reward = 1.0 / (queue_penalty + avg_waiting_time + avg_highlighted_queue * 1.2 + avg_highlighted_waiting * 1.2 + 1)
        pheromones[tl_id][duration] = (1 - EVAPORATION_RATE) * pheromones[tl_id][duration] + reward

    # Main simulation loop
    step = 0
    max_steps = 10000
    initial_vehicles = traci.simulation.getMinExpectedNumber()
    print(f"Starting simulation with {initial_vehicles} vehicles expected.")

    min_steps = 100 if initial_vehicles == 0 else 0
    current_green_direction = {tl: None for tl in tl_ids}

    while step < max_steps and (traci.simulation.getMinExpectedNumber() > 0 or step < min_steps):
        traci.simulationStep()

        if step % 3 == 0:  # Update every 3 steps instead of 5
            for tl_id in tl_ids:
                if tl_id not in traci.trafficlight.getIDList():
                    print(f"Traffic light {tl_id} not found, skipping.")
                    continue

                queue_length, waiting_time, highlighted_queue, highlighted_waiting, ns_in_queue, ns_out_queue, ew_in_queue, ew_out_queue = get_traffic_metrics(tl_id)

                # Jo queues continuously bani hui hain, unko check karo.
                if ns_in_queue > 0 and previous_queues[tl_id]["ns_in"] > 0 and ns_in_queue >= previous_queues[tl_id]["ns_in"]:
                    traci.trafficlight.setPhase(tl_id, 0)  # North-south green
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    print(f"Step {step}: TL {tl_id} extended green to {max(PHASE_OPTIONS)}s for north-south "
                          f"due to persistent queue (in queue: {ns_in_queue})")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue
                elif ew_in_queue > 0 and previous_queues[tl_id]["ew_in"] > 0 and ew_in_queue >= previous_queues[tl_id]["ew_in"]:
                    traci.trafficlight.setPhase(tl_id, 2)  # East-west green
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    print(f"Step {step}: TL {tl_id} extended green to {max(PHASE_OPTIONS)}s for east-west "
                          f"due to persistent queue (in queue: {ew_in_queue})")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue

                # Update previous queues
                previous_queues[tl_id]["ns_in"] = ns_in_queue
                previous_queues[tl_id]["ew_in"] = ew_in_queue

                # Zero-traffic scenarios
                if ns_in_queue == 0 and ew_in_queue > 0:
                    traci.trafficlight.setPhase(tl_id, 2)  # East-west green
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    current_green_direction[tl_id] = "east-west"
                    print(f"Step {step}: TL {tl_id} giving continuous green to east-west (north-south empty)")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue
                elif ew_in_queue == 0 and ns_in_queue > 0:
                    traci.trafficlight.setPhase(tl_id, 0)  # North-south green
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    current_green_direction[tl_id] = "north-south"
                    print(f"Step {step}: TL {tl_id} giving continuous green to north-south (east-west empty)")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue

                # Jis direction mein sabse bada incoming queue ho, usko priority do
                if ns_in_queue > ew_in_queue and ns_in_queue > 3:  # Lowered threshold to 3
                    traci.trafficlight.setPhase(tl_id, 0)  # North-south wali green light
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    print(f"Step {step}: TL {tl_id} extended green to {max(PHASE_OPTIONS)}s for north-south "
                          f"(in queue: {ns_in_queue})")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue
                elif ew_in_queue > ns_in_queue and ew_in_queue > 3:
                    traci.trafficlight.setPhase(tl_id, 2)  # East-west green
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    print(f"Step {step}: TL {tl_id} extended green to {max(PHASE_OPTIONS)}s for east-west "
                          f"(in queue: {ew_in_queue})")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue

                # Highlighted edge ko priority dene wale rule pe wapas aa jao
                if HIGHLIGHTED_EDGE in [lane.rsplit("_", 1)[0] for lane in tl_lanes[tl_id]] and highlighted_waiting > WAITING_TIME_THRESHOLD:
                    traci.trafficlight.setPhase(tl_id, 2)  # East-west green for e1_out_east
                    traci.trafficlight.setPhaseDuration(tl_id, max(PHASE_OPTIONS))
                    print(f"Step {step}: TL {tl_id} extended green to {max(PHASE_OPTIONS)}s for highlighted edge "
                          f"(wait: {highlighted_waiting:.2f}s)")
                    coordinate_traffic_lights(tl_id, max(PHASE_OPTIONS))
                    continue

                # ACO optimization
                best_duration = None
                best_score = float('inf')
                for ant in range(NUM_ANTS):
                    duration = choose_duration(tl_id, pheromones, ns_in_queue, ew_in_queue)
                    avg_queue_length, avg_waiting_time, avg_highlighted_queue, avg_highlighted_waiting = evaluate_solution(tl_id, duration)
                    score = avg_queue_length + avg_waiting_time + (avg_highlighted_queue + avg_highlighted_waiting) * 1.2
                    update_pheromones(tl_id, duration, avg_queue_length, avg_waiting_time, avg_highlighted_queue, avg_highlighted_waiting)

                    if score < best_score:
                        best_score = score
                        best_duration = duration

                if best_duration is not None:
                    traci.trafficlight.setPhaseDuration(tl_id, best_duration)
                    if ns_in_queue > ew_in_queue:
                        traci.trafficlight.setPhase(tl_id, 0)  # North-south green
                        print(f"Step {step}: TL {tl_id} set phase to north-south green")
                    else:
                        traci.trafficlight.setPhase(tl_id, 2)  # East-west green
                        print(f"Step {step}: TL {tl_id} set phase to east-west green")
                    coordinate_traffic_lights(tl_id, best_duration)
                    print(f"Step {step}: TL {tl_id} set green duration to {best_duration}s, "
                          f"Avg Queue: {avg_queue_length:.2f}, Avg Wait: {avg_waiting_time:.2f}, "
                          f"Highlighted Queue: {avg_highlighted_queue:.2f}, Highlighted Wait: {avg_highlighted_waiting:.2f}")

        step += 1
        if step % 100 == 0:
            print(f"Step {step}: Vehicles remaining: {traci.simulation.getMinExpectedNumber()}")

except traci.FatalTraCIError as e:
    print(f"TraCI error: {e}. Simulation may have ended unexpectedly.")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    traci.close()
    print("Simulation completed.")