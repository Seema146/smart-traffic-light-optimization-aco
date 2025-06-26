# Smart Traffic Light Optimization using Ant Colony Optimization (ACO)

This project implements an **AI-powered smart traffic light system** using the **Ant Colony Optimization (ACO)** algorithm in **SUMO (Simulation of Urban Mobility)**. It aims to minimize vehicle waiting time and optimize signal timings in a multi-intersection traffic network.

---

## Project Features

- **Ant Colony Optimization (ACO)** applied to dynamic traffic signal durations.
- Real-time traffic metrics (queue length, waiting time) using SUMO’s `traci` module.
- Coordination between multiple traffic lights for smoother vehicle flow.
- Focused optimization on eastbound movement from node `n1` to `n2` (`e1_out_east`).
- Intelligent queue balancing between north-south and east-west lanes.
- Visual GUI settings with realistic road schemes and vehicle labels.

---

## File Structure

