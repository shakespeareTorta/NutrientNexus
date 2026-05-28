## [2026-05-26T11:03:51Z] Architectural Decisions

- Package: single monolithic nutrient_nexus_navigation (no splitting)
- Navigation: SLAM toolbox + Nav2 (no custom path planning)
- Zones: 3 zones (zone_a, zone_b, zone_c) in 4m x 4m world
- Launch: separate sim_bringup.launch.py and real_bringup.launch.py
- Tests: tests-after approach (pytest + launch_testing)
- Real robot: manual validation acceptable (Week 1)

- Decision: Keep the RViz config minimal and YAML-only, matching the existing ROS2/RViz2 convention and the task requirements.
