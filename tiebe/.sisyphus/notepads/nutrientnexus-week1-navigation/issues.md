## [2026-05-26T11:03:51Z] Known Issues / Gotchas

- SLAM rebuilds map every run (map persistence deferred to Week 2)
- Odometry drift acceptable 10-20cm for zone detection
- Real robot Nav2 tuning deferred (accept "good enough")
- Gazebo startup takes ~30s - integration tests need sleep/wait
- TURTLEBOT3_MODEL env var must be set or hardcoded in launch files
- ROS_DOMAIN_ID must match between laptop and real robot

- Task 10 limitation: environment verification could prove launch wiring and readiness structure, but not a stable full Nav2 goal-to-zone_a execution path; evidence records this explicitly to avoid claiming end-to-end navigation success.
- Task 12 blocker: this session has no access to physical TurtleBot3 hardware, so real-robot validation evidence must be captured later by a human operator.
- F2 review: earlier changed-file-only scope flagged README absolute-path drift and world/README architecture drift. The direct-workspace launch failure was later fixed by adding source-tree fallbacks in the launch files and `zone_detector.py`; local pytest now passes again.
