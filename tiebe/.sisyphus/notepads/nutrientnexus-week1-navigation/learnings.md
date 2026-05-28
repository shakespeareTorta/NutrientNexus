 - Zone detection tests can stay fully unit-level by subclassing the ROS node and overriding initialization to inject in-memory zone data.
 - The current `get_zone()` implementation is boundary-inclusive (`<=` on both ends), so coverage should include edge coordinates.

## [2026-05-26T12:15:31Z] Task 11 README notes
- Added project-root README with the 10 required sections for the Week 1 navigation foundation.
- Documented the active world file as `nutrient_nexus_navigation/worlds/farm_box.world` and traced it back to `docs/SimFiles/new_world.world`.
- Kept the README scoped to implemented launch, config, world, zone, and test artifacts only; avoided claiming unimplemented autonomy features.
