## Goal

Replace the broken ad-hoc `shell.nix` import with a reproducible ROS 2 Jazzy development shell based on `lopsided98/nix-ros-overlay`, targeting TurtleBot3 Burger development and Gazebo usage.

## Current Problem

The existing `shell.nix` uses `pkgs ? import ../. {}`. In this workspace, `../.` resolves to `/home/tiebe`, which is the home directory rather than a pinned `nixpkgs` or overlay source. As a result, the shell does not correctly import `nix-ros-overlay` and cannot reliably provide `rosPackages`.

## Recommended Approach

Use a flake as the source of truth and keep a small compatibility `shell.nix` shim.

Why this approach:

- It matches the upstream `nix-ros-overlay` documentation.
- It pins `nixpkgs` through `nix-ros-overlay/nixpkgs`, which the overlay expects.
- It makes the shell reproducible and easier to update.
- It still allows a simple entrypoint for users who expect a local shell file.

## File Layout

### `flake.nix`

Add a new `flake.nix` that:

- declares `nix-ros-overlay` as an input from `github:lopsided98/nix-ros-overlay/master`
- sets `nixpkgs.follows = "nix-ros-overlay/nixpkgs"`
- imports `nixpkgs` with `nix-ros-overlay.overlays.default`
- exposes `devShells.default`
- sets ROS cache settings in `nixConfig`

### `shell.nix`

Replace the current implementation with a minimal shim that delegates to the flake-based dev shell. This preserves `nix-shell` compatibility while avoiding duplicated package definitions.

## Shell Contents

The dev shell should target ROS 2 Jazzy and include:

- `colcon` for workspace builds
- a ROS package environment built from `pkgs.rosPackages.jazzy`
- `ros-core` as the base runtime
- `ament-cmake-core` and `python-cmake-module` for common package builds
- Gazebo integration packages such as `ros-gz` and `gz-launch-vendor`
- TurtleBot3 packages selected from the Jazzy package set by preferring `turtlebot3`, `turtlebot3-msgs`, `turtlebot3-description`, `turtlebot3-simulations`, and closely related simulation packages when they are present in the overlay

If some preferred TurtleBot3 packages are absent from the overlay, include the available upstream TurtleBot3 metapackages first and avoid inventing local package overrides in this change.

## Environment Behavior

Preserve the current shell workaround for Gazebo on Wayland by unsetting `QT_QPA_PLATFORM` in `shellHook`.

If the chosen ROS package set requires Qt wrapper handling for newer distros only, keep the workaround scoped to those distros. For Jazzy, avoid unnecessary Qt wrapper complexity unless evaluation proves it is required.

## Verification

Implementation will be considered successful when:

- `nix develop` evaluates successfully in `/home/tiebe/CBL`
- the shell exposes ROS 2 Jazzy packages
- TurtleBot3 and Gazebo packages evaluate successfully
- the shell still applies the Wayland workaround

## Out of Scope

- Full ROS workspace scaffolding
- User-specific graphics wrappers such as `nixGL`
- Launch script customization for TurtleBot3
- Non-Jazzy ROS distributions
