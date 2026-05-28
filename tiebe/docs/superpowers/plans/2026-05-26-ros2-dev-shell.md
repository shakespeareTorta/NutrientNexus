# ROS 2 Dev Shell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the broken local `shell.nix` import with a working ROS 2 Jazzy development shell based on `nix-ros-overlay` for TurtleBot3 Burger and Gazebo work.

**Architecture:** Add a `flake.nix` as the source of truth, wired exactly the way `nix-ros-overlay` expects, then reduce `shell.nix` to a compatibility shim that imports the flake's default dev shell. Keep package selection focused on ROS core, Gazebo integration, and standard TurtleBot3 packages. Preserve the existing Wayland workaround in the shell hook.

**Tech Stack:** Nix flakes, `lopsided98/nix-ros-overlay`, ROS 2 Jazzy, Gazebo, TurtleBot3 packages

---

### Task 1: Add the flake entrypoint

**Files:**
- Create: `flake.nix`

- [ ] **Step 1: Add `flake.nix` with the overlay and pinned `nixpkgs`**

```nix
{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    let
      system = builtins.currentSystem;
      pkgs = import nixpkgs {
        inherit system;
        overlays = [ nix-ros-overlay.overlays.default ];
      };
      ros = pkgs.rosPackages.jazzy;
    in {
      devShells.${system}.default = pkgs.mkShell {
        nativeBuildInputs = [
          (ros.buildEnv {
            paths = with ros; [
              colcon
              ros-core
              ament-cmake-core
              python-cmake-module
              ros-gz
              gz-launch-vendor
              turtlebot3
              turtlebot3-msgs
              turtlebot3-description
              turtlebot3-simulations
            ];
          })
        ];

        shellHook = ''
          unset QT_QPA_PLATFORM
        '';
      };
    };

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
```

- [ ] **Step 2: Evaluate the flake shell to verify package names exist**

Run: `nix develop .#default --command true`
Expected: success, or a concrete missing-package error naming the package to adjust

### Task 2: Replace the legacy shell file with a shim

**Files:**
- Modify: `shell.nix`

- [ ] **Step 1: Replace `shell.nix` with a flake compatibility shim**

```nix
(builtins.getFlake (toString ./. )).devShells.${builtins.currentSystem}.default
```

- [ ] **Step 2: Verify the legacy entrypoint still works**

Run: `nix-shell --run true`
Expected: success, with Nix entering and exiting the same shell definition from the flake

### Task 3: Validate the ROS shell behavior

**Files:**
- No file changes expected

- [ ] **Step 1: Check that the shell exposes ROS 2 Jazzy tools**

Run: `nix develop .#default --command bash -lc 'printenv ROS_DISTRO && command -v colcon'`
Expected: `jazzy` is printed and `colcon` resolves to a store path

- [ ] **Step 2: Check that Gazebo and TurtleBot3 packages evaluate through the shell**

Run: `nix develop .#default --command bash -lc 'command -v ros2 && test -n "$AMENT_PREFIX_PATH"'`
Expected: `ros2` resolves and `AMENT_PREFIX_PATH` is populated

- [ ] **Step 3: Confirm the Wayland workaround remains active**

Run: `nix develop .#default --command bash -lc 'test -z "$QT_QPA_PLATFORM"'`
Expected: success because `QT_QPA_PLATFORM` is unset inside the shell
