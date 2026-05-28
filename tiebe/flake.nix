{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { nix-ros-overlay, nixpkgs, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        lib = pkgs.lib;
        ros = pkgs.rosPackages.jazzy;
        rosIfPresent = names:
          builtins.concatLists (map (name: lib.optionals (builtins.hasAttr name ros) [ (builtins.getAttr name ros) ]) names);
      in {
        devShells.default = pkgs.mkShell {
          packages = [
            pkgs.colcon
            (ros.buildEnv {
              paths =
                (with ros; [
                  ros-core
                  ament-cmake-core
                  python-cmake-module
                ])
                ++ rosIfPresent [
                  "ros-gz"
                  "ros-gz-sim"
                  "gz-launch-vendor"
                  "turtlebot3-msgs"
                  "turtlebot3-description"
                  "turtlebot3-simulations"
                  "turtlebot3-gazebo"
                  "turtlebot3-navigation2"
                  "turtlebot3-teleop"
                ];
            })
          ];

          shellHook = ''
            export ROS_DISTRO=jazzy
            export TURTLEBOT3_MODEL=burger
            # Gazebo is currently broken on Wayland.
            unset QT_QPA_PLATFORM
          '';
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
