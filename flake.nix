{
  description = "ROS integration for Franka research robots";

  inputs = {
    gazebros2nix.url = "github:gepetto/gazebros2nix";
    flake-parts.follows = "gazebros2nix/flake-parts";
    nixpkgs.follows = "gazebros2nix/nixpkgs";
    nix-ros-overlay.follows = "gazebros2nix/nix-ros-overlay";
    systems.follows = "gazebros2nix/systems";
    treefmt-nix.follows = "gazebros2nix/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gazebros2nix.flakeModule
          { gazebros2nix-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default =
          _final: prev:
          let
            scope = _ros-final: ros-prev: {
              agimus-libfranka = ros-prev.agimus-libfranka.overrideAttrs {
                src = lib.fileset.toSource {
                  root = ./.;
                  fileset = lib.fileset.unions [
                    ./cmake
                    ./doc
                    ./examples
                    ./include
                    ./scripts
                    ./src
                    ./test
                    ./CHANGELOG.md
                    ./CMakeLists.txt
                    ./LICENSE
                    ./NOTICE
                    ./README.md
                  ];
                };
              };
            };
          in
          {
            rosPackages = prev.rosPackages // {
              humble = prev.rosPackages.humble.overrideScope scope;
              jazzy = prev.rosPackages.jazzy.overrideScope scope;
              kilted = prev.rosPackages.kilted.overrideScope scope;
              rolling = prev.rosPackages.rolling.overrideScope scope;
            };
          };
        perSystem =
          { pkgs, ... }:
          {
            packages = lib.filterAttrs (_n: v: v.meta.available && !v.meta.broken) (rec {
              default = humble-agimus-libfranka;
              humble-agimus-libfranka = pkgs.rosPackages.humble.agimus-libfranka;
              # jazzy-agimus-libfranka = pkgs.rosPackages.jazzy.agimus-libfranka;
              # kilted-agimus-libfranka = pkgs.rosPackages.kilted.agimus-libfranka;
              # rolling-agimus-libfranka = pkgs.rosPackages.rolling.agimus-libfranka;
            });
          };
      }
    );
}
