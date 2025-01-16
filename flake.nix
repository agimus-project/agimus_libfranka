{
  description = "With this library, you can control research versions of Franka Robotics robots.";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
  };

  outputs =
    { nixpkgs, nix-ros-overlay, self, ... }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nix-ros-overlay.inputs.nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in
      {
        packages = {
          default = self.packages.${system}.libfranka;
          libfranka = pkgs.callPackage ./default.nix {};
        };
      }
    );
}