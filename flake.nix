{
  description = "Tutorial for humanoid path planner platform";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros.overrideAttrs.hpp-tutorial = _: {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./Media
                  ./meshes
                  ./package.xml
                  ./rviz
                  ./script
                  ./src
                  ./srdf
                  ./urdf
                ];
              };
            };
          }
        ];
      }
    );
}
