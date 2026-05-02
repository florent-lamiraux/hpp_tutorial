{
  description = "Tutorial for humanoid path planner platform";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-tutorial = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./doc
              ./include
              ./launch
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
    );
}
