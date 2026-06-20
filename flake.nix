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
              ./tutorial_1
              ./tutorial_2
              ./tutorial_3
              ./tutorial_4
              ./tutorial_4
              ./tutorial_6
              ./tutorial_7
              ./tutorial_8
              ./urdf
            ];
          };
        };
      }
    );
}
