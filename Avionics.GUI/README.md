Avionics
===========================

## Installation (Visual Studio 2013)

- Run CMake in the root directories of ThirdParty/freetype-gl and ThirdParty/glfw
- Download glew from https://sourceforge.net/projects/glew/files/glew/1.11.0/glew-1.11.0.zip/download
- Extract glew into ThirdParty/glew-1.11.0 such that ThirdParty/glew-1.11.0/Makefile exists
- Open Avionics.sln
- If prompted agree to upgrade projects
- Right click and reload any projects which failed to load
- Open the Configuration Manager
- Make sure all projects are set to build and to the same target architecture
- You will need to add a new architecture for some of the ThirdParty projects
- Build All

## Troubleshooting

If you are encountering linker errors try removing and re-adding all project references for Frontier.Engine and Avionics