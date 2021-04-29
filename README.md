This repository is a project for CS591C1 (Spring 2021).

Team members are Xingjian (Jessie) Han and Benjamin Verdier.

The important modifications are preceded by a comment containing `CS591 related`.

# Instant Stitch Meshes

A combination of [Stitch Meshing](https://github.com/kuiwuchn/stitchMeshing) and [Instant Meshes](https://github.com/wjakob/instant-meshes) (IM).

The goal of this project is to use IM as the quad-dominant mesh generator instead of [Robust Quad/Hex-dominant Meshes](https://github.com/gaoxifeng/robust_hex_dominant_meshing) (RIM).

The reason for this is that RIM is an extension of IM to hex-dominant meshes, and Stitch Meshing uses the quad-dominant version. Additionally, IM offers more functionnalities for the surface case, such as user-defined orientation fields.

## Compiling

Compiling from scratch requires CMake and a recent version of XCode on Mac,
Visual Studio 2015 on Windows. This has not been tested on Linux.

First, install [Gurobi](https://www.gurobi.com/). The version used for testing was 7.5.2, available [here (Windows)](https://packages.gurobi.com/7.5/Gurobi-7.5.2-win32.msi) and [here (Mac)](https://packages.gurobi.com/7.5/gurobi7.5.2_mac64.pkg). Windows needs the 32 bits version.

On MacOS, compiling should be straightforward:

    git clone --recursive https://github.com/BenjaminVerdier/InstantStitchMeshes
    cd InstantStitchMeshes
    mkdir build && cd build
    cmake ..
    make -j 4

Note that some further adjustments in CmakeList.txt will be needed for linking Gurobi library before compiling: line ```197```

    optimized gurobi_c++
    debug gurobi_c++


On Windows, open the generated file ``InstantMeshes.sln`` after step 3 and proceed building as usual from within Visual Studio.

## Usage

### Quadrangulation

For the quadrangulation part, the workflow is the same as in IM:

To get started, launch the binary and select a dataset using the "Open mesh" button on the top left (the application must be located in the same directory as the 'datasets' folder, otherwise the panel will be empty).

The standard workflow is to solve for an orientation field (first blue button) and a position field (second blue button) in sequence, after which the 'Export mesh' button becomes active. Many user interface elements display a descriptive message when hovering the mouse cursor above for a second.

A range of additional information about the input mesh, the computed fields,
and the output mesh can be visualized using the check boxes accessible via the
'Advanced' panel.

Clicking the left mouse button and dragging rotates the object; right-dragging
(or shift+left-dragging) translates, and the mouse wheel zooms. The fields can also be manipulated using brush tools that are accessible by clicking the first icon in each 'Tool' row.

### Stitch Meshing

To get the Stitch Meshing part to work, there needs to be no polygons with more than 4 sides. To implement this, check the "Pure Quads" checkbox in the "Export" menu to divide every polygon into quads through their center, or use the "Split n-gons" buttons to split pentagons and up into triangles/quads through their flatest angle.

Once the mesh is ready, use "Prep Label" to compute the polyhedron structure used by the Stitch Meshing code. After this, you can select the first brush tool above to define certain labels on certain edges: Left click to make it a Wale edge, right-click to make it a Course edge.
Note: Those constraints are satisfied if possible. If there is an impossibility the solver will most likely ignore one or more of those constraints.

To label the entire mesh, click "Label". This will assign Course (red) and Wale (green) labels to the mesh elements, splitting and rearranging as necessary.

Use the second brush tool above to select the alignment picking tool. Click on a face to change the course direction of the row it's in. There is no particular rule for determining in advance which direction the arrow will point towards, but right-click will always be one direction and left-click will always be the other.

To align, click "Align". This will define a Course direction for every row.

To compute the final stitch mesh, click "Stitch Mesh". This will split all elements into quads through their respective centers.
