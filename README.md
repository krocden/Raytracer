# Raytracing (3D rendering method)
#### Raytracer Application in C++ using Eigen library. 

![cornel_box](https://github.com/user-attachments/assets/6a9eadff-4943-401f-b0d9-a7c342a33fb7)

This application takes a scene in .JSON format and 
generates the scene depending on the parameters.\
Supports local-illumination, global illumination, anti-aliasing , two-side rendering, and sphere and rectangle primitives.

The source code is located in /src\
This application requires Eigen library.

Building the solution:

    1) Adjust directory for Eigen libarry in CMakeLists.txt 
    2) Create build folder: mkdir build
    3) Create the src folder: mkdir src
    3) cd build
    3) cmake ../
    4) make
    5) ./raytracer <filename.json>

Area lighting can take a long time to compute so I made it faster by changing the 
sampling, which makes it less accuate but a faster. This is especially important 
when combined with global illumination which is more complex computationally
than local-illumination.
