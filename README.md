# Advancing Volumes

> Thesis project - **MD in Computer Science** (**Università degli Studi di Cagliari**)

| **Student**          | **ID**      | **E-Mail**                        |
|----------------------|-------------|-----------------------------------|
| Federico Meloni      | 60/73/65243 | <f.meloni62@studenti.unica.it>    |

<details>
<summary><b>File Structure</b></summary>

```
·
│
│ CODE FILES
├── main.cpp
├── sphere.cpp/h        # utility functions for getting the sphere mesh
├── text_utils.h        # utility macros for text output
├── CMakeLists.txt
│
│ DATASET
├── spheres                     # sphere models
│   ├── sphere.mesh
│   ├── sphere_coarse.mesh
│   ├── sphere18.mesh
│   ├── sphere31.mesh
│   ├── sphere40.mesh
│   └── sphere50.mesh
├── data                        # tet models
│   └── ···
│
│ MARKDOWN & OTHER DOCUMENTATION STUFF
├── README.md
│
├── img                 # images for README.md
│   └── ···
│
│ OTHER
├── .gitignore
├── .gitattributes
└── LICENSE
```

</details>

## What is this project about
This thesis project aims to create a tet-mesh by placing a starting model *- a sphere in our case, but it can be any model -*
inside a surface target mesh and deforming/refining it so that it fills the target volume.  
This process can be done simultaneously on two models in order to create a map between them
(starting from the same model and doing the same refinement steps).

## Keyboard commands

| **Key** | **Action**                                       | **Type** |
|:-------:|--------------------------------------------------|:--------:|
|  **J**  | Target visualization mode (transparent or matte) |  Toggle  |
|  **K**  | Undo (only 1 action)                             |  Action  |
|  **L**  | Show the target                                  |  Toggle  |
|  **V**  | Show the direction where the verts will move     |  Toggle  |
|  **B**  | Show the inactive fronts in a different color    |  Toggle  |
|  **N**  | Expand the model moving the verts                |  Action  |
|  **M**  | Refine the model in the active front             |  Action  |

## Starting model placement
Our starting model is a sphere with 67V / 358E / 544F / 252P.  
This is placed at the point where the minimum distance from the target surface is the greatest,
with the radius of half of that distance.

We can potentially go on to select even different resolution spheres with 18/31/40/50 vertices as the starting model.

## Expanding the model
The model is expanded by going to move each vertex in the direction of its normal by a distance proportional 
to its minimum distance from the surface until a minimum stop distance is reached.

In case during the expansion the model intersects the target, the vertex that caused this intersection is moved 
back by half of its displacement up to 5 times, after which it is returned to its starting position if it continues 
to make the model intersect the target in any way.

