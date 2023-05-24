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

