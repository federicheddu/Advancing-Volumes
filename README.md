# Advancing Volumes

> Thesis project - **MD in Computer Science** (**Università degli Studi di Cagliari**)

> [!NOTE]
> I am still developing this project as a PhD student

| **Student**          | **ID**      | **E-Mail**                        |
|----------------------|-------------|-----------------------------------|
| Federico Meloni      | 60/73/65243 | <f.meloni62@studenti.unica.it>    |

## What is this project about
This thesis project aims to create a tet-mesh by placing a starting model *- a sphere in our case, but it can be any model -*
inside a surface target mesh and deforming/refining it so that it fills the target volume.  
In the future this process can be done simultaneously on two models in order to create a map between them
(starting from the same model and doing the same refinement steps).

## How to use it
This project is written in C++ and uses the following libraries:
- **cinolib** (C++ library for computer graphics)
- **CGAL** (Computational Geometry Algorithms Library)

**Cinolib:**
``` bash
git clone https://github.com/mlivesu/cinolib.git
```
Then edit with your path `set(cinolib_DIR <your-path>)` in the `CMakeLists.txt` file.

**CGAL (MacOS):**
After installing `homebrew`, run the following command:
``` bash
brew install cgal
```

## Keyboard commands

|  **Key**  | **Action**                      | **Type** |
|:---------:|---------------------------------|:--------:|
| **SPACE** | Advancing Volumes (1 iteration) |  Action  |
|   **M**   | View/Hide target mesh           |  Toggle  |

