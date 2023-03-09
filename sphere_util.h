//
// Created by Federico Meloni on 06/03/23.
//

#ifndef DISTANCE_FIELD_SPHERE_UTIL_H
#define DISTANCE_FIELD_SPHERE_UTIL_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/gl/glcanvas.h>
#include <cinolib/gl/volume_mesh_controls.h>
#include <cinolib/gl/surface_mesh_controls.h>
#include "text_util.h"

using namespace cinolib;

void trasl_center(DrawableTetmesh<> &m);
void scale_unicube(DrawableTetmesh<> &m);
void mesh_standardizer(DrawableTetmesh<> &m);

DrawableTetmesh<> get_sphere(const vec3d &center = vec3d(0.0, 0.0, 0.0),
                             const double &scale_factor = 1.0,
                             const int &num_polys = 18);

DrawableTetmesh<> get_sphere18();
DrawableTetmesh<> get_sphere31();
DrawableTetmesh<> get_sphere40();
DrawableTetmesh<> get_sphere50();

void show_allspheres(GLcanvas &gui);

#endif //DISTANCE_FIELD_SPHERE_UTIL_H
