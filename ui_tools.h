#ifndef ADVANCING_VOLUMES_UI_TOOLS_H
#define ADVANCING_VOLUMES_UI_TOOLS_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/drawable_arrow.h>
#include <cinolib/drawable_octree.h>
#include <cinolib/gl/glcanvas.h>

using namespace cinolib;

void showArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui);

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui);

void updateArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui);

void showFronts(DrawableTetmesh<> &m);

void deleteFronts(DrawableTetmesh<> &m);

#endif //ADVANCING_VOLUMES_UI_TOOLS_H
