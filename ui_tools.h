#ifndef ADVANCING_VOLUMES_UI_TOOLS_H
#define ADVANCING_VOLUMES_UI_TOOLS_H

#include <cinolib/meshes/meshes.h>
#include <cinolib/drawable_arrow.h>
#include <cinolib/drawable_octree.h>
#include <cinolib/gl/glcanvas.h>

using namespace cinolib;

void showArrows(Tetmesh<> &sp, Octree &oct, std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui);

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui);

void updateArrows(Tetmesh<> &sp, Octree &oct, std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui);

void showInactive(DrawableTetmesh<> &sp, std::set<uint> &inactive_verts, std::set<uint> &inactive_faces);

void hideInactive(DrawableTetmesh<> &sp, std::set<uint> &inactive_verts, std::set<uint> &inactive_faces);

#endif //ADVANCING_VOLUMES_UI_TOOLS_H
