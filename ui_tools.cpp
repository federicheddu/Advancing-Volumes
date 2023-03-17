#include "ui_tools.h"

void showArrows(Tetmesh<> &sp, Octree &oct, std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui) {

    for (auto vid: sp.get_surface_verts()) {
        dir_arrows.emplace_back(sp.vert(vid),
                                sp.vert(vid) + sp.vert_data(vid).normal *
                                               oct.closest_point(sp.vert(vid)).dist(sp.vert(vid)));
        dir_arrows.back().size = 0.005;
    }

    for (auto &id: dir_arrows) {
        gui.push(&id, false);
    }

}

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui) {

    for (auto &id: dir_arrows) {
        gui.pop(&id);
    }

}

void updateArrows(Tetmesh<> &sp, Octree &oct, std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui) {
    deleteArrows(dir_arrows, gui);
    showArrows(sp, oct, dir_arrows, gui);
}

void showInactive(DrawableTetmesh<> &sp, std::set<uint> &inactive_verts, std::set<uint> &inactive_faces) {

    for(auto vid : inactive_verts)
        sp.vert_data(vid).color = Color::BLUE();

    for(auto fid : inactive_faces)
        sp.poly_data(sp.adj_f2p(fid)[0]).color = Color::PASTEL_CYAN();

}

void hideInactive(DrawableTetmesh<> &sp, std::set<uint> &inactive_verts, std::set<uint> &inactive_faces) {

    for(auto vid : inactive_verts)
        sp.vert_data(vid).color = Color::PASTEL_YELLOW();

    for(auto fid : inactive_faces)
        sp.poly_data(sp.adj_f2p(fid)[0]).color = Color::PASTEL_YELLOW();

}