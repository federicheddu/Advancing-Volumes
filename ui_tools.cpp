#include "ui_tools.h"

void showArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {

    for (auto vid : active_fronts) {
        dir_arrows.emplace_back(m.vert(vid),
                                m.vert(vid) + m.vert_data(vid).normal *
                                              oct.closest_point(m.vert(vid)).dist(m.vert(vid)));
        dir_arrows.back().size = 0.005;
    }

    for (auto &da: dir_arrows)
        gui.push(&da, false);

}

void deleteArrows(std::vector<DrawableArrow> &dir_arrows, GLcanvas &gui) {


    for (auto &id: dir_arrows) {
        gui.pop(&id);
    }
    dir_arrows.clear();

}

void updateArrows(Tetmesh<> &m, Octree &oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {
    deleteArrows(dir_arrows, gui);
    showArrows(m, oct, dir_arrows, active_fronts, gui);
}

void showFronts(DrawableTetmesh<> &m, std::vector<bool> &active_mask) {

    uint pid = 0;
    for (uint fid : m.get_surface_faces())
        //if one of the three verts are not active the poly is not active (true in the mask is inactive)
        if(active_mask[m.face_verts_id(fid)[0]] || active_mask[m.face_verts_id(fid)[1]] || active_mask[m.face_verts_id(fid)[2]])
            m.poly_data(m.adj_f2p(fid)[0]).color = Color::PASTEL_YELLOW();
        else
            m.poly_data(m.adj_f2p(fid)[0]).color = Color::PASTEL_ORANGE();

    m.updateGL();

}

void deleteFronts(DrawableTetmesh<> &m) {

    for (uint fid : m.get_surface_faces())
            m.poly_data(m.adj_f2p(fid)[0]).color = Color::PASTEL_YELLOW();

    m.updateGL();

}