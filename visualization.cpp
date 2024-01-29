#include "visualization.h"

void UI_Manager(Data &d, UI_Mode uiMode, std::vector<DrawableArrow> &dir_arrows) {
    static UI_Mode prev = BLANK;

    if(prev == BLANK && uiMode == BLANK)
        return;

    switch (uiMode) {
        case BLANK: {
            clearColors(d.m);
            deleteArrows(dir_arrows, *d.gui);
            break;
        }
        case HIGHLIGHT: {
            highlightModel(d.m);
            break;
        }
        case DIRECTION: {
            if (prev != DIRECTION)
                showArrows(d.m, d.oct, dir_arrows, d.fronts_active, *d.gui);
            else
                deleteArrows(dir_arrows, *d.gui);
            break;
        }
        case FRONTS: {
            showFronts(d);
            break;
        }
        case VOLUME: {
            showVolume(d.m);
            break;
        }
    }

    prev = uiMode;
}

void showArrows(Tetmesh<> &m, Octree *oct, std::vector<DrawableArrow> &dir_arrows, std::vector<uint> &active_fronts, GLcanvas &gui) {

    deleteArrows(dir_arrows, gui);

    for (auto vid : active_fronts) {
        dir_arrows.emplace_back(m.vert(vid),
                                m.vert(vid) + m.vert_data(vid).normal *
                                              oct->closest_point(m.vert(vid)).dist(m.vert(vid)));
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

void showFronts(Data &d) {

    d.gui->pop_all_markers();
    for(auto vid : d.fronts_active)
        d.gui->push_marker(d.m.vert(vid), "", Color::RED(), 2, 4);

}

void showVolume(DrawableTetmesh<> &m) {

    PARALLEL_FOR(0,m.num_polys(),1000,[&](uint pid)
    {
        double volume = orient3d(m.poly_vert(pid, 0), m.poly_vert(pid, 1), m.poly_vert(pid, 2), m.poly_vert(pid, 3));
        m.poly_data(pid).color = volume > 0 ? Color::RED() : Color::PASTEL_CYAN();
    });

}

void clearColors(DrawableTetmesh<> &m) {

    PARALLEL_FOR(0, m.num_polys(), 100, [&](uint pid)
    {
        m.poly_data(pid).color = Color::WHITE();
    });

}

void highlightModel(DrawableTetmesh<> &m) {

    PARALLEL_FOR(0, m.num_polys(), 100, [&](uint pid)
    {
        m.poly_data(pid).color = Color(255.f/255.f, 198.f/255.f, 70.f/255.f);
    });

}

void show_stuck_v(DrawableTetmesh<> &m, std::set<uint> &stuck, GLcanvas &gui) {
    //push markers
    for(uint vid : stuck)
        gui.push_marker(m.vert(vid), std::to_string(vid));
}

void show_stuck_e(DrawableTetmesh<> &m, std::set<uint> &stuck, GLcanvas &gui) {

    for(uint eid = 0; eid < m.num_edges(); eid++)
        if(stuck.find(m.edge_vert_id(eid, 0)) != stuck.end() || stuck.find(m.edge_vert_id(eid, 1)) != stuck.end())
            m.edge_data(eid).flags[MARKED] = true;
        else
            m.edge_data(eid).flags[MARKED] = false;

    m.show_marked_edge(true);

}