//
// Created by Federico Meloni on 06/03/23.
//

#include "sphere_util.h"

/// Translate the mesh to the origin (centroid)
/// @param m a tet mesh
void trasl_center(DrawableTetmesh<> &m) {
    vec3d centroid;

    centroid = m.centroid();
    std::cout << std::endl << TXT_BOLDWHITE << "CENTROID" << TXT_RESET << std::endl;
    std::cout << TXT_BOLDMAGENTA << "PRIMA\t" << TXT_RESET << centroid << std::endl;

    m.translate(-centroid);
    //sphere.center_bbox();
    m.updateGL();

    centroid = m.centroid();
    std::cout << TXT_BOLDGREEN << "DOPO\t" << TXT_RESET << centroid << std::endl;
}

/// Scale the mesh to the unitary cube
/// @param m a tet mesh
void scale_unicube(DrawableTetmesh<> &m) {
    vec3d centroid;
    AABB bbox = m.bbox();
    double deltaX = bbox.delta_x(), deltaY = bbox.delta_y(), deltaZ = bbox.delta_z();

    std::cout << std::endl << TXT_BOLDWHITE << "BBOX" << TXT_RESET << std::endl;
    std::cout << TXT_BOLDMAGENTA << "PRIMA\t" << TXT_RESET << bbox << std::endl;
    std::cout << "Size X = " << deltaX << " / Size Y = " << deltaY << " / Size Z = " << deltaZ << std::endl;

    deltaX = 1.0/deltaX;
    deltaY = 1.0/deltaY;
    deltaZ = 1.0/deltaZ;

    for(uint vid = 0; vid < m.num_verts(); vid++) {
        m.vert(vid).x() *= deltaX;
        m.vert(vid).y() *= deltaY;
        m.vert(vid).z() *= deltaZ;
    }

    m.updateGL();
    m.update_bbox();

    bbox = m.bbox();
    deltaX = bbox.delta_x();
    deltaY = bbox.delta_y();
    deltaZ = bbox.delta_z();

    std::cout << TXT_BOLDGREEN << "DOPO\t" << TXT_RESET << bbox << std::endl;
    std::cout << "Size X = " << deltaX << " / Size Y = " << deltaY << " / Size Z = " << deltaZ << std::endl;

    centroid = m.centroid();
    std::cout << std::endl << TXT_BOLDWHITE << "CENTROID" << TXT_RESET << std::endl;
    std::cout << TXT_BOLDMAGENTA << "PRIMA\t" << TXT_RESET << centroid << std::endl;
}

/// Transform the sphere mesh to have the centroid in the origin and the bounding box in the unitary cube
/// @param m a tet mesh
void mesh_standardizer(DrawableTetmesh<> &m) {
    trasl_center(m);
    scale_unicube(m);
}

/// Get a quadmesh representing the bounding box of the mesh
/// @param m a tet mesh
/// @return a quad mesh representing the bounding box of the mesh
DrawableQuadmesh<> get_bbox_mesh(DrawableTetmesh<> &m) {
    AABB bbox = m.bbox();
    DrawableQuadmesh<> m_bbox;

    m_bbox.vert_add(vec3d(bbox.min.x(), bbox.min.y(), bbox.min.z())); //0
    m_bbox.vert_add(vec3d(bbox.max.x(), bbox.min.y(), bbox.min.z())); //1
    m_bbox.vert_add(vec3d(bbox.max.x(), bbox.min.y(), bbox.max.z())); //2
    m_bbox.vert_add(vec3d(bbox.min.x(), bbox.min.y(), bbox.max.z())); //3
    m_bbox.vert_add(vec3d(bbox.min.x(), bbox.max.y(), bbox.min.z())); //4
    m_bbox.vert_add(vec3d(bbox.max.x(), bbox.max.y(), bbox.min.z())); //5
    m_bbox.vert_add(vec3d(bbox.max.x(), bbox.max.y(), bbox.max.z())); //6
    m_bbox.vert_add(vec3d(bbox.min.x(), bbox.max.y(), bbox.max.z())); //7

    m_bbox.poly_add({0, 1, 2, 3});
    m_bbox.poly_add({4, 5, 6, 7}); //idk why the face is not visible
    m_bbox.poly_add({4, 5, 1, 0});
    m_bbox.poly_add({5, 6, 2, 1});
    m_bbox.poly_add({6, 7, 3, 2});
    m_bbox.poly_add({7, 4, 0, 3});

    return m_bbox;
}

/// get an hardcoded sphere
/// @param center: center of the sphere (default: (0,0,0))
/// @param scale_factor: scale factor of the sphere (default: 1.0)
/// @param num_polys: number of polygons of the sphere (default: 18)
DrawableTetmesh<> get_sphere(const vec3d &center, const double &scale_factor, const int &num_polys) {

    DrawableTetmesh<> sphere;
    double coeff = 1.0;

    switch (num_polys) {
        case 31:
            sphere = get_sphere31();
            break;
        case 40:
            sphere = get_sphere40();
            break;
        case 50:
            sphere = get_sphere50();
            break;
        default:
            sphere = get_sphere18();
    }

    //scale the sphere to the desired size
    coeff = scale_factor / sqrt(pow(sphere.bbox().delta().x(),2)
           + pow(sphere.bbox().delta().y(),2)
           + pow(sphere.bbox().delta().z(),2));
    sphere.scale(coeff);
    sphere.translate(center);

    sphere.updateGL();

    return sphere;
}

/// hardcoded sphere with 18 tetrahedrons centered in (0,0,0) and with bbox edge length 1
/// @return a DrawableTetmesh<> sphere
DrawableTetmesh<> get_sphere18() {

    std::vector<vec3d> vertices{ vec3d( 0.14304169836480085,   0.35718176282666159,  -0.36952151508954673),
                                 vec3d(-0.017028116396762463,  0.51049279738420339,   0.20000768969462634),
                                 vec3d( 0.44216045989897851,  -0.2601061833226288,    0.10731053352960375),
                                 vec3d(-0.0060954450539385217,-0.48950720261579656,   0.22434768573992619),
                                 vec3d(-0.4558765004289107,    0.3395051173188085,   -0.12984302882705945),
                                 vec3d( 0.46727084220577753,  -0.10346701477224393,  -0.16184229872858066),
                                 vec3d( 0.1423428227537398,    0.0067184098359288026, 0.54327227772134923),
                                 vec3d( 0.46509959584120469,   0.17795132082711212,   0.15279634477548637),
                                 vec3d(-0.53272915779422247,  -0.0018696772794382361, 0.37150309024321415),
                                 vec3d(-0.51141345165685703,  -0.32734283116601476,  -0.12680940204596564),
                                 vec3d( 0.034251881564850303, -0.36784597162327864,  -0.3544936547344012),
                                 vec3d(-0.17102462929866138,   0.15828947258668569,  -0.45672772227865072) };

    std::vector<uint> polys{ 12, 1,   6, 11,
                              2, 12,  1,  8,
                              5, 10,  4,  9,
                             12, 10,  4,  5,
                              4,  9,  7,  5,
                              7,  9,  2,  5,
                              2, 12,  8,  5,
                              2,  5,  8,  7,
                              7,  4,  8,  3,
                              4,  5,  8, 12,
                              5,  8,  7,  4,
                              2,  1, 12,  5,
                              4, 12, 11, 10,
                             12,  8,  6,  1,
                              8,  4,  6,  3,
                             12,  4,  6,  8,
                             11,  4,  6, 12,
                              4,  6,  3, 11 };

    for(uint &poly : polys)
        poly -= 1;

    return DrawableTetmesh<>(vertices, polys);
}

/// hardcoded sphere with 31 tetrahedrons centered in (0,0,0) and with bbox edge length 1
/// @return a DrawableTetmesh<> sphere
DrawableTetmesh<> get_sphere31() {

    std::vector<vec3d> vertices{ vec3d( -0.27042741119466335, -0.05297100248550133, 0.44688966130115998),
                                 vec3d(  0.041786607692406684, 0.47405131604104261,-0.17254724621232284),
                                 vec3d( -0.21125276543526514,  0.40540039513096665, 0.19720286265445772),
                                 vec3d(  0.44355431021500435, -0.28757641960359448, 0.055744717744398417),
                                 vec3d(  0.28999790225014405, -0.10812506532407123, 0.37336606233288211),
                                 vec3d(  0.017798334754711834,-0.52594868395895744, 0.17173138575096394),
                                 vec3d( -0.40940623714009722,  0.33548403543432537,-0.17928023787145256),
                                 vec3d(  0.18953480083629307,  0.4203932445350837,  0.16012283719637987),
                                 vec3d( -0.3161740174512862,  -0.46428021158625971,-0.011483031560529034),
                                 vec3d(  0.46740429232362829, -0.12481152252883829,-0.21099229361124422),
                                 vec3d(  0.058843052822927226, 0.020041348543751142,0.49676907040577251),
                                 vec3d(  0.46534203033510985,  0.16761231279077171, 0.10082226438962669),
                                 vec3d( -0.53259570767637177,  0.020576993180804234,0.18275205369053454),
                                 vec3d( -0.44389078400426624, -0.20455496781594554,-0.29131587918053375),
                                 vec3d(  0.056120452125595729,-0.3995296157032695, -0.40191447898624844),
                                 vec3d(  0.29221747005673043,  0.17705629978968829,-0.41463681844961892),
                                 vec3d( -0.13885233051060769,  0.14718154356000243,-0.50323092959422755)};

    std::vector<uint> polys{  3,  9,  1, 13,
                             14,  7,  3, 17,
                              2,  5, 10,  8,
                              6,  9,  5,  4,
                             14,  7, 13,  3,
                              3,  9, 13, 14,
                              9,  6, 15,  4,
                             15, 17, 10, 16,
                             12,  5, 11,  8,
                             12,  8, 10,  5,
                             16,  8, 10, 12,
                              1,  9,  5,  6,
                              1,  3,  5,  9,
                             12,  5, 10,  4,
                             17, 10, 16,  2,
                              1,  5, 11,  6,
                              5, 11,  8,  3,
                              1,  3, 11,  5,
                              3,  7,  2, 17,
                              8,  3,  2,  5,
                              3,  2,  5,  9,
                              3, 14,  2,  9,
                             17, 14,  2,  3,
                             17,  9,  2, 14,
                              9, 17, 15, 14,
                             16,  8,  2, 10,
                             15,  4, 10,  9,
                              5,  9, 10,  4,
                             17, 10,  2,  9,
                             10,  2,  9,  5,
                             15,  9, 10, 17 };

    for(uint &poly : polys)
        poly -= 1;

    return DrawableTetmesh<>(vertices, polys);
}

/// hardcoded sphere with 40 tetrahedrons centered in (0,0,0) and with bbox edge length 1
/// @return a DrawableTetmesh<> sphere
DrawableTetmesh<> get_sphere40() {

    std::vector<vec3d> vertices{ vec3d(-0.43043363389912598,    0.19355106864966035,   0.22632420271245421),
                                 vec3d(-0.2419734283867789,    -0.060966807840047106,  0.50775299267726992),
                                 vec3d( 0.075764985815070796,    0.46605551068649681,  -0.12087549813570923),
                                 vec3d(-0.18175173119467353,    0.3974045897764209,    0.25436118875770092),
                                 vec3d( 0.48464166961069854,    -0.29557222495814028,   0.11080400146485504),
                                 vec3d( 0.32836819498835956,    -0.11612087067861701,   0.43313840562578521),
                                 vec3d( 0.17756766088847442,    -0.45220234413473837,  -0.2300166264140564),
                                 vec3d( 0.051352258193697604,   -0.53394448931350325,   0.22851175051725761),
                                 vec3d(-0.46421773862211002,    0.13335431892051627,  -0.12618034405551617),
                                 vec3d( 0.2261274735664856,      0.41239743918053789,   0.21673094722658748),
                                 vec3d(-0.28852948688443825,   -0.47227601694080545,   0.04257868574068667),
                                 vec3d(-0.25425104069992743,    0.40469559736038629,  -0.11866245443574493),
                                 vec3d( 0.50891365944943057,    -0.13280732788338406,  -0.15989101663723049),
                                 vec3d( 0.093123232081561244,    0.012045543189205365,  0.55837254292967919),
                                 vec3d( 0.50681490726579592,     0.15961650743622594,   0.15655043628884838),
                                 vec3d( 0.022305739078867889,    0.23570404806461165,  -0.39916489354304868),
                                 vec3d(-0.49108634055056954,   -0.13613733871762027,   0.19008093787809258),
                                 vec3d( 0.31040653547879549,     0.28447544568107963,  -0.25005130083147525),
                                 vec3d(-0.41850610682029699,   -0.21255077317049131,  -0.24140649264559313),
                                 vec3d(-0.0059719998982850509, -0.32342401260711107,  -0.4042708714806007),
                                 vec3d( 0.19278845208110396,    -0.017586526149906224, -0.44162745707032075),
                                 vec3d(-0.20145326154213755,    0.05428866344922699,  -0.43305913656992134) };

    std::vector<uint> polys{ 19, 11, 20, 22,
                             11, 19,  9, 22,
                              9,  1, 17,  4,
                              1, 12,  4,  9,
                             11, 19, 17,  9,
                             15,  6, 13,  5,
                             20, 11, 21, 22,
                              2, 11,  6,  8,
                              4,  6, 18, 10,
                              2, 17,  6, 11,
                             16,  4,  3, 12,
                              2,  4,  6,  1,
                             17,  1,  6,  4,
                              2,  1,  6, 17,
                             17,  9,  6, 11,
                              4,  9,  6, 17,
                             10, 15, 14,  6,
                              2,  6, 14,  8,
                             10,  6, 14,  4,
                              2,  4, 14,  6,
                              9, 12, 16, 22,
                             16,  4, 12,  9,
                              3,  4, 18, 10,
                             16,  6, 18,  4,
                             16,  4,  9,  6,
                              3, 16, 18,  4,
                             21,  6, 18, 16,
                              6, 18, 10, 15,
                              9, 22, 21, 11,
                             16, 22, 21,  9,
                             21,  6, 13, 18,
                              6, 13, 18, 15,
                              9,  6, 21, 16,
                              9, 11, 21,  6,
                             13,  6,  7,  5,
                             21,  6,  7, 13,
                             21, 20,  7, 11,
                              6,  7,  5,  8,
                              6, 11,  7,  8,
                             21, 11,  7,  6 };

    for(uint &poly : polys)
        poly -= 1;

    return DrawableTetmesh<>(vertices, polys);
}

/// hardcoded sphere with 50 tetrahedrons centered in (0,0,0) and with bbox edge length 1
/// @return a DrawableTetmesh<> sphere
DrawableTetmesh<> get_sphere50() {

    std::vector<vec3d> vertices{ vec3d( -0.44230122806181132,     0.22278160183641008,     0.17574279332564813),
                                 vec3d( -0.27880966441226945,     0.087989635158904317,    0.41832535177134594),
                                 vec3d(  0.063897391652385463,    0.49324470673998516,    -0.1780979239593119),
                                 vec3d( -0.19361932535735885,     0.42510805148018216,     0.2043160531197197),
                                 vec3d( -0.012009531415064919,   -0.073454477230532514,    0.4950149763608056),
                                 vec3d(  0.4727740754480132,     -0.26267765776694757,     0.058012994680950822),
                                 vec3d(  0.31650060082567422,    -0.084570577769218683,    0.3865128075992863),
                                 vec3d(  0.16570006672578907,    -0.41813445706636321,    -0.28932663475877418),
                                 vec3d(  0.08373965978249058,    -0.50675529326001489,    -0.0056484706535095824),
                                 vec3d( -0.47608533278479537,     0.16303578736397326,    -0.183504237619045),
                                 vec3d(  0.21425987940380026,     0.43998858909960581,     0.16596604411208404),
                                 vec3d( -0.30039708104712359,    -0.43805775752108927,    -0.011517291761864515),
                                 vec3d( -0.26611863486261278,     0.43234444195622523,    -0.17584255055816722),
                                 vec3d(  0.49704606528674522,    -0.10113203633374315,    -0.21785970663970713),
                                 vec3d( -0.22204132056171891,    -0.24240509738197027,     0.37535200051512713),
                                 vec3d(  0.12666644271029387,     0.18492216949562876,     0.44811637462302489),
                                 vec3d(  0.3891723141146633,      0.21140914794990406,     0.23891749988459274),
                                 vec3d(  0.010438144916182553,    0.2646188122990677,     -0.46171026303443552),
                                 vec3d( -0.50295393471325489,    -0.10443710197912562,     0.13880629022840729),
                                 vec3d( -0.093414024116931296,   -0.42648744702384983,     0.24759774267501966),
                                 vec3d(  0.49469222587207373,     0.1559972534986637,     -0.052347865063232302),
                                 vec3d(  0.29853894131611014,     0.31302486223807618,    -0.30974451976591105),
                                 vec3d( -0.43037370098298228,    -0.18027812161166284,    -0.30093435915283751),
                                 vec3d( -0.017839594060970389,   -0.29032080702481328,    -0.46691390489084),
                                 vec3d(  0.13590587814067867,    -0.36285848150423539,     0.30448615885550501),
                                 vec3d(  0.18092085791841861,     0.013225643685664047,   -0.5049850236391944),
                                 vec3d( -0.21332085570482287,     0.084562414562547289,   -0.49625281371970881),
                                 vec3d( -0.00096831603160211013, -0.00068380389126907241, -0.0024815225349805589) };

    std::vector<uint> polys{ 12, 15, 20, 28,
                              5,  2, 16, 28,
                             22,  3, 18, 28,
                              4, 13,  3, 28,
                             15,  2,  5, 28,
                             12, 20,  9, 28,
                             27, 26, 18, 28,
                             15, 12, 19, 28,
                              5, 20, 15, 28,
                             25, 20,  5, 28,
                              3, 13, 18, 28,
                             11,  4,  3, 28,
                              2,  4, 16, 28,
                             17, 16, 11, 28,
                             16,  4, 11, 28,
                             20, 25,  9, 28,
                             13, 27, 18, 28,
                             13, 10, 27, 28,
                             10,  1, 19, 28,
                             10, 13,  1, 28,
                             13,  4,  1, 28,
                              4,  2,  1, 28,
                             12, 23, 19, 28,
                             23, 10, 19, 28,
                              7, 16, 17, 28,
                              7,  5, 16, 28,
                              5,  7, 25, 28,
                              2, 15, 19, 28,
                             11,  3, 22, 28,
                             26, 22, 18, 28,
                             25,  7,  6, 28,
                              6, 17, 21, 28,
                              6,  7, 17, 28,
                             17, 11, 21, 28,
                             21, 11, 22, 28,
                             27, 24, 26, 28,
                              1,  2, 19, 28,
                             10, 23, 27, 28,
                             12,  9,  8, 28,
                             25,  6,  9, 28,
                             21, 22, 14, 28,
                             22, 26, 14, 28,
                             26, 24,  8, 28,
                             23, 24, 27, 28,
                             24, 23, 12, 28,
                             24, 12,  8, 28,
                              9,  6,  8, 28,
                             14, 26,  8, 28,
                              6, 21, 14, 28,
                              6, 14,  8, 28 };

    for(uint &poly : polys)
        poly -= 1;

    return DrawableTetmesh<>(vertices, polys);
}

