#include "advancing_rationals.h"
#include <cinolib/predicates.h>

using namespace cinolib;

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool rationals_are_working()
{
    CGAL_Q a[3] = { 0,0,0 };
    CGAL_Q b[3] = { 1,2,3 };
    CGAL_Q c[3] = { 9,3,1 };
    CGAL_Q d[3];
    midpoint(a,b,c,d);

    double aa[3] = { 0,0,0 };
    double bb[3] = { 1,2,3 };
    double cc[3] = { 9,3,1 };
    double dd[3] = { (aa[0]+bb[0]+cc[0])/3,
                     (aa[1]+bb[1]+cc[1])/3,
                     (aa[2]+bb[2]+cc[2])/3 };

    if(orient3d(a,b,c,d)==0 && orient3d(aa,bb,cc,dd)!=0) return true;
    return false;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q orient3d(const CGAL_Q * pa,
                const CGAL_Q * pb,
                const CGAL_Q * pc,
                const CGAL_Q * pd)
{
    CGAL_Q adx = pa[0] - pd[0];
    CGAL_Q bdx = pb[0] - pd[0];
    CGAL_Q cdx = pc[0] - pd[0];
    CGAL_Q ady = pa[1] - pd[1];
    CGAL_Q bdy = pb[1] - pd[1];
    CGAL_Q cdy = pc[1] - pd[1];
    CGAL_Q adz = pa[2] - pd[2];
    CGAL_Q bdz = pb[2] - pd[2];
    CGAL_Q cdz = pc[2] - pd[2];

    return adx * (bdy * cdz - bdz * cdy)
           + bdx * (cdy * adz - cdz * ady)
           + cdx * (ady * bdz - adz * bdy);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
              CGAL_Q * res) // res = (pa + pb)/2
{
    res[0] = (pa[0] + pb[0])/2;
    res[1] = (pa[1] + pb[1])/2;
    res[2] = (pa[2] + pb[2])/2;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
              const CGAL_Q * pc,
              CGAL_Q * res) // res = (pa + pb + pc)/3
{
    res[0] = (pa[0] + pb[0] + pc[0])/3;
    res[1] = (pa[1] + pb[1] + pc[1])/3;
    res[2] = (pa[2] + pb[2] + pc[2])/3;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
              const CGAL_Q * pc,
              const CGAL_Q * pd,
              CGAL_Q * res) // res = (pa + pb + pc + pd)/3
{
    res[0] = (pa[0] + pb[0] + pc[0] + pd[0])/4;
    res[1] = (pa[1] + pb[1] + pc[1] + pd[1])/4;
    res[2] = (pa[2] + pb[2] + pc[2] + pd[2])/4;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void copy(const CGAL_Q * src, CGAL_Q * dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q sqrd_distance3d(const CGAL_Q * pa,
                       const CGAL_Q * pb)
{
    return (pa[0]-pb[0])*(pa[0]-pb[0]) +
           (pa[1]-pb[1])*(pa[1]-pb[1]) +
           (pa[2]-pb[2])*(pa[2]-pb[2]);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool point_in_tet(const CGAL_Q * pa,
                  const CGAL_Q * pb,
                  const CGAL_Q * pc,
                  const CGAL_Q * pd,
                  const CGAL_Q * q) // true if q is inside tet abcd
{
    // according to refrence tet as in cinolib/standard_elements_tables.h
    CGAL_Q A = orient3d(pa,pc,pb,q);
    CGAL_Q B = orient3d(pa,pb,pd,q);
    CGAL_Q C = orient3d(pa,pd,pc,q);
    CGAL_Q D = orient3d(pb,pc,pd,q);

    if((A>=0 && B>=0 && C>=0 && D>=0) || (A<=0 && B<=0 && C<=0 && D<=0)) return true;
    return false;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void triangle_normal(const CGAL_Q * pa,
                     const CGAL_Q * pb,
                     const CGAL_Q * pc,
                     CGAL_Q * n) // n is the normal of triangle abc
{
    CGAL_Q v0[3] = { pb[0]-pa[0], pb[1]-pa[1], pb[2]-pa[2] };
    CGAL_Q v1[3] = { pc[0]-pa[0], pc[1]-pa[1], pc[2]-pa[2] };
    cross(v0,v1,n);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q dot(const CGAL_Q * pa,
           const CGAL_Q * pb)
{
    return pa[0]*pb[0] +
           pa[1]*pb[1] +
           pa[2]*pb[2];
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void cross(const CGAL_Q * va,
           const CGAL_Q * vb,
           CGAL_Q * res)
{
    res[0] = va[1] * vb[2] - va[2] * vb[1];
    res[1] = va[2] * vb[0] - va[0] * vb[2];
    res[2] = va[0] * vb[1] - va[1] * vb[0];
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void plane_line_intersection(const CGAL_Q * p0,
                             const CGAL_Q * p1,
                             const CGAL_Q * p2,
                             const CGAL_Q * l0,
                             const CGAL_Q * l1,
                             CGAL_Q * res)
{
    // https://en.wikipedia.org/wiki/Line–plane_intersection

    CGAL_Q n[3];
    triangle_normal(p0,p1,p2,n);

    CGAL_Q l[3];
    l[0] = l1[0] - l0[0];
    l[1] = l1[1] - l0[1];
    l[2] = l1[2] - l0[2];

    CGAL_Q pl[3];
    pl[0] = p0[0] - l0[0];
    pl[1] = p0[1] - l0[1];
    pl[2] = p0[2] - l0[2];

    CGAL_Q d = dot(pl,n)/dot(l,n);

    res[0] = l0[0] + l[0]*d;
    res[1] = l0[1] + l[1]*d;
    res[2] = l0[2] + l[2]*d;

    assert(orient3d(p0,p1,p2,res)==0);
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::