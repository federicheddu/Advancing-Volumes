#undef NDEBUG

#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Gmpq.h>

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

typedef CGAL::Lazy_exact_nt<CGAL::Gmpq> CGAL_Q;

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q orient3d(const CGAL_Q * pa,
                const CGAL_Q * pb,
                const CGAL_Q * pc,
                const CGAL_Q * pd);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
              const CGAL_Q * pc,
              const CGAL_Q * pd,
                    CGAL_Q * res); // res = (pa + pb + pc + pd)/4

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
              const CGAL_Q * pc,
                    CGAL_Q * res); // res = (pa + pb + pc)/3

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void midpoint(const CGAL_Q * pa,
              const CGAL_Q * pb,
                    CGAL_Q * res); // res = (pa + pb)/2

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q sqrd_distance3d(const CGAL_Q * pa,
                       const CGAL_Q * pb);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool point_in_tet(const CGAL_Q * pa,
                  const CGAL_Q * pb,
                  const CGAL_Q * pc,
                  const CGAL_Q * pd,
                  const CGAL_Q * q); // true if q is inside tet abcd

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void triangle_normal(const CGAL_Q * pa,
                     const CGAL_Q * pb,
                     const CGAL_Q * pc,
                           CGAL_Q * n); // n is the normal of triangle abc

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

CGAL_Q dot(const CGAL_Q * pa,
           const CGAL_Q * pb);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void cross(const CGAL_Q * va,
           const CGAL_Q * vb,
                 CGAL_Q * res);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void plane_line_intersection(const CGAL_Q * p0,
                             const CGAL_Q * p1,
                             const CGAL_Q * p2,
                             const CGAL_Q * l0,
                             const CGAL_Q * l1,
                                   CGAL_Q * res);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void copy(const CGAL_Q * src, CGAL_Q * dst);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool rationals_are_working();

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


