#ifndef SUMMERCAMP_CAMERA_H
#define SUMMERCAMP_CAMERA_H
#include <GSLAM/core/Point.h>

namespace summercamp {

class CameraOpenCV {
 public:
  typedef GSLAM::Point2d Point2d;
  typedef GSLAM::Point3d Point3d;
  CameraOpenCV(double Fx, double Fy, double Cx, double Cy, double K1, double K2,
               double P1, double P2, double K3);

  virtual std::string CameraType() const { return "OpenCV"; }
  virtual bool isValid() const;
  virtual Point2d Project(const Point3d& p3d) const;
  virtual Point3d UnProject(const Point2d& p2d) const;
  virtual bool applyScale(double scale = 0.5);
  double fx, fy, cx, cy, k1, k2, p1, p2, k3;
};

CameraOpenCV::CameraOpenCV(double Fx, double Fy, double Cx, double Cy, double K1, double K2, double P1, double P2, double K3)
    :fx(Fx), fy(Fy), cx(Cx), cy(Cy), k1(K1), k2(K2), p1(P1), p2(P2), k3(K3){}

bool CameraOpenCV::isValid() const{
    return fx!=0&&fy!=0;
}

GSLAM::Point2d CameraOpenCV::Project(const Point3d &p3d) const{
    double x = p3d.x, y = p3d.y;
    if(p3d.z != 1.0)
    {
        double z_inv = 1.0/p3d.z;
        x *= z_inv;
        y *= z_inv;
    }
    double r2, r4, r6, x2, y2, xy, x_d, y_d;
    x2 = x*x;
    y2 = y*y;
    xy = x*y;
    r2 = x2+y2;
    r4 = r2*r2;
    r6 = r2*r4;

    x_d = x * (1 + k1*r2 + k2*r4 + k3*r6) + 2*p1*xy + p2*(r2 + 2*x2);
    y_d = y * (1 + k1*r2 + k2*r4 + k3*r6) + p1*(r2 + 2*y2) + 2*p2*xy;

    return Point2d(fx*x_d + cx, fy*y_d + cy);
}

GSLAM::Point3d CameraOpenCV::UnProject(const Point2d &p2d) const{
    double x_d, y_d, x0, y0, xn, yn;
    x_d = (p2d.x - cx)/fx;
    y_d = (p2d.y - cy)/fy;

    //迭代
    x0 = x_d;
    y0 = y_d;
    double r2, r4, r6, x2, y2, xy,k;

    for(int i = 0; i < 5; i++)
    {
        x2 = x0*x0;
        y2 = y0*y0;
        xy = x0*y0;
        r2 = x2+y2;
        r4 = r2*r2;
        r6 = r2*r4;
        k = 1/(1.0 + k1*r2 + k2*r4 + k3*r6);
        xn = (x_d - 2*p1*xy - p2*(r2 + 2*x2))*k;
        yn = (y_d- p1*(r2 + 2*y2) - 2*p2*xy)*k;
        x0 = xn;
        y0 = yn;
    }

    return CameraOpenCV::Point3d(x0, y0, 1);

}


bool CameraOpenCV::applyScale(double scale)
{
    fx *= scale;
    fy *= scale;
    cx *= scale;
    cy *= scale;
    return true;
}


}

#endif
