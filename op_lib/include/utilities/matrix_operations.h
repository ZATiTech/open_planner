
/// \file matrix_operations.h
/// \brief Simple matrix operations
/// \author Hatem Darweesh
/// \date Jun 19, 2016


#ifndef MATRIXOPERATIONS_H_
#define MATRIXOPERATIONS_H_

#include "planning/RoadNetwork.h"
#include <math.h>
#include <iostream>

namespace op {
	namespace utilities{

class Vector
{
public:
	Vector(double val[3])
	{
		v[0] = val[0]; v[1] = val[1]; v[2] = val[2];
	}
  Vector(double a = 0.0, double b = 0.0, double c = 0.0)
	{
    v[0] = a; v[1] = b; v[2] = c;
	}
  WayPoint GetWayPoint()
  {
	  return WayPoint(v[0], v[1], v[2], 0);
  }

  void GetArr(float* val)
  {
	  val[0] = v[0];
	  val[1] = v[1];
	  val[2] = v[2];
  }

  double &operator[](int i) { return v[i]; }
  double operator[](int i) const { return v[i]; }
  Vector operator+(Vector const &w) const {
    return Vector(v[0] + w[0], v[1] + w[1], v[2] + w[2]);
  }
  Vector operator-(Vector const &w) const {
    return Vector(v[0] - w[0], v[1] - w[1], v[2] - w[2]);
  }
  Vector operator*(double const t) const {
    return Vector(v[0] * t, v[1] * t, v[2] * t);
  }
  Vector operator/(double const t) const {
    return *this * (1.0 / t);
  }

  //Dot Product
  double operator*(Vector const &w) const {
    return v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
  }

  //Cross Product
  Vector operator^(Vector const &w) const {
    return Vector(v[1] * w[2] - v[2] * w[1],
		  v[2] * w[0] - v[0] * w[2],
		  v[0] * w[1] - v[1] * w[0]);
  }
  double length() const {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }
  Vector normalized() const { return *this / length(); }
  friend std::ostream &operator<<(std::ostream &os, const Vector &v) {
    os << "<" << v[0] << ", " << v[1] << ", " << v[2] << ">";
    return os;
  }
private:
  double v[3];
};

class Point {
public:
	Point(double val[3])
{
		p[0] = val[0]; p[1] = val[1]; p[2] = val[2];
}
  Point(double a = 0.0, double b = 0.0, double c = 0.0)
{
    p[0] = a; p[1] = b; p[2] = c;
  }

  WayPoint GetWayPoint()
  {
	  return WayPoint(p[0], p[1], p[2], 0);
  }

  void GetArr(float* val)
  {
	  val[0] = p[0];
	  val[1] = p[1];
	  val[2] = p[2];
  }

  double &operator[](int i) { return p[i]; }
  double operator[](int i) const { return p[i]; }
  Point operator+(Vector const &v) const {
    return Point(p[0] + v[0], p[1] + v[1], p[2] + v[2]);
  }
  Vector operator-(Point const &q) const {
    return Vector(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
  }
  friend std::ostream &operator<<(std::ostream &os, const Point &p) {
    os << "<" << p[0] << ", " << p[1] << ", " << p[2] << ">";
    return os;
  }
private:
  double p[3];
};

class Mat3
{
	double m[3][3];

public:
	Mat3()
	{
		//initialize Identity by default
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				m[i][j] = 0;

		m[0][0] = m[1][1] = m[2][2] = 1;
	}

	Mat3(double transX, double transY, bool mirrorX, bool mirrorY )
	{
		m[0][0] = (mirrorX == true ) ? -1 : 1; m[0][1] =  0; m[0][2] =  transX;
		m[1][0] = 0; m[1][1] =  (mirrorY==true) ? -1 : 1; m[1][2] =  transY;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(double transX, double transY)
	{
		m[0][0] = 1; m[0][1] =  0; m[0][2] =  transX;
		m[1][0] = 0; m[1][1] =  1; m[1][2] =  transY;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	/**
	 * x axis -> 0
	 * y axis -> 1
	 * z axis -> 2
	 */
	Mat3(double rotation_angle, int rot_axis)
	{
		double c = cos(rotation_angle);
		double s = sin(rotation_angle);
		if(rot_axis == 0)
		{
			m[0][0] = 1; m[0][1] = 0; m[0][2] =  0;
			m[1][0] = 0; m[1][1] = c; m[1][2] = -s;
			m[2][0] = 0; m[2][1] = s; m[2][2] =  c;
		}
		else if(rot_axis == 1)
		{
			m[0][0] =  c; m[0][1] = 0; m[0][2] =  s;
			m[1][0] =  0; m[1][1] = 1; m[1][2] =  0;
			m[2][0] = -s; m[2][1] = 0; m[2][2] =  c;
		}
		else if(rot_axis == 2)
		{
			m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
			m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
			m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
		}
	}

	Mat3(double rotation_angle)
	{
		double c = cos(rotation_angle);
		double s = sin(rotation_angle);
		m[0][0] = c; m[0][1] = -s; m[0][2] =  0;
		m[1][0] = s; m[1][1] =  c; m[1][2] =  0;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}

	Mat3(GPSPoint rotationCenter)
	{
		double c = cos(rotationCenter.a);
		double s = sin(rotationCenter.a);
		double u = rotationCenter.x;
		double v = rotationCenter.y;
		m[0][0] = c; m[0][1] = -s; m[0][2] = -u*c + v*s + u;
		m[1][0] = s; m[1][1] =  c; m[1][2] = -u*s - v*c + v;
		m[2][0] = 0; m[2][1] =  0; m[2][2] =  1;
	}


	GPSPoint operator * (GPSPoint v)
	{
		GPSPoint _v = v;
		v.x = m[0][0]*_v.x + m[0][1]*_v.y + m[0][2]*1;
		v.y = m[1][0]*_v.x + m[1][1]*_v.y + m[1][2]*1;
		return v;
	}

	GPSPoint Mul3D (GPSPoint v)
	{
		GPSPoint _v = v;
		v.x = m[0][0]*_v.x + m[0][1]*_v.y + m[0][2]*_v.z;
		v.y = m[1][0]*_v.x + m[1][1]*_v.y + m[1][2]*_v.z;
		v.z = m[2][0]*_v.x + m[2][1]*_v.y + m[2][2]*_v.z;
		return v;
	}

	static Point rotatePoint(const Point &p, const Point &center, const Vector &axis,double theta)
	{
		double const c = cos(theta), s = sin(theta);
		double const C = 1.0 - c;
		Vector tmp = p - center;
		return center +	Vector(tmp[0] * (axis[0] * axis[0] * C + c) +
		   tmp[1] * (axis[0] * axis[1] * C - axis[2] * s) +
		   tmp[2] * (axis[0] * axis[2] * C + axis[1] * s),
		   tmp[0] * (axis[1] * axis[0] * C + axis[2] * s) +
		   tmp[1] * (axis[1] * axis[1] * C + c) +
		   tmp[2] * (axis[1] * axis[2] * C - axis[0] * s),
		   tmp[0] * (axis[2] * axis[0] * C - axis[1] * s) +
		   tmp[1] * (axis[2] * axis[1] * C + axis[0] * s) +
		   tmp[2] * (axis[2] * axis[2] * C + c));
	}
};

 /* namespace utilities */	}
} /* namespace op */

#endif /* MATRIXOPERATIONS_H_ */
