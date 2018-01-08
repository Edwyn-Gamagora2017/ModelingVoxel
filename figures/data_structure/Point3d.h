#ifndef POINT3D_H
#define POINT3D_H

#include "vec3.h"
class FigureFace;
#include "Edge.h"
class Edge;
#include <deque>
#include <string>

#pragma once
class Point3d
{
	// coordonnees du point3d
	double x = 0;
	double y = 0;
	double z = 0;
	// identifiant du point
	int index;

	// Faces that contain the point
	std::deque<FigureFace *> faces;
	// Edges that contain the point
	std::deque<Edge *> edges;

public:
	Point3d(double x, double y, double z, int index);
	~Point3d();

	static Point3d * zero(int index);

	double getX();
	double getY();
	double getZ();
	int getIndex();
	int getEdgesCount();

	void set(double x, double y, double z);
	void setX(double x);
	void setY(double y);
	void setZ(double z);

	std::string toString();

	vec3 vectorFrom( Point3d origin );
	vec3 toVector();
	static Point3d fromVector(vec3 vect, int index);

	// Faces
	bool isAssociatedTo(FigureFace * f);
	void associateTo(FigureFace * f);
	void disassociateTo(FigureFace * f);

	// Edges
	bool isAssociatedTo(Edge * e);
	void associateTo(Edge * e);

	// Color
	Point3d add( double x, double y, double z );
};

#endif
