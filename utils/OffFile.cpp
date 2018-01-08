#include "OffFile.h"

#include <iostream>
#include <fstream>
#include <deque>
#include <climits>  // INT_MAX
#include "utils.h"
#include "../figures/data_structure/vec3.h"
#include "../figures/data_structure/FigureFace.h"

Figure * OffFile::readFile( string filename )
{
    std::ifstream reader;
	reader.open( string(filename+".off").c_str() );
	if( !reader.is_open() ){ PRINT("FILE ERROR"); reader.close(); return NULL; }

	string type;
	reader >> type;
	if( type != "OFF" ){ reader.close(); return NULL; }

	int nPoints, nFaces, nEdges;
	reader >> nPoints >> nFaces >> nEdges;
	std::deque<Point3d*> points;
	std::deque<FigureFace*> faces;

	double x,y,z;
	int pPerFace, pIndex;
	FOR( i, nPoints )
	{
        reader >> x >> y >> z;
	    points.push_back( new Point3d(x,y,z, i) );
	}
	FOR( i, nFaces )
	{
	    reader >> pPerFace;
	    std::deque<Point3d*> pointsFace;

	    FOR(j,pPerFace)
	    {
            reader >> pIndex;
            pointsFace.push_back( points[ pIndex ] );
	    }

	    if( pPerFace >= 3 )
        {
            // Normal from the fist points
            vec3 normal = pointsFace[1]->toVector().soustraction( pointsFace[0]->toVector() ).produitVectoriel( pointsFace[2]->toVector().soustraction( pointsFace[0]->toVector() ) );
            faces.push_back( new FigureFace( pointsFace, normal, i ) );
        }
	}

	reader.close();

	Figure * resultat = new Figure( new vec3(0,0,0), new vec3(1,1,1), new vec3(0,0,0), new Point3d(1,1,1,-1), false, false );
	resultat->setPoints( points );
	resultat->setFaces( faces );

	resultat->centralizeFigure();
	//resultat->normalizeFigure();

	return resultat;
}

void OffFile::writeFile( Figure * f, string filename )
{
    std::ofstream writer;
	writer.open( string(filename+".off").c_str() );
	if( !writer.is_open() ){ PRINT("FILE ERROR"); writer.close(); return; }

	writer << "OFF" << std::endl;
	writer << f->getPoints().size() << " " << f->getFaces().size() << " " << 0 << std::endl;

	FOR(i,f->getPoints().size())
	{
	    writer << f->getPoints()[i]->getX() << " " << f->getPoints()[i]->getY() << " " << f->getPoints()[i]->getZ() << std::endl;
	}
	FOR(i,f->getFaces().size())
	{
	    writer << f->getFaces()[i]->getPoints().size() << " ";
	    FOR(j,f->getFaces()[i]->getPoints().size())
	    {
	        writer << f->getFaces()[i]->getPoints()[j]->getIndex() << " ";
	    }
	    writer << std::endl;
	}

	writer.close();
}

void OffFile::printInfo( Figure * f )
{
    std::deque<Point3d*> points = f->getPoints();
	cout << "Vertices: " << points.size() << std::endl;

	std::deque<Edge*> edges = f->getEdges();
	cout << "Edges: " << edges.size() << std::endl;

    std::deque<FigureFace*> faces = f->getFaces();
	cout << "Faces: " << faces.size() << std::endl;

	// Edges per Face
	int maxEdgesPerFace = 0;
	int minEdgesPerFace = INT_MAX;
    FOR(i,faces.size()){
        int countEdges = faces[i]->getEdgesCount();
        if( countEdges > maxEdgesPerFace ){
            maxEdgesPerFace = countEdges;
        }
        if( countEdges < minEdgesPerFace ){
            minEdgesPerFace = countEdges;
        }
    }
	cout << "Max Edges per Face: " << maxEdgesPerFace << std::endl;
	cout << "Min Edges per Face: " << minEdgesPerFace << std::endl;

	// Edges per Vertex
	int maxEdgesPerVertex = 0;
	int minEdgesPerVertex = INT_MAX;
    FOR(i,points.size()){
        int countEdges = points[i]->getEdgesCount();
        if( countEdges > maxEdgesPerVertex ){
            maxEdgesPerVertex = countEdges;
        }
        if( countEdges < minEdgesPerVertex ){
            minEdgesPerVertex = countEdges;
        }
    }
	cout << "Max Edges per Vertex: " << maxEdgesPerVertex << std::endl;
	cout << "Min Edges per Vertex: " << minEdgesPerVertex << std::endl;

	// Shared Edges
    int edgeSharedByZeroFace = 0;
    int edgeSharedByOneFace = 0;
    int edgeSharedByTwoFaces = 0;
    FOR(i,edges.size()){
        int countFaces = edges[i]->getFacesCount();
        if( countFaces == 2 ){
            edgeSharedByTwoFaces++;
        } else
        if( countFaces == 1 ){
            edgeSharedByOneFace++;
        } else
        if( countFaces == 0 ){
            edgeSharedByZeroFace++;
        }
    }
    cout << "Edges shared by 0 face: " << edgeSharedByZeroFace << std::endl;
	cout << "Edges shared by 1 face: " << edgeSharedByOneFace << std::endl;
	cout << "Edges shared by 2 faces: " << edgeSharedByTwoFaces << std::endl;
}
