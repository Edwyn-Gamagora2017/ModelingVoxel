#ifndef OFFFILE_H
#define OFFFILE_H

#include "../figures/data_structure/Figure.h"

#include <string>
using namespace std;

class OffFile
{
    public:
        static Figure * readFile( string filename );
        static void writeFile( Figure * f, string filename );
        static void printInfo( Figure * f );
};

#endif
