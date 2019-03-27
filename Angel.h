//Zhaoran Wang Math for NavMesh and related test

#ifndef __ANGEL_H__
#define __ANGEL_H__

#include <cmath>
#include <iostream>
#ifndef M_PI
#  define M_PI  3.14159265358979323846
#endif
#ifdef __APPLE__  
#  include <OpenGL/OpenGL.h>
#  include <GLUT/glut.h>
#else
#  include <GL/glew.h>
#  include <GL/freeglut.h>
#  include <GL/freeglut_ext.h>
#pragma comment (lib, "glew32.lib")
#endif 

// Define a helpful macro for handling offsets into buffer objects
#define BUFFER_OFFSET( offset )   ((GLvoid*) (offset))

namespace Angel {

GLuint InitShader( const char* vertexShaderFile,
		   const char* fragmentShaderFile );

const GLfloat  DivideByZeroTolerance = GLfloat(1.0e-07);

const GLfloat  DegreesToRadians = M_PI / 180.0;

}

#include "vec.h"
#include "mat.h"

#define Print(x)  do { std::cerr << #x " = " << (x) << std::endl; } while(0)

using namespace Angel;

#endif 
