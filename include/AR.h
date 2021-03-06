#ifndef AR_H
#define AR_H

#include <GL/glew.h>

#include <GL/glut.h>
#include "MapPoint.h"
// Include GLM
#include <glm/glm.hpp>
using namespace glm;


using namespace std;
namespace ORB_SLAM2
{
class MapPoint;

class AR
{
public:
    AR(long unsigned int pID, int type);

    bool SetOriginID(long unsigned int pID);
    bool SetScaleID(long unsigned int pID);
	bool SetScaleID2(long unsigned int pID);

    bool SetValid(const std::vector<MapPoint*> allvMPs, vector<vec3> vert, vector<vec3> uv);
    float distance(const std::vector<MapPoint*> allvMPs);
    void scale3DModel(float scaleFactor);

    void Draw(GLfloat x, GLfloat y, GLfloat z, GLuint tex);
    void Draw(GLfloat x, GLfloat y, GLfloat z, GLfloat x1, GLfloat y1, GLfloat z1);

    long unsigned int originID, scaleID, scaleID2;

    bool valid;
    bool originValid, scaleValid, scale2Valid;

    vector<vec3> vertices;
	vector<vec3> uvs;
    
    // origin, valid, scale, vert, vt
    // Cuando se inserte se usa origin y scale para ver distancia (luego origin no se puede borrar pero scale si)
    // Se carga objeto escalando con esa distancia y ya estan disponibles los vertices y uv para textura
};

} //namespace ORB_SLAM

#endif // AR_H
