#include "AR.h"

namespace ORB_SLAM2
{

AR::AR(long unsigned int pID, bool isOrigin)
{
    if (isOrigin){
        this->originID = pID;
        this->originValid = true;
        this->scaleValid = false;
    }
    else {
        this->scaleID = pID;
        this->scaleValid = true;
        this->originValid = false;    
    }
    
    this->valid = false;
}

bool AR::SetOriginID(long unsigned int pID)
{
    // No changes available if it is valid
    if (this->valid)
        return false;
    // No change if it is the same point as scale
    if (this->scaleValid && this->scaleID == pID)
        return false;

    this->originID = pID;
    this->originValid = true;
    return true;
}

bool AR::SetScaleID(long unsigned int pID)
{
    // No changes available if it is valid
    if (this->valid)
        return false;
    // No change if it is the same point as origin
    if (this->originValid && this->originID == pID)
        return false;

    this->scaleID = pID;
    this->scaleValid = true;
    return true;
}

bool AR::SetValid(const std::vector<MapPoint*> allvMPs, vector<vec3> vert, vector<vec3> uv)
{
    if (this->originValid && this->scaleValid){
        // Distance between orig and scale
        const float dist = distance(allvMPs);

        // Scale object and store it
        vertices = vert;
        uvs = uv;
        scale3DModel(dist);

        this->valid = true;
        return true;
    }
    return false;
}

float AR::distance(const std::vector<MapPoint*> allvMPs)
{
    cv::Mat posOrig;
    cv::Mat posScale;
    bool getPosOrig = false;
    bool getPosScale = false;
    // Get points of each AR object
    for(size_t i=0, iend=allvMPs.size(); i<iend;i++)
    {
        // Get position of AR points
        if (allvMPs[i]->mnId == this->originID){
            posOrig = allvMPs[i]->GetWorldPos();
            getPosOrig = true;
            if (getPosScale)
                break;
        }
        if (allvMPs[i]->mnId == this->scaleID){
            posScale = allvMPs[i]->GetWorldPos();
            getPosScale = true;
            if (getPosOrig)
                break;
        }        
    }
    // Calculate distance
    const GLfloat x0 = posOrig.at<float>(0);
    const GLfloat y0 = posOrig.at<float>(1);
    const GLfloat z0 = posOrig.at<float>(2); 

    const GLfloat sx = posScale.at<float>(0);
    const GLfloat sy = posScale.at<float>(1);
    const GLfloat sz = posScale.at<float>(2); 

    const float width = sqrt(pow(sx - x0, 2) +  
                             pow(sy - y0, 2)); // +  
                            // pow(sz - z0, 2)); 
    return width;
}

void AR::scale3DModel(float scaleFactor)
{
    for (size_t i = 0; i < vertices.size(); i += 1)
    {
        vertices[i] = vertices[i] * vec3(scaleFactor * 1.0f, scaleFactor * 1.0f, scaleFactor * 1.0f);
    }

    for (size_t i = 0; i < uvs.size(); i += 1)
    {
        uvs[i] = uvs[i] * vec3(scaleFactor * 1.0f, scaleFactor * 1.0f, scaleFactor * 1.0f);
    }
}

void AR::Draw(GLfloat x, GLfloat y, GLfloat z, GLuint tex)
{
    glPushMatrix();

    glTranslatef(x, y, z);
    glEnable(GL_TEXTURE_2D);    
    glBindTexture(GL_TEXTURE_2D, tex);

    glBegin(GL_TRIANGLES);

    glm::vec3 a;
    glm::vec3 b;

    for (size_t i = 0; i < vertices.size(); i += 1)
    {
        a = vertices[i];
        b = uvs[i];
        glNormal3f(a.x, a.y, a.z);
        glTexCoord2d(b.x, b.y);
            
        glVertex3f(a.x, a.y, a.z);      
    }

    glEnd();//end drawing of line loop
    glDisable(GL_TEXTURE_2D);

    glPopMatrix();
}

} //namespace ORB_SLAM
