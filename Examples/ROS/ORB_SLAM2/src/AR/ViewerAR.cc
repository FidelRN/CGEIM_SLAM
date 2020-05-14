/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ViewerAR.h"

#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <thread>
#include <cstdlib>

#include <GL/glut.h>

using namespace std;









/// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
//#include <glfw3.h>
//GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

// Standard includes:
//#include <gl/gl.h>
//#include <gl/glu.h>

#include "objloader.hpp"
#include "texture.hpp"

GLuint VertexArrayID;

std::vector<glm::vec3> vertices;
std::vector<glm::vec2> uvs;
std::vector<glm::vec3> normals; // Won't be used at the moment.

bool res;

glm::vec3 a;
glm::vec2 b;

GLuint Texture;


























namespace ORB_SLAM2
{

const float eps = 1e-4;

cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

ViewerAR::ViewerAR(){}

// Check if string is a number
bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

void ViewerAR::Run()
{
    int w,h,wui;

    cv::Mat im, Tcw;
    int status;
    vector<cv::KeyPoint> vKeys;
    vector<MapPoint*> vMPs;
    
    vector<MapPoint*> allvMPs;
    vector<AR*> elems_AR;


    while(1)
    {
        GetImagePose(im,Tcw,status,vKeys,vMPs, allvMPs, elems_AR);
        if(im.empty())
            cv::waitKey(mT);
        else
        {
            w = im.cols;
            h = im.rows;
            break;
        }
    }

    wui=200;

    pangolin::CreateWindowAndBind("Viewer",w+wui,h);

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);

    pangolin::Var<bool> menu_pause("menu.Pause", false, true);
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube",false,false);
    pangolin::Var<bool> menu_clear("menu.Clear All",false,false);
    pangolin::Var<bool> menu_drawim("menu.Draw Image",true,true);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube",true,true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid",true,true);
    pangolin::Var<int> menu_ngrid("menu. Grid Elements",3,1,10);
    pangolin::Var<float> menu_sizegrid("menu. Element Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawpoints("menu.Draw Points",false,true);

    pangolin::Var<bool> menuShowPointsIDs("menu.Show Points IDs",false,true);

    pangolin::Var<bool> menu_draw_ar("menu.Draw AR",true,true);

    pangolin::Var<bool> menu_LocalizationMode("menu.Localization Mode",false,true);
    bool bLocalizationMode = false;

    pangolin::View& d_image = pangolin::Display("image")
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w,h,fx,fy,cx,cy,0.001,1000);

    vector<Plane*> vpPlane;









    // Load .dds file as texture 
    Texture=loadDDS("/home/freviriego/CGEIM_SLAM/model/texture.dds");
    
    // load(parse) .obj file
    res = loadOBJ("/home/freviriego/CGEIM_SLAM/model/object.obj", vertices, uvs, normals);


    scale3DModel(0.01f);








    while(1)
    {
        if (menu_pause) {
            // Add text to image
            AddTextToImage("PAUSE",im,0,0,255);
        }
        else {
            if(menu_LocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menu_LocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Activate camera view
            d_image.Activate();
            glColor3f(1.0,1.0,1.0);

            // Get last image and its computed pose from SLAM
            GetImagePose(im,Tcw,status,vKeys,vMPs, allvMPs, elems_AR);

            // Add text to image
            PrintStatus(status,bLocalizationMode,im);

            if(menu_drawpoints)
                DrawTrackedPoints(vKeys,vMPs,im, menuShowPointsIDs);

            // Draw image
            if(menu_drawim)
                DrawImageTexture(imageTexture,im);

            glClear(GL_DEPTH_BUFFER_BIT);

            // Load camera projection
            glMatrixMode(GL_PROJECTION);
            P.Load();

            glMatrixMode(GL_MODELVIEW);

            // Load camera pose
            LoadCameraPose(Tcw);

            // Draw virtual things
            if(status==2)
            {
                if (menu_draw_ar) {
                    // Draw AR
                    DrawAR(allvMPs, elems_AR);
                }
                
                if(menu_clear)
                {
                    if(!vpPlane.empty())
                    {
                        for(size_t i=0; i<vpPlane.size(); i++)
                        {
                            delete vpPlane[i];
                        }
                        vpPlane.clear();
                        cout << "All cubes erased!" << endl;
                    }
                    menu_clear = false;
                }
                if(menu_detectplane)
                {
                    Plane* pPlane = DetectPlane(Tcw,vMPs,50);
                    if(pPlane)
                    {
                        cout << "New virtual cube inserted!" << endl;
                        vpPlane.push_back(pPlane);
                    }
                    else
                    {
                        cout << "No plane detected. Point the camera to a planar region." << endl;
                    }
                    menu_detectplane = false;
                }

                if(!vpPlane.empty())
                {
                    // Recompute plane if there has been a loop closure or global BA
                    // In localization mode, map is not updated so we do not need to recompute
                    bool bRecompute = false;
                    if(!bLocalizationMode)
                    {
                        if(mpSystem->MapChanged())
                        {
                            cout << "Map changed. All virtual elements are recomputed!" << endl;
                            bRecompute = true;
                        }
                    }

                    for(size_t i=0; i<vpPlane.size(); i++)
                    {
                        Plane* pPlane = vpPlane[i];

                        if(pPlane)
                        {
                            if(bRecompute)
                            {
                                pPlane->Recompute();
                            }
                            glPushMatrix();
                            pPlane->glTpw.Multiply();

                            // Draw cube
                            if(menu_drawcube)
                            {
                                DrawCube(menu_cubesize);
                            }

                            // Draw grid plane
                            if(menu_drawgrid)
                            {
                                DrawPlane(menu_ngrid,menu_sizegrid);
                            }

                            glPopMatrix();
                        }
                    }
                }
            }
        }

        pangolin::FinishFrame();
        usleep(mT*1000);
    }

}

void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs,
                            const vector<ORB_SLAM2::MapPoint*> &allvMPs, const vector<AR*> &elems_AR)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;

    mallvMPs = allvMPs;
    melems_AR = elems_AR;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs,
                            std::vector<MapPoint*> &allvMPs, std::vector<AR*> &elems_AR)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;

    allvMPs = mallvMPs;
    elems_AR = melems_AR;
}

void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
        pangolin::OpenGlMatrix M;

        M.m[0] = Tcw.at<float>(0,0);
        M.m[1] = Tcw.at<float>(1,0);
        M.m[2] = Tcw.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Tcw.at<float>(0,1);
        M.m[5] = Tcw.at<float>(1,1);
        M.m[6] = Tcw.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Tcw.at<float>(0,2);
        M.m[9] = Tcw.at<float>(1,2);
        M.m[10] = Tcw.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = Tcw.at<float>(0,3);
        M.m[13] = Tcw.at<float>(1,3);
        M.m[14] = Tcw.at<float>(2,3);
        M.m[15]  = 1.0;

        M.Load();
    }
}

void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
{
    if(!bLocMode)
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
        }
    }
    else
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
        }
    }
}

void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void ViewerAR::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
{
    if(!im.empty())
    {
        imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();
    }
}

void ViewerAR::DrawCube(const float &size,const float x, const float y, const float z)
{
    pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
    glPushMatrix();
    M.Multiply();
    pangolin::glDrawColouredCube(-size,size);
    glPopMatrix();
}

void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
{
    glPushMatrix();
    pPlane->glTpw.Multiply();
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
}

void ViewerAR::DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs, cv::Mat &im, const bool showPointsIDs)
{
    const int N = vKeys.size();


    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
            if (showPointsIDs){
                string s = to_string(vMPs[i]->mnId);
                cv::putText(im,s,vKeys[i].pt,cv::FONT_HERSHEY_PLAIN,0.6,cv::Scalar(0,0,1),0.8,8);
            }
        }
    }
}

Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                vPoints.push_back(pMP->GetWorldPos());
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = std::max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    vector<MapPoint*> vInlierMPs(nInliers,NULL);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}

Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    Tpw = cv::Mat::eye(4,4,CV_32F);


    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;

}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;
}

float distance(float x1, float y1,  
            float z1, float x2,  
            float y2, float z2) 
{ 
    float d = sqrt(pow(x2 - x1, 2) +  
                pow(y2 - y1, 2) +  
                pow(z2 - z1, 2) * 1.0); 
    return d; 
} 

void ViewerAR::DrawAR(const std::vector<MapPoint*> allvMPs, const std::vector<AR*> elems_AR)
{
    if (elems_AR.size() == 0)
        return;

    int num_elems = elems_AR.size();
    bool lastValid = elems_AR[elems_AR.size()-1]->valid;
    if (!lastValid)
        num_elems -= 1;

    for (size_t j=0; j<num_elems; j++)
    {
        cv::Mat posOrig;
        cv::Mat posScale;
        bool getPosOrig = false;
        bool getPosScale = false;
        // Get points of each AR object
        for(size_t i=0, iend=allvMPs.size(); i<iend;i++)
        {
            // Get position of AR points
            if (allvMPs[i]->mnId == elems_AR[j]->originID){
                posOrig = allvMPs[i]->GetWorldPos();
                getPosOrig = true;
                if (getPosScale)
                    break;
            }
            if (allvMPs[i]->mnId == elems_AR[j]->scaleID){
                posScale = allvMPs[i]->GetWorldPos();
                getPosScale = true;
                if (getPosOrig)
                    break;
            }        
        }
        // Draw AR object
/*
        // Draw line
        //pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x,-size-y,-z);
        glPushMatrix();
        //M.Multiply();
        pangolin::glDrawLine(posOrig.at<float>(0),posOrig.at<float>(1),posOrig.at<float>(2),
                             posScale.at<float>(0),posScale.at<float>(1),posScale.at<float>(2));
        glPopMatrix();
*/
/*
        // Draw cube
        const GLfloat x0 = posOrig.at<float>(0);
        const GLfloat y0 = posOrig.at<float>(1);
        const GLfloat z0 = posOrig.at<float>(2); 

        const GLfloat sx = posScale.at<float>(0);
        const GLfloat sy = posScale.at<float>(1);
        const GLfloat sz = posScale.at<float>(2); 

        const float width = sqrt(pow(sx - x0, 2) +  
                                 pow(sy - y0, 2)); // +  
                                // pow(sz - z0, 2)); 

        const GLfloat x1 = x0 - width;
        const GLfloat y1 = y0 - width;
        const GLfloat z1 = z0 + width;

        
        const GLfloat verts[] = {
            x0,y0,z0,  x1,y0,z0,  x0,y1,z0,  x1,y1,z0,  // FRONT
            x0,y0,z1,  x0,y1,z1,  x1,y0,z1,  x1,y1,z1,  // BACK
            x0,y0,z0,  x0,y1,z0,  x0,y0,z1,  x0,y1,z1,  // LEFT
            x1,y0,z1,  x1,y1,z1,  x1,y0,z0,  x1,y1,z0,  // RIGHT
            x0,y1,z0,  x1,y1,z0,  x0,y1,z1,  x1,y1,z1,  // TOP
            x0,y0,z0,  x0,y0,z1,  x1,y0,z0,  x1,y0,z1   // BOTTOM
        };


        glVertexPointer(3, GL_FLOAT, 0, verts);
        glEnableClientState(GL_VERTEX_ARRAY);
        
        glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);
        
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
        glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);
        
        glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
        glColor4f(0.0f, 1.0f, 1.0f, 1.0f);
        glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
        
        glDisableClientState(GL_VERTEX_ARRAY);

    }
    */



        glPushMatrix();

        glTranslatef(posOrig.at<float>(0), posOrig.at<float>(1), posOrig.at<float>(2));

        //glEnable(GL_TEXTURE_2D);    
        //glBindTexture(GL_TEXTURE_2D, Texture);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_TRIANGLES);
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
        
        for (int i = 0; i < vertices.size(); i += 1)
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








}

void ViewerAR::scale3DModel(float scaleFactor)
{
    for (int i = 0; i < vertices.size(); i += 1)
    {
        vertices[i] = vertices[i] * vec3(scaleFactor * 1.0f, scaleFactor * 1.0f, scaleFactor * 1.0f);
    }

    for (int i = 0; i < normals.size(); i += 1)
    {
        normals[i] = normals[i] * vec3(scaleFactor * 1.0f, scaleFactor * 1.0f, scaleFactor * 1.0f);
    }

    for (int i = 0; i < uvs.size(); i += 1)
    {
        uvs[i] = uvs[i] * vec2(scaleFactor * 1.0f, scaleFactor * 1.0f);
    }
}


}
