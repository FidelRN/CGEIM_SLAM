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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include "obj.h"
#include "png.h"

#include <mutex>

// Include GLEW
#include <GL/glew.h>

// Include GLM
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/polar_coordinates.hpp>
using namespace glm;

#include "png.h"
GLuint tex;
PNG tex_png;

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

// Check if string is a number
bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuPause("menu.Pause", false, true);
    pangolin::Var<bool> menu_drawim("menu.Draw Image",true,true);
    pangolin::Var<bool> menu_drawpoints("menu.Draw Points",false,true);
    pangolin::Var<bool> menuShowPointsIDs("menu.Show Points IDs",false,true);
    pangolin::Var<string> menu_originID("menu.Origin-ID");
    pangolin::Var<string> menu_scaleID("menu.Scale-ID");
    pangolin::Var<bool> menu_insert_ar("menu.Insert AR",false,false);
    pangolin::Var<bool> menu_clear_last_ar("menu.Clear last AR",false,false);
    pangolin::Var<bool> menu_clear_ar("menu.Clear all AR",false,false);
    pangolin::Var<bool> menu_draw_ar("menu.Draw AR",true,true);

    // ViewerAR --> Modify/delete
    pangolin::Var<string> menu_separator("menu. ORIGINAL AR (CUBE)");
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube",false,false);
    pangolin::Var<bool> menu_clear("menu.Clear All",false,false);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube",true,true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid",true,true);
    pangolin::Var<int> menu_ngrid("menu. Grid Elements",3,1,10);
    pangolin::Var<float> menu_sizegrid("menu. Element Size",0.05,0.01,0.3);
    

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bPause = false;
    bool bFollowCam, bLocMode;

    // AR points
    bool ARorig = false;
    bool ARscale = false;
    long unsigned int origAR, scaleAR; 


    // Load model (OBJ)
    OBJ obj;
    mat4 xf = rotate(radians(180.0f),vec3(1.0f,0.0f,0.0f));
    obj.load("/home/freviriego/CGEIM_SLAM/model/object.obj", xf);


    // Load texture (image)
    tex_png.load("/home/freviriego/CGEIM_SLAM/model/texture.png");
    glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D,tex);
    glTexStorage2D(GL_TEXTURE_2D,
                    8,
                    GL_RGB32F,
                    tex_png.width(),tex_png.height());
    glTexSubImage2D(GL_TEXTURE_2D,
                    0,
                    0,0,
                    tex_png.width(),tex_png.height(),
                    GL_RGB,
                    GL_FLOAT,
                    tex_png.pixels().data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);

    while(1)
    {
        if (menuPause && bPause){
            menuFollowCamera = false;
            menuLocalizationMode = true;


            if (!ARorig) {
                // Origin point
                if (is_number(menu_originID)){
                    long unsigned int originID = stoul(menu_originID);
                    bool addPoint;
                    if (!ARscale)
                        addPoint = mpMapDrawer->CreateAR(originID,true);
                    else
                        addPoint = mpMapDrawer->SetOriginARPoint(originID);

                    if (addPoint) {
                        ARorig = true;
                        origAR = originID;
                        cout << "Added orig point: " << originID << endl;
                    }
                    else {
                        // Point not exists
                        menu_originID = "";  
                    }
                }
                else {
                    // Bad argument
                    menu_originID = "";       
                }
            }
            if (!ARscale) {
                // Point 2
                if (is_number(menu_scaleID)){
                    long unsigned int scaleID = stoul(menu_scaleID);
                    bool addPoint;
                    if (!ARorig)
                        addPoint = mpMapDrawer->CreateAR(scaleID,false);
                    else
                        addPoint = mpMapDrawer->SetScaleARPoint(scaleID);

                    if (addPoint) {
                        ARscale = true;
                        scaleAR = scaleID;
                        cout << "Added scale point: " << scaleID << endl;
                    }
                    else {
                        // Point not exists
                        menu_scaleID = "";  
                    }
                }
                else {
                    // Bad argument
                    menu_scaleID = "";       
                }
            }
            if (ARorig){
                // Check if value has changed
                // Origin point
                if (is_number(menu_originID)){
                    long unsigned int originID_aux = stoul(menu_originID);
                    if (origAR != originID_aux){
                        // Value has changed
                        bool addPoint = mpMapDrawer->SetOriginARPoint(originID_aux);
                        if (addPoint) {
                            origAR = originID_aux;
                            cout << "Added orig point (changed): " << originID_aux << endl;
                        }
                        else {
                            // Point not exists, set last value
                            menu_originID = to_string(origAR);    
                        }
                    }
                }
                else {
                    // Bad argument, set last value
                    menu_originID = to_string(origAR);   
                }
            }
            if (ARscale){
                // Check if value has changed
                // Scale point
                if (is_number(menu_scaleID)){
                    long unsigned int scaleID_aux = stoul(menu_scaleID);
                    if (scaleAR != scaleID_aux){
                        // Value has changed
                        bool addPoint = mpMapDrawer->SetScaleARPoint(scaleID_aux);
                        if (addPoint) {
                            scaleAR = scaleID_aux;
                            cout << "Added scale point (changed): " << scaleID_aux << endl;
                        }
                        else {
                            // Point not exists, set last value
                            menu_scaleID = to_string(scaleAR); 
                        }
                    }
                }
                else {
                    // Bad argument, set last value
                    menu_scaleID = to_string(scaleAR);   
                }
            }
            if (menu_insert_ar){
                menu_insert_ar = false;
                if (ARorig && ARscale){
                    // Insert AR
                    bool insertedAr = mpMapDrawer->InsertAR(obj.faces(), obj.texcoord());
                    if (insertedAr){
                        cout << "Inserted AR" << endl;
                        ARorig = false;
                        ARscale = false;
                        menu_originID = ""; 
                        menu_scaleID = "";  
                    }
                }
            }
        }
        else if(menuPause && !bPause){
            // Store menu options before pause
            bFollowCam = menuFollowCamera;
            bLocMode = menuLocalizationMode;
            // Stop follow camera and start loc mode (stop local mapping and loop closing)
            menuFollowCamera = false;
            menuLocalizationMode = true;
            bPause = true;

        }
        else if (!menuPause && bPause){
            // Restore menu options
            menuFollowCamera = bFollowCam;
            menuLocalizationMode = bLocMode;
            bPause = false;
        }


        if (menu_clear_last_ar){
            // Clear last AR
            menu_clear_last_ar = false;
            mpMapDrawer->ClearLastAR();
        }

        if (menu_clear_ar){
            // Clear AR
            menu_clear_ar = false;
            ARorig = false;
            ARscale = false;
            menu_originID = "";
            menu_scaleID = "";   
            mpMapDrawer->ResetAR();
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);


        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints(menuShowPointsIDs);
        if(menu_draw_ar)
            mpMapDrawer->DrawAR(tex);

        pangolin::FinishFrame();

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
            ARorig = false;
            ARscale = false;
            menu_originID = "";
            menu_scaleID = "";   
            // Clear AR
            mpMapDrawer->ResetAR();
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
