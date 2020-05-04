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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

#include <GL/glut.h>

namespace ORB_SLAM2
{

void displayText( float x, float y, float z, int r, int g, int b, const char *string ) {
    int j = strlen( string );

    glColor3f( r, g, b );
    glRasterPos3f(x, y, z);
    for( int i = 0; i < j; i++ ) {
        glutBitmapCharacter( GLUT_BITMAP_TIMES_ROMAN_10, string[i] );
    }
}

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];



    int argc = 1;
    char *argv[1] = {(char*)"Something"};
    glutInit(&argc, argv);
}

void MapDrawer::DrawMapPoints(const bool drawTextPoints)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    const vector<unsigned long int> ARPoints = mpMap->GetARPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();

        // AR point draw in other color and size
        if (count(ARPoints.begin(), ARPoints.end(), vpMPs[i]->mnId)){
            // Change point size
            glEnd();
            glPointSize(mPointSize*2);
            glBegin(GL_POINTS);
            glColor3f(0.0,1.0,0.0);
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            glEnd();
            // Set point type as initial
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(0.0,0.0,0.0);
        }
        else {
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }

        
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();

        // AR point draw in other color and size
        if (count(ARPoints.begin(), ARPoints.end(), (*sit)->mnId)){
            // Change point size
            glEnd();
            glPointSize(mPointSize*2);
            glBegin(GL_POINTS);
            glColor3f(0.0,1.0,0.0);
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            glEnd();
            // Set point type as initial
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(1.0,0.0,0.0);
        }
        else {
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
    }

    glEnd();

    // Draw position IDs for each point
    if (drawTextPoints) {
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad())
                continue;
            cv::Mat pos = vpMPs[i]->GetWorldPos();
            
            // Get ID, convert to string and display its
            string s = to_string(vpMPs[i]->mnId);
            char val[s.size()+1];
            strcpy(val,s.c_str());
            displayText(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2), 0,0,1, val);
        }
    }

    DrawAR();

}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

// Check if point exist and add it to map
bool MapDrawer::AddMapARPoint(unsigned long int pointID)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad())
            continue;
        if (vpMPs[i]->mnId == pointID){
            vector<unsigned long int> ARPoints = mpMap->GetARPoints();
            if (count(ARPoints.begin(), ARPoints.end(), pointID)){
                // Point already added
                return false;
            } 
            else {
                // Add point to map
                mpMap->AddARPoint(pointID);
                return true;
            }
        }
    }
    return false;
}

void MapDrawer::ResetARPoint()
{
    vector<unsigned long int> ARPoints = mpMap->GetARPoints();
    for(size_t i=0, iend=ARPoints.size(); i<iend;i++)
    {
        mpMap->EraseARPoint(ARPoints[i]);
    }
}

void MapDrawer::DrawAR()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<unsigned long int> ARPoints = mpMap->GetARPoints();
    if(ARPoints.size() < 2)
        return;

 //   glBegin(GL_LINES);
//    glColor3f(0.0,1.0,1.0);
    cv::Mat pos0;
    cv::Mat pos1;
    bool getPos0 = false;
    bool getPos1 = false;

    // Get position of AR points
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if (vpMPs[i]->mnId == ARPoints[0]){
            pos0 = vpMPs[i]->GetWorldPos();
            getPos0 = true;
            if (getPos1)
                break;
        }
        if (vpMPs[i]->mnId == ARPoints[1]){
            pos1 = vpMPs[i]->GetWorldPos();
            getPos1 = true;
            if (getPos0)
                break;
        }        
    }
/*
    // Draw line
    glVertex3f(pos0.at<float>(0),pos0.at<float>(1),pos0.at<float>(2));
    glVertex3f(pos1.at<float>(0),pos1.at<float>(1),pos1.at<float>(2));
    glEnd();
*/




    const GLfloat x0 = pos0.at<float>(0);
    const GLfloat y0 = pos0.at<float>(1);
    const GLfloat z0 = pos0.at<float>(2);
/*
    const GLfloat x1 = pos1.at<float>(0);
    const GLfloat y1 = pos1.at<float>(1);
    const GLfloat z1 = pos1.at<float>(2);
*/
    const float width = 0.2;
    const GLfloat x1 = x0 + width;
    const GLfloat y1 = y0 + width;
    const GLfloat z1 = z0;


    const GLfloat w = x1 - x0;
    
    const GLfloat verts[] = {
        x0,y0,z0,    x1,y0,z0,    x0,y1,z0,    x1,y1,z1,    // FRONT
        x0,y0,z0-w,  x0,y1,z0-w,  x1,y0,z0-w,  x1,y1,z1-w,  // BACK
        x0,y0,z0,    x0,y1,z0,    x0,y0,z0-w,  x0,y1,z0-w,  // LEFT
        x1,y0,z0-w,  x1,y1,z1-w,  x1,y0,z0,    x1,y1,z1,    // RIGHT
        x0,y1,z0,    x1,y1,z1,    x1,y0,z0-w,  x1,y1,z1-w,  // TOP
        x0,y0,z0,    x0,y0,z0-w,  x1,y0,z0,    x1,y0,z0-w   // BOTTOM
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
    glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);
    
    glDisableClientState(GL_VERTEX_ARRAY);











}
} //namespace ORB_SLAM