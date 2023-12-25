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

#include "DenseMapping.h"

#include<mutex>

namespace ORB_SLAM2
{

DenseMapping::DenseMapping(Map *pMap, const float bMonocular, const bool mbDenseMap_, const string& strSettingPath):
    mbMonocular(bMonocular), mpMap(pMap), mbFinishRequested(false), mbFinished(true), mDepth_gap(0)
{
    mbDenseMap = mbDenseMap_;
    if(mbDenseMap)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        param.lv_f = fSettings["BDISparam.lv_f"];
        param.lv_l = fSettings["BDISparam.lv_l"];
        param.maxiter = fSettings["BDISparam.maxiter"];
        param.miniter = fSettings["BDISparam.miniter"];
        param.mindprate = fSettings["BDISparam.mindprate"];
        param.mindrrate = fSettings["BDISparam.mindrrate"];
        param.minimgerr = fSettings["BDISparam.minimgerr"];
        param.patchsz = fSettings["BDISparam.patchsz"];
        param.poverl = fSettings["BDISparam.poverl"];
        param.usefbcon = fSettings["BDISparam.usefbcon"];
        param.patnorm = fSettings["BDISparam.patnorm"];
        param.costfct = fSettings["BDISparam.costfct"];
        param.verbosity = fSettings["BDISparam.verbosity"];
        param.ratio_patch_valid = fSettings["BDISparam.ratio_patch_valid"];
        param.num_window = fSettings["BDISparam.num_window"];
        param.unit_disturb = fSettings["BDISparam.unit_disturb"];
        param.min_prob = fSettings["BDISparam.min_prob"];
        if (int(fSettings["BDISparam.bool_var"])!= 0)
            param.bool_var = true;
        mDepth_gap = fSettings["Viewer3D.depth_gap"];
    }
    mbNewData = false;
}


void DenseMapping::Run()
{
    while(!mbMonocular) // Only works for stereo
    {
        if(mbFinishRequested)
        {
            mbFinished = true;
            break;
        }
        // Check if there are keyframes in the queue
        if(mbNewData)
        {
            AddDensePoints();
        }
        
        mbNewData = false;

        usleep(3000);
    }
}


void DenseMapping::AddDensePoints()
{
    int rpyrtype = CV_32FC1;
    int selectchannel = 1;
    cv::Mat flowout = getDisp(mImGray, mImGray_right, selectchannel, rpyrtype, param);
    
    mpMap->backupMapPoints();
    mpMap->mbDenseInProgress = true;   

    const float &fx = mpCurrentKeyFrame->fx;
    const float &cx = mpCurrentKeyFrame->cx;
    const float &cy = mpCurrentKeyFrame->cy;
    const float &bf = mpCurrentKeyFrame->mbf;
    const int rows = mImGray.rows;
    const int cols = mImGray.cols;

    const float max_z = bf / 8.0;	//	Minimal 3 pixel parallax
    const float max_y = cy * max_z / fx;
    const float max_x = cx * max_z / fx;

    int k = 0;
    int n = 0;
    int thresh_color255 = 40;
    float thresh_color = thresh_color255 / 255.0;


    cv::Mat T_CurrentKeyFramewc = mpCurrentKeyFrame->GetPoseInverse();   // Current Keyframe
    cv::Mat T_CurrentKeyFramecw = mpCurrentKeyFrame->GetPose();   // Current Keyframe
    
     const list<cv::Mat> visMPs = mpMap->GetVisMapPoints();
     auto it = visMPs.begin();
     vector<int> vIndErasePts; 
     if(!visMPs.empty())
     {
         int i = 0;
         while(i < visMPs.size())
         {
            cv::Mat pos = (*it).clone();
            it++;
            pos = T_CurrentKeyFramecw.rowRange(0,3).colRange(0,3)*pos+T_CurrentKeyFramecw.rowRange(0,3).col(3);
            if(pos.at<float>(2) <= 0 ||
               pos.at<float>(2) >= max_z ||
               std::abs(pos.at<float>(0)) > max_x ||
               std::abs(pos.at<float>(1)) > max_y)
            {
            	i++;continue;
            }
            const int u = std::round(fx*pos.at<float>(1)/pos.at<float>(2)+cy);
            const int v = std::round(fx*pos.at<float>(0)/pos.at<float>(2)+cx);

            if(u>=0 && v>=0 && u<rows && v<cols)
            {
                if(flowout.at<float>(u, v) < 0)
                {
                    if((int)mImRGB.at<cv::Vec3b>(u,v)[2] > thresh_color255 &&
                       (int)mImRGB.at<cv::Vec3b>(u,v)[1] > thresh_color255 &&
                       (int)mImRGB.at<cv::Vec3b>(u,v)[0] > thresh_color255)
                       {
                           //mpMap->EraseMapPoint(i);
                           vIndErasePts.push_back(i);
                           n++;
                       }
                }
            }
            i++;
         }
     }
     mpMap->EraseMapPoint(vIndErasePts);

    

    //  II. Add new points
    uint8_t* pixelPtr = (uint8_t*)mImRGB.data;
    for(int u=0;u<mImRGB.rows;u+=mDepth_gap)
    {
        for(int v=0;v<mImRGB.cols;v+=mDepth_gap)
        {
            if(flowout.at<float>(u, v) < 0)
            {
                const float z = -bf/flowout.at<float>(u, v);
                const float y = (u-cy)*z/fx;
                const float x = (v-cx)*z/fx;
                cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
                cv::Mat x3C;
                if(mImRGB.channels()==3)
                {
                    x3C = (cv::Mat_<float>(3,1) << (float)pixelPtr[u*mImRGB.cols*3 + v*3 + 2]/255.0,(float)pixelPtr[u*mImRGB.cols*3 + v*3 + 1]/255.0,(float)pixelPtr[u*mImRGB.cols*3 + v*3 + 0]/255.0);
                    
                }
                else
                {
                     x3C = (cv::Mat_<float>(3,1) << (float)pixelPtr[u*mImRGB.cols + v + 2]/255.0,(float)pixelPtr[u*mImRGB.cols + v + 1]/255.0,(float)pixelPtr[u*mImRGB.cols + v + 0]/255.0);
                }
                if(x3C.at<float>(0) < thresh_color &&
                   x3C.at<float>(1) < thresh_color && 
                   x3C.at<float>(2) < thresh_color)
                   {
                       continue;
                   }

                cv::Mat x3D = T_CurrentKeyFramewc.rowRange(0,3).colRange(0,3)*x3Dc+T_CurrentKeyFramewc.rowRange(0,3).col(3);
                mpMap->AddVisMapPoint(x3D, x3C);
                k++;
            }
        }
    }
    mpMap->mbDenseInProgress = false;
}

void DenseMapping::InsertKeyFrame(KeyFrame *pKF, cv::Mat& mImGray_, cv::Mat& mImGray_right_, cv::Mat& mImRGB_)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mpCurrentKeyFrame = pKF;

    mImGray = mImGray_;
    mImGray_right = mImGray_right_;
    mImRGB = mImRGB_;

    mbNewData = true;   //  New data added
}

bool DenseMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
void DenseMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

} //namespace ORB_SLAM
