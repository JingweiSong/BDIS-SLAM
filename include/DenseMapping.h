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

#ifndef DENSEMAPPING_H
#define DENSEMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>

#include "getDisp.h"    //  BDIS stereo mapping

namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class DenseMapping
{
public:
    DenseMapping(Map* pMap, const float bMonocular, const bool mbDenseMap_, const string& strSettingPath);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF, cv::Mat& mImGray_, cv::Mat& mImGray_right_, cv::Mat& mImRGB_);

    // Thread Synch
    void RequestFinish();
    bool isFinished();

protected:

    void AddDensePoints();
 
    std::mutex mMutexNewKFs;
    std::mutex mMutexFinish;
    KeyFrame* mpCurrentKeyFrame;
    Map* mpMap;

    bool mbMonocular;

    bool mbDenseMap;
    cv::Mat mImGray;
    cv::Mat mImGray_right; 
    cv::Mat mImRGB; 
    string strSettingPath_BDIS;
    ParamBDIS param; 
    int mDepth_gap;

    bool mbNewData;
    bool mbFinishRequested;
    bool mbFinished;
};

} //namespace ORB_SLAM

#endif // DENSEMAPPING_H
