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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0),mbDenseInProgress(false)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}


void Map::AddVisMapPoint(cv::Mat& x3D, cv::Mat& x3C)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpVisMapPoints.push_back(x3D);
    mvpVisMapPointsColor.push_back(x3C);
}

void Map::EraseMapPoint(vector<int>& vIndErasePts)
{
    unique_lock<mutex> lock(mMutexMap);
    auto it_pt = mvpVisMapPoints.begin();
    auto it_color = mvpVisMapPointsColor.begin();
    int k = 0;
    int i = 0;
    for(auto it_pt=mvpVisMapPoints.begin();it_pt!=mvpVisMapPoints.end();)
    {
        if(k >= vIndErasePts.size())
        {
            break;
        }  
        if(i == vIndErasePts[k])
        {
            it_pt = mvpVisMapPoints.erase(it_pt);
            k++;
        }
        else
        {
            ++it_pt;
        }
        i++;
    }
    i = 0;k = 0;
    for(auto it_color=mvpVisMapPointsColor.begin();it_color!=mvpVisMapPointsColor.end();)
    {
        if(k >= vIndErasePts.size())
        {
            break;
        }  
        if(i == vIndErasePts[k])
        {
            it_color = mvpVisMapPointsColor.erase(it_color);
            k++;
        }
        else
        {
            ++it_color;
        }
        i++;
    }
}


void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

list<cv::Mat> Map::GetVisMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    if(mbDenseInProgress)   
        return list<cv::Mat>(mvpVisMapPoints_backup.begin(),mvpVisMapPoints_backup.end());
    else
        return list<cv::Mat>(mvpVisMapPoints.begin(),mvpVisMapPoints.end());
}
list<cv::Mat> Map::GetVisMapPointsColor()
{
    unique_lock<mutex> lock(mMutexMap);
    if(mbDenseInProgress)   
        return list<cv::Mat>(mvpVisMapPointsColor_backup.begin(),mvpVisMapPointsColor_backup.end());
    else
        return list<cv::Mat>(mvpVisMapPointsColor.begin(),mvpVisMapPointsColor.end());
}
void Map::backupMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    mvpVisMapPoints_backup.clear();
    mvpVisMapPointsColor_backup.clear();
    mvpVisMapPoints_backup.insert(     mvpVisMapPoints_backup.end(),      mvpVisMapPoints.begin(),      mvpVisMapPoints.end() );
    mvpVisMapPointsColor_backup.insert(mvpVisMapPointsColor_backup.end(), mvpVisMapPointsColor.begin(), mvpVisMapPointsColor.end() );
}


void Map::ClearVisMapPointsColor()
{
    unique_lock<mutex> lock(mMutexMap);
    mvpVisMapPointsColor.clear();
    mvpVisMapPoints.clear();
}


long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

	void Map::Save ( const string& filename )
	{

	}
	void Map::SaveMapPoint( ofstream& f, MapPoint* mp)
	{

	}

} //namespace ORB_SLAM
