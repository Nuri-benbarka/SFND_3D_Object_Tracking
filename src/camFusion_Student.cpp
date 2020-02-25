
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <set>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 0.5, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 0.5, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    map< pair<int,int> , size_t > bbMatchesCounter;
    std::vector<cv::DMatch> tt;
    for(auto match:matches){
        for(auto currBBox:currFrame.boundingBoxes){
            for(auto prevBBox:prevFrame.boundingBoxes){
                if(currBBox.roi.contains(currFrame.keypoints[match.trainIdx].pt) && prevBBox.roi.contains(prevFrame.keypoints[match.queryIdx].pt)){
                    bbMatchesCounter[make_pair(prevBBox.boxID,currBBox.boxID)]++;
                    if(prevBBox.boxID==0 && currBBox.boxID==0)
                        tt.push_back(match);
                }
            }
        }
    }
    cv::Mat matchImg = (currFrame.cameraImg).clone();
    cv::drawMatches(prevFrame.cameraImg, prevFrame.keypoints,
                    currFrame.cameraImg, currFrame.keypoints,
                    tt, matchImg,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "Matching keypoints between two camera images";
    cv::namedWindow(windowName, 7);
    cv::imshow(windowName, matchImg);
    cout << "Press key to continue to next image" << endl;
    cv::waitKey(0); // wait for key to be pressed
    /*std::map<std::pair<int, int>, size_t> bbMatchesCounter;
    for (const auto & match : matches)
    {
        // keypoint indices are same as descripors
        int prevFrameDescIndex = match.queryIdx;
        int currFrameDescIndex = match.trainIdx;
        const auto & prevFramePt = prevFrame.keypoints.at(prevFrameDescIndex).pt;
        const auto & currFramePt = currFrame.keypoints.at(currFrameDescIndex).pt;
        for (const auto & prevFrameBB : prevFrame.boundingBoxes)
        {
            if (!prevFrameBB.roi.contains(prevFramePt))
                continue;
            for (const auto & currFrameBB : currFrame.boundingBoxes)
            {
                if (!currFrameBB.roi.contains(currFramePt))
                    continue;
                bbMatchesCounter[std::make_pair(prevFrameBB.boxID, currFrameBB.boxID)]++;
            }
        }
    }*/
    std::vector<std::tuple<int, int, size_t>> bbMatches;
    for (const auto & pair : bbMatchesCounter)
        bbMatches.push_back(std::make_tuple(pair.first.first, pair.first.second, pair.second));

    // sort by matches count
    std::sort(bbMatches.begin(), bbMatches.end(),
              [](const std::tuple<int, int, size_t> & l, const std::tuple<int, int, size_t> & r)
              {
                  return std::get<2>(l) > std::get<2>(r);
              });

    std::set<int> visitedPrevFrameBBs;
    bbBestMatches.clear();
    for (const auto & tup : bbMatches)
    {
        if (visitedPrevFrameBBs.count(std::get<0>(tup)))
            continue;
        visitedPrevFrameBBs.insert(std::get<0>(tup));
        bbBestMatches.insert(std::make_pair(std::get<0>(tup), std::get<1>(tup)));
        cout << std::get<0>(tup) << " - " << std::get<1>(tup) << " : " <<  std::get<2>(tup) << endl;
    }

    /*for(auto boundingBox:currFrame.boundingBoxes){
        vector<int> BBKeypointMatches(prevFrame.boundingBoxes.size(),0);
        std::vector<cv::KeyPoint> currBBKeypoints;
        std::vector<cv::KeyPoint> prevBBKeypoints;
        std::vector<cv::DMatch> tt;

        for (int i=0; i < currFrame.keypoints.size(); i++){
            if(boundingBox.roi.contains(currFrame.keypoints[i].pt)) {
                currBBKeypoints.push_back(currFrame.keypoints[i]);
                //boundingBox.keypoints.push_back(currFrame.keypoints[i]);
                cout << "pushed " << i << " " << endl;
                for (auto kptMatch:matches) {
                    cout << "looking at" << kptMatch.queryIdx << " " << kptMatch.trainIdx << " " <<endl;
                    if (kptMatch.queryIdx == i){
                        cout << "found " << kptMatch.trainIdx << " " << endl;
                        prevBBKeypoints.push_back(prevFrame.keypoints[kptMatch.queryIdx]);
                        tt.push_back(kptMatch);

                    }


                }
                //boundingBox.kptMatches.push_back(matches[i]);
            }

        }


        //clusterKptMatchesWithROI(boundingBox, prevFrame.keypoints, currFrame.keypoints, currFrame.kptMatches);
        //cout<<boundingBox.boxID<<endl;

        for(auto match:boundingBox.kptMatches){
            for(int i=0; i < prevFrame.boundingBoxes.size(); i++){
                if(prevFrame.boundingBoxes[i].roi.contains(prevFrame.keypoints[match.trainIdx].pt))
                    BBKeypointMatches[i]++;
            }
        }
        int maxElement = 0;
        int maxValue = 0;
        for(int i = 0; i < BBKeypointMatches.size(); i++){
            if(BBKeypointMatches[i] > maxValue){
                maxValue = BBKeypointMatches[i];
                maxElement = i;
            }
        }
        if(maxValue > 0)
            bbBestMatches.insert({boundingBox.boxID,prevFrame.boundingBoxes[maxElement].boxID});



    }*/
}
