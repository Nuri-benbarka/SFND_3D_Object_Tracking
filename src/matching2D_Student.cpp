#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, const std::string& descriptorType, const std::string& matcherType, const std::string& selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType == "MAT_BF")
    {
        int normType = descriptorType == "SIFT" ? cv::NORM_L2 : cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType == "MAT_FLANN")
    {
        if(descSource.type() != CV_32F){
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }


    // perform matching task
    if (selectorType == "SEL_NN")
    { // nearest neighbor (best match)
        auto t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << matcherType << " (NN) matching with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl ;
    }
    else if (selectorType == "SEL_KNN")
    { // k nearest neighbors (k=2)

        std::vector< std::vector<cv::DMatch> > knn_matches;
        auto t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << matcherType << " (KNN) matching with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl ;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < 0.8 * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, const string& descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == "BRISK")
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType == "BRIEF") extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    else if (descriptorType == "SIFT") extractor = cv::xfeatures2d::SIFT::create();
    else if (descriptorType == "ORB") extractor = cv::ORB::create();
    else if (descriptorType == "AKAZE") extractor = cv::AKAZE::create();
    else if (descriptorType == "FREAK") extractor = cv::xfeatures2d::FREAK::create();

    // perform feature description
    auto t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool useHarrisDetector, const cv::Mat& mask)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    auto t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k);

    // add corners to result vector
    for (auto & corner : corners)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(corner.x, corner.y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    if (!useHarrisDetector)
        cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    else
        cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    float average = 0;
    int i = 0;
    for(const auto& keypoint: keypoints) {
        if (i == 0){
            average = keypoint.size;
            i++;
            continue;
        }
        average = (average * i + keypoint.size) / ++i;
    }
    cout << "Diameter of the meaningful keypoint neighborhood " << average << endl ;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string& detectorType, bool bVis, const cv::Mat& mask){
    //FAST, BRISK, ORB, AKAZE, SIFT
    cv::Ptr<cv::FeatureDetector> detector;
    if(detectorType =="SIFT") detector = cv::xfeatures2d::SIFT::create();
    else if (detectorType == "BRISK") detector = cv::BRISK::create();
    else if (detectorType == "FAST") detector = cv::FastFeatureDetector::create();
    else if (detectorType == "ORB") detector = cv::ORB::create();
    else if (detectorType == "AKAZE") detector = cv::AKAZE::create();

    auto t = (double)cv::getTickCount();
    detector->detect(img,keypoints,mask);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType <<" detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    float average = 0;
    int i = 0;
    for(const auto& keypoint: keypoints) {
        if (i == 0){
            average = keypoint.size;
            i++;
            continue;
        }
        average = (average * i + keypoint.size) / ++i;
    }
    cout << "Diameter of the meaningful keypoint neighborhood " << average << endl ;

    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }



}