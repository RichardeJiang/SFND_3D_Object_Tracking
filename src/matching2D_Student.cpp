
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
// void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
//                       std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType, std::vector<float> &recordedMatchesNum)
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0) {
        int normType;
        if (descriptorType.compare("DES_BINARY")) {
            normType = cv::NORM_HAMMING;
        } else {
            normType = cv::NORM_L2;
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    } else if (matcherType.compare("MAT_FLANN") == 0) {
        // need to convert to CV_32F per https://stackoverflow.com/q/43830849/3455398
        if (descSource.type() != CV_32F) {
            descSource.convertTo(descSource, CV_32F);
        }
        if (descRef.type() != CV_32F) {
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0) { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1

        //recordedMatchesNum.push_back(matches.size());
    } else if (selectorType.compare("SEL_KNN") == 0) { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // ratio should be 0.8 instead of 0.7 in the tutorial
        const float ratio_thresh = 0.8f;
        // vector<DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                matches.push_back(knn_matches[i][0]);
            }
        }

        //recordedMatchesNum.push_back(matches.size());
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
//void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, std::vector<float> &recordedDescriptorTime)
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // BRIEF, ORB, FREAK, AKAZE, SIFT
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0) {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (descriptorType.compare("ORB") == 0) {
        extractor = cv::ORB::create();
    } else if (descriptorType.compare("FREAK") == 0) {
        extractor = cv::xfeatures2d::FREAK::create();
    } else if (descriptorType.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create();
    } else if (descriptorType.compare("SIFT") == 0) {
        extractor = cv::xfeatures2d::SIFT::create();
    } else if (descriptorType.compare("BRIEF") == 0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    // recordedDescriptorTime.push_back(1000 * t / 1.0);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
// void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::vector<float> &recordedDetectorTime, bool bVis)
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // recordedDetectorTime.push_back(1000 * t / 1.0);

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

// Harris, assuming the passed in img is alr in gray scale
//void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::vector<float> &recordedDetectorTime, bool bVis) {
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis) {

    // Param from original Harris exercise
    int blockSize = 2;
    int apertureSize = 3;
    int minResponse = 100;
    double k = 0.04;

    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0;
    for (int r = 0; r < dst_norm.rows; r++)
    {
        for (int c = 0; c < dst_norm.cols; c++)
        {
            int response = (int)dst_norm.at<float>(r, c);
            if (response > minResponse)
            {

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(c, r);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {
                            *it = newKeyPoint;
                            break;
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    // recordedDetectorTime.push_back(1000 * t / 1.0);

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// All other detectors: FAST, BRISK, ORB, AKAZE, and SIFT
//void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, std::vector<float> &recordedDetectorTime, bool bVis) {
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis) {
    cv::Ptr<cv::FeatureDetector> detector;
    double t = (double)cv::getTickCount();
    if (detectorType.compare("FAST") == 0) {
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        detector = cv::FastFeatureDetector::create(30, true, type);
    } else if (detectorType.compare("BRISK") == 0) {
        // threshold, octaves, patternScale
        // cv::BRISK briskDetector(60, 4, 1.0f);
        // briskDetector.create("Feature2D.BRISK");
        // briskDetector.detect(img, keypoints);
        detector = cv::BRISK::create();
    } else if (detectorType.compare("ORB") == 0) {
        detector = cv::ORB::create();
    } else if (detectorType.compare("AKAZE") == 0) {
        detector = cv::AKAZE::create();
    } else if (detectorType.compare("SIFT") == 0) {
        detector = cv::xfeatures2d::SIFT::create();
    } // assuming the input will always be correct, so ignore the else statement

    detector->detect(img, keypoints);
    
    //detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << detectorType << " detector with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // recordedDetectorTime.push_back(1000 * t / 1.0);

    string windowName = detectorType + " Detector Results";

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}