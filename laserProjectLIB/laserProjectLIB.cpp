
#include <iostream>
#include "laserProjectLIB.h"


namespace laserProject {
    void printMes() {
        std::cout << "hello";
    }
    cv::Mat takeImg(cv::VideoCapture capture) {
        cv::Mat frame;
        if (capture.isOpened()) {
            cout << "camera opened" << endl;
            capture >> frame;
        }
        else {
            try
            {
                bool isopen = capture.open(0);
                if (isopen == false)
                    throw "can't open cam";
                capture >> frame;
            }
            catch (const char* msg)
            {
                cerr << msg << endl;
            }
        }

        return frame;

    }

    //find ellipse and send out vector<int> containing indexes of the 6 ellipse 
    vector<int> findCircleContoursIndexes(vector<vector<cv::Point>> const& contour, int numOfContour, int contourSize, int contourArea) {
        vector<int> index;
        vector<cv::Vec4i> hierachy;
        for (int i = 0; i < contour.size(); i++) {
            double area = cv::contourArea(contour[i]);
            if (contour[i].size() > contourSize && area > contourArea) {
                if (checkCircularity(contour[i]) != true)
                    continue;
                index.push_back(i);
            }
        }
        try
        {
            if (index.size() != numOfContour)
                throw 0;
            else
                cout << "number of holes found " << index.size()<<endl;
        }
        catch (int x)
        {
            //cout << "number of holes are not accurate" << endl;
            //cout << "number of holes found " << index.size() << endl;
        }
        catch (...) {
            cout << "default exception" << endl;
        }
        return index;
    }

    // checks the circularity of contour 
    bool checkCircularity(vector<cv::Point>const &contour) {
        double contourArea = cv::contourArea(contour);
        double contourPerimeter = cv::arcLength(contour, TRUE);
        double circularity = (4.0 * M_PI * contourArea) / (pow(contourPerimeter, 2));
        if (circularity > 0.8 && contourArea > 100)
            return true;

        else
            return false;
    }

    // indexes is the indexes of 6 ellipses found 
    // contours are the all the contours found in the image
    std::vector<cv::Point2f>  findBiggestEllipse(vector<vector<cv::Point>>const& contours, const vector<int>& indexes) {

        std::vector<cv::Point2f> ver(4);
        //find the biggest ellipse 
        int maxEllipseIndex = indexes[indexes.size() - 1];

        cv::RotatedRect rect = cv::fitEllipse(contours[maxEllipseIndex]);
        rect.points(ver.data());

        return ver;
    }

    // (x,y)
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point> const & contour) {
        int size = contour.size();
        std::vector<cv::Point3f> laserCon(size);
        for (int i = 0; i < size; i++) {
            laserCon[i].x = contour[i].x;
            laserCon[i].y = contour[i].y;
            laserCon[i].z = 1;
        }
        return laserCon;
    }

    //overriding for point2f 
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point2f> const & contour) {
        int size = contour.size();
        std::vector<cv::Point3f> laserCon(size);
        for (int i = 0; i < size; i++) {
            laserCon[i].x = contour[i].x;
            laserCon[i].y = contour[i].y;
            laserCon[i].z = 1;
        }
        return laserCon;
    }

    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point> const& contour, cv::Mat const& T) {
        //hCoordPoint is the (x,y,1) of laserContours before transformed
        std::vector<cv::Point3f> hCoordPoint = makeHomogeneous(contour);
        std::vector<cv::Point2f> transformedPoints(contour.size());

        cv::Mat laserConMat(contour.size(), 3, CV_32F, hCoordPoint.data());
        cv::transpose(laserConMat, laserConMat);
        cv::Mat m(T * laserConMat);

        for (int i = 0; i < contour.size(); i++) {
            transformedPoints[i].x = m.at<float>(0, i) / m.at<float>(2, i);
            transformedPoints[i].y = m.at<float>(1, i) / m.at<float>(2, i);
        }
        return transformedPoints;
    }

    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point2f> const& contour, cv::Mat const& T) {
        //hCoordPoint is the (x,y,1) of laserContours before transformed
        std::vector<cv::Point3f> hCoordPoint = makeHomogeneous(contour);
        std::vector<cv::Point2f> transformedPoints(contour.size());

        cv::Mat laserConMat(contour.size(), 3, CV_32F, hCoordPoint.data());
        cv::transpose(laserConMat, laserConMat);
        cv::Mat m(T * laserConMat);

        for (int i = 0; i < contour.size(); i++) {
            transformedPoints[i].x = m.at<float>(0, i) / m.at<float>(2, i);
            transformedPoints[i].y = m.at<float>(1, i) / m.at<float>(2, i);
        }
        return transformedPoints;
    }



    void drawWithContour(const std::vector<cv::Point2f>& pt, const cv::Mat& img, const char* str) {
        for (int i = 0; i < pt.size(); i++) {
            cv::circle(img, pt[i], 1, cv::Scalar(0, 100, 250), 1);
        }
        //cv::imshow(str, img);
    }
    void drawWithContourI(const std::vector<cv::Point>& pt, const cv::Mat& img, const char* str) {
        cv::RNG rng(12345);
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        for (int i = 0; i < pt.size(); i++) {
            cv::circle(img, pt[i], 1, color);
        }
        //cv::imshow(str, img);
    }

    myCircle findCircleWithCenters(const std::vector<vector<cv::Point2f>> contours) {
        myCircle result;
        int circleNum = contours.size();
        vector<cv::Point2f> circleCen(circleNum);
        vector<float> circleRad(circleNum);
        for (int i = 0; i < contours.size(); i++) {
            cv::minEnclosingCircle(contours[i], circleCen[i], circleRad[i]);
        }
        cv::minEnclosingCircle(circleCen, result.center, result.radius);

        return result;
    }

    //return the points in (3*N) Matrix 
    std::vector<cv::Point2f> returnCirclePoints(int numberOfPoints, const myCircle& circ) {
        std::vector<cv::Point2f> circPoint2f(numberOfPoints);
        for (int i = 0; i < numberOfPoints; i++) {
            circPoint2f[i].x = circ.center.x + circ.radius*(float)cos(2 * M_PI*i / numberOfPoints);
            circPoint2f[i].y = circ.center.y + circ.radius*(float)sin(2 * M_PI*i / numberOfPoints);
        }
        return circPoint2f;
    }

    cv::Mat thresholdBlurredImage(const cv::Mat& original, int minThresh) {
        cv::Mat originalGray;
        cv::Mat blurredOriginal;
        cv::Mat thresholdImg;
        cv::cvtColor(original, originalGray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(originalGray, blurredOriginal, cv::Size(5, 5), 0, 0);
        cv::threshold(blurredOriginal, thresholdImg, minThresh, 255, cv::THRESH_BINARY);
        return thresholdImg;
    }
    std::vector<std::vector<cv::Point>> getContoursSorted(const cv::Mat& thresholdImg) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholdImg, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> &a, const vector<cv::Point> &b) {return a.size() < b.size(); });
        return contours;
    }
    std::vector<std::vector<cv::Point>> getContoursSortedExternal(const cv::Mat& thresholdImg) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholdImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> &a, const vector<cv::Point> &b) {return a.size() < b.size(); });
        return contours;
    }

    cv::Mat findTmatrixWithEllipse(std::vector<std::vector<cv::Point>> contours, int numOfContour, int contourSize, int contourArea) {

        vector<int> indexes = findCircleContoursIndexes(contours, numOfContour, contourSize, contourArea);
        std::vector<cv::Point2f> ver = findBiggestEllipse(contours, indexes);
        cv::Point2f transformedRect[4] = { {0,0},{400,0},{400,400},{0,400} };
        cv::Mat transformMat = cv::getPerspectiveTransform(ver.data(), transformedRect);
        transformMat.convertTo(transformMat, CV_32F);
        return transformMat;
    }

    myCircle getCircleWithCenters(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& tmatrix, std::vector<int> indexes, int circleNum) {
        std::vector<std::vector<cv::Point2f>> circleContours(circleNum);
        for (int i = 0; i < circleNum; i++) {
            circleContours[i] = transform2Dpoints(contours[indexes[i]], tmatrix);
        }
        myCircle circleThroughCenters = findCircleWithCenters(circleContours);
        return circleThroughCenters;
    }


    vector<vector<cv::Point>> getLaserContour(const cv::Mat& frame, const cv::Mat& originalGray, int threshVal) {
        cv::Mat laserImageGray;
        cv::Mat OnlylaserGray;
        cv::Mat laserBinary;
        cv::cvtColor(frame, laserImageGray, cv::COLOR_BGR2GRAY);
        cv::subtract(laserImageGray, originalGray, OnlylaserGray);
        cv::threshold(OnlylaserGray, laserBinary, threshVal, 255, cv::THRESH_BINARY);
        vector<vector<cv::Point>> laserContour;
        cv::findContours(laserBinary, laserContour, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

        /* check image*/
        //cv::imshow("binary", laserBinary);
        //cv::imshow("gray", laserImageGray);

        return laserContour;

    }


    cv::Point2f getTransformedLaserCenter(const vector<cv::Point>& laserContour, const cv::Mat& tmatrix) {
        std::vector<cv::Point2f> transformedContour = transform2Dpoints(laserContour, tmatrix);
        cv::Moments cen = cv::moments(transformedContour);
        float cx = cen.m10 / cen.m00;
        float cy = cen.m01 / cen.m00;
        return cv::Point2f(cx, cy);
    }


    cv::Point2f getOriginalLaserCenter(const cv::Point2f& targetPoint, const cv::Mat& tMat) {
        cv::Point2f OriginalLaserCenterPoint;
        float arr[3] = { targetPoint.x,targetPoint.y,1 };
        cv::Mat laserCenter(3, 1, CV_32F, arr);
        cv::Mat tinv = tMat.inv();
        tinv.convertTo(tinv, CV_32F);
        cv::Mat OriginalLaserCenter = tinv * laserCenter;
        OriginalLaserCenterPoint.x = OriginalLaserCenter.at<float>(0, 0) / OriginalLaserCenter.at<float>(2, 0);
        OriginalLaserCenterPoint.y = OriginalLaserCenter.at<float>(1, 0) / OriginalLaserCenter.at<float>(2, 0);
        return OriginalLaserCenterPoint;
    }


    std::vector<float> distanceInOriginal(cv::Point2f original, cv::Point2f target) {
        std::vector<float> distance(2);
        float distX = original.x - target.x;
        float distY = original.y - target.y;
        distance[0] = distX;
        distance[1] = distY;
        return distance;
    }

    void checkLaser(const cv::Mat& img, int minThreshold) {
        //check the brighest part in the image
        cv::Mat result;
        cv::cvtColor(img, result, cv::COLOR_BGR2GRAY);
        cv::threshold(result, result, minThreshold, 255, cv::THRESH_BINARY);
        cv::imshow("find the brightest part", result);
    }
    double distancePt(cv::Point pt1, cv::Point pt2) {
        cv::Point temp = pt1 - pt2;
        return sqrt(temp.x*temp.x + temp.y*temp.y);
    }

    Circles find_circlesWithRansac(Point2fs& pnts, Mat img ,
        const float min_distance ,
        const int min_rad ,
        const int max_rad ,
        const float epsilon_rad ,
        const float match_threshold ,
        const float min_fitness ,
        const int sample_count ) {
        Circles circles;     //vector<Circle> Circles 
        std::mt19937 rng(std::time(0));
        std::uniform_int_distribution<unsigned long> distr(0, pnts.size());
        for (int i = 0; i < sample_count; i++)
        {
            //random 3 points
         /*   Point2f p0 = pnts[rand() % pnts.size()];
            Point2f p1 = pnts[rand() % pnts.size()];
            Point2f p2 = pnts[rand() % pnts.size()];*/
            Point2f p0 = pnts[distr(rng)];
            Point2f p1 = pnts[distr(rng)];
            Point2f p2 = pnts[distr(rng)];
            while (norm(p1 - p0) < min_distance)p1 = pnts[distr(rng)];
            while (norm(p1 - p2) < min_distance || norm(p2 - p0) < min_distance)p2 = pnts[distr(rng)];
            if (!img.empty()) {
                circle(img, p0, 3, { 0,255,255 });
                circle(img, p1, 3, { 0,255,255 });
                circle(img, p2, 3, { 0,255,255 });
                _show(img);
                _wait();
            }

            Vec2f m1 = (p0 + p1) / 2.0;
            Vec2f m2 = (p1 + p2) / 2.0;
            Vec2f n1 = p1 - p0;
            Vec2f n2 = p2 - p1;
            Matx22f A(n1(0), n1(1), n2(0), n2(1));
            Matx21f b(m1.dot(n1), m2.dot(n2));
            if (determinant(A) != 0)
            {
                Matx21f X = A.inv()*b;
                Point2f center{ X(0), X(1) };
                float R = norm(center - p0);
                //check validity
                if (R < min_rad || R> max_rad)continue;

                float perimeter = CV_2PI * R;
                //scan for match
                int cnt = 0;
                Circle tempCir;
                std::vector<cv::Point2f> inlierPts;
                for (auto&p : pnts) {
                    if (abs(norm(center - p) - R) < epsilon_rad) {
                        cnt++;
                        inlierPts.push_back(p);
                        //check inliers 
                    }
                }

                float fitness = cnt / perimeter;
                //cout << "Fitness=" << fitness << endl;



                if (fitness > min_fitness) // this is a candidate circle
                {
                    if (!img.empty()) {
                        circle(img, center, R, { 0,255,0 });
                        _show(img);
                        _wait();
                    }
                    bool exist = false;
                    for (int k = 0; k < circles.size(); k++) {
                        Circle& cir = circles[k];
                        float match_value = norm(center - cir.C) + abs(R - cir.R);
                        if (match_value < match_threshold) {
                            exist = true;
                            if (fitness > cir.fitness) //replace
                            {
                                cir = { center,R,fitness, cir.match_cnt,inlierPts };
                            }
                            cir.match_cnt++;
                            break;
                        }
                    }
                    if (!exist) {
                        circles.push_back({ center,R,fitness ,0,inlierPts });
                        //cout << "Detected circle: " << center << ", R=" << R << endl;
                    }
                }
            }
        }
        return circles;
    }
}