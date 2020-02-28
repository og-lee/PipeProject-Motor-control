
#include <iostream>
#include "laserProjectLIB.h"


namespace laserProject {
	void printMes() {
		std::cout << "hello"; 
	}
	cv::Mat takeImg(cv::VideoCapture capture) {
		cv::Mat frame;
		if (capture.isOpened()) {
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
	vector<int> findCircleContours(vector<vector<cv::Point>> const &contour,cv::Mat const &originalImg,int numOfContour,int contourSize,int contourArea) {
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
        }
        catch (int x)
        {
            cout << "number of holes are not accurate" << endl;
        }
        catch (...) {
            cout << "default exception" << endl; 
        }
		return index;             
    }

    // checks the circularity of contour 
    bool checkCircularity(vector<cv::Point>const &contour) {
        double contourArea = cv::contourArea(contour);
        double contourPerimeter = cv::arcLength(contour,TRUE);
        double circularity = (4.0 * M_PI * contourArea) / (pow(contourPerimeter,2));
        if (circularity > 0.8 && contourArea > 100)
            return true;

        else
            return false;
    }
        
    // indexes is the indexes of 6 ellipses found 
    // contours are the all the contours found in the image
    std::vector<cv::Point2f>  findBiggestEllipse(vector<vector<cv::Point>>const& contours,const vector<int>& indexes){

        std::vector<cv::Point2f> ver(4);
        //find the biggest ellipse 
        int maxEllipseIndex = 0;
        int maxVal = 0;
        for (int i : indexes) {
            if (contours[i].size() > maxVal) {
                maxEllipseIndex = i;
                maxVal = contours[i].size();
            }
        }
        // 
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

    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point> const& contour,cv::Mat const& T) {
        //hCoordPoint is the (x,y,1) of laserContours before transformed
        std::vector<cv::Point3f> hCoordPoint = makeHomogeneous(contour);
        std::vector<cv::Point2f> transformedPoints(contour.size()) ;

        cv::Mat laserConMat(contour.size(), 3, CV_32F, hCoordPoint.data());
        cv::transpose(laserConMat, laserConMat);
        cv::Mat m(T * laserConMat);

        for (int i = 0; i < contour.size(); i++) {
            transformedPoints[i].x = m.at<float>(0,i) / m.at<float>(2,i);
            transformedPoints[i].y = m.at<float>(1,i) / m.at<float>(2,i);
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

    
    
    void drawWithContour(const std::vector<cv::Point2f>& pt,const cv::Mat& img,const char* str) {
        for (int i = 0; i < pt.size(); i++) {
            cv::circle(img, pt[i], 1, cv::Scalar(0, 100, 250), 2);
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
            circPoint2f[i].x = circ.center.x + circ.radius*(float)cos(2*M_PI*i / numberOfPoints);
            circPoint2f[i].y = circ.center.y + circ.radius*(float)sin(2*M_PI*i / numberOfPoints);
        }
        return circPoint2f;
    }

    cv::Mat thresholdBlurredImage(const cv::Mat& original, int minThresh) {
        cv::Mat originalGray;
        cv::Mat blurredOriginal;
        cv::Mat thresholdImg;
        cv::cvtColor(original, originalGray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(originalGray, blurredOriginal,cv::Size(5,5), 0, 0);
        cv::threshold(blurredOriginal, thresholdImg, minThresh,255,cv::THRESH_BINARY);
        return thresholdImg;
    }
   
}