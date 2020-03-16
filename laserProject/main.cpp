// test branch
//check mounted scene 
#include "laserProjectLIB.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>
#include <stdio.h>
#include <string.h>
#include <tchar.h>
#include "SerialPort.h"
#include <atomic>
#include "MotorController.h"
#include "INAmessage.h"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "ransac_ellipse2d.h"

using namespace cv;

static int stepsX = 0;
static std::atomic<int> stepsY = 0;
static int absStepsX = 0;
static int absStepsY = 0;
static bool finished = false;
// new things to update 
// test with not subtracting images // just the laser on , can I detect laser ?? 
// 

#if 1

//Laser controller class
class LaserRangeFinder {
public:
    ~LaserRangeFinder() { close(); }

    void open(std::string const& portname) {
        std::cout << "Setting up laser range finder..." << std::endl;
        m_connected = m_port.OpenPort((char*)portname.c_str());
        if (m_connected) {
            std::cout << "Connected to " << portname << std::endl;
            m_port.ConfigurePort(CBR_19200, 8, FALSE, NOPARITY, ONESTOPBIT);
            m_port.SetCommunicationTimeouts(0, 0, 0, 0, 0);
            laserOFF();
        }
        else {
            std::cout << "Failed to connecte to port " << portname << std::endl;
        }
    }

    //close device
    void close() { if (m_connected) m_port.ClosePort(); }

    //turn laser on
    void laserON() {
        if (m_connected) {
            m_port.WriteByte('O');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }
    //turn laser off
    void laserOFF() {
        if (m_connected) {
            m_port.WriteByte('C');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    //read laser distance
    double readDistance() {
        //your code here
        if (m_connected) {
            m_port.WriteByte('C');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    bool connected()const { return m_connected; }

protected:
    CSerialPort m_port;
    bool m_connected = false;
};


struct Circle {
    float x, y, r;
    cv::Point center()const { return { (int)x,(int)y }; }
    cv::Point2f centerf()const { return { x,y }; }
};

// Function to find the circle on 
// which the given three points lie 
Circle findCircle(int x1, int y1, int x2, int y2, int x3, int y3)
{
    float r;
    cv::Point2f pt;
    std::vector<cv::Point> arr(3);
    arr[0] = cv::Point(x1, y1);
    arr[1] = cv::Point(x2, y2);
    arr[2] = cv::Point(x3, y3);
    cv::minEnclosingCircle(arr, pt, r);
    return { pt.x,pt.y,r };
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    std::vector<cv::Point> *pt = (vector<cv::Point>*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        pt->push_back(cv::Point(x, y));
        cout <<pt->size()<<"point"<<"(" << x << ", " << y << ") saved" << endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        pt->clear();
        cout << "all the points cleared" << endl;
    }
}

struct model_cnt {
    cv::RotatedRect ellip;
    int counts;
};

bool compareByLength(const model_cnt &a, const model_cnt &b) {
    return a.counts < b.counts;
}

int main() {
    std::vector<cv::Point> userPicked;
    std::vector<model_cnt> models; 
    cv::VideoCapture cap;
    LaserRangeFinder laser;
    /*  laser.open("\\\\.\\COM27");
      Sleep(10);
      if (!cap.open(0))
          return 0;*/
    cv::Mat img;
    cv::Mat roiImg;
    cv::Mat roiImgG;
    img = cv::imread("C:/Users/oggyu/Pictures/Camera Roll/9.jpg", 1);
    cv::Rect2d r = cv::selectROI(img);
    //laser.laserON();
    int cannyMin = 75;
    int cannyMax = 150;
    cv::Mat edge;
    cv::Canny(img, edge, cannyMin, cannyMax);

    edge = edge(r);
    roiImg = img(r);
    cv::cvtColor(roiImg,roiImgG,cv::COLOR_RGB2GRAY);

    cv::resize(edge, edge, cv::Size(0, 0), 2, 2);
    cv::resize(roiImg, roiImg, cv::Size(0, 0), 2, 2);
    cv::resize(roiImgG, roiImgG, cv::Size(0, 0), 2, 2);

    cv::namedWindow("my edges");
    cv::setMouseCallback("my edges", CallBackFunc, &userPicked);
    
    vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(edge);

    //for (int i = 0; i < contour.size(); i++) {
    //    if (contour[i].size() < 50)
    //        continue;
    //    cv::RotatedRect rect = cv::fitEllipse(contour[i]);
    //    //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0));
    //    laserProject::drawWithContourI(contour[i], roiImg, " ");
    //}
    for (int i = contour.size()-1; i>contour.size()-2; i--) {
        cv::RotatedRect rect = cv::fitEllipse(contour[i]);
        //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0),2);
        laserProject::drawWithContourI(contour[i], roiImg, " ");

    }
    cv::SimpleBlobDetector::Params params;
    /*params.minThreshold = 10;
    params.maxThreshold = 200;*/
    params.filterByArea = true;
    params.minArea = 10;

    /*params.filterByCircularity = true;
    params.minCircularity = 0.1;*/

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat im_withKeyPoints;
    cv::Mat edgeWithColor;
    cv::cvtColor(edge, edgeWithColor, cv::COLOR_GRAY2RGB);
    detector->detect(edge, keypoints);
    cv::drawKeypoints(edgeWithColor, keypoints, im_withKeyPoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("keyPoints", im_withKeyPoints);
    cv::waitKey(1);


    // Parameters Settings (Sect. 4.2)
    cv::Size sz = roiImg.size();
    int		iThLength = 16;
    float	fThObb = 3.0f;
    float	fThPos = 1.0f;
    float	fTaoCenters = 0.05f;
    int 	iNs = 16;
    float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;

    float	fThScoreScore = 0.7f;

    // Other constant parameters settings. 

    // Gaussian filter parameters, in pre-processing
  
    //////////////////////////////////////////////////////////////////////
    //connected components testing 
    cv::Mat labelImg;
    cv::Mat stat,centroid;
    std::vector<int> lab_final;
    int nLabels = cv::connectedComponentsWithStats(edge, labelImg, stat, centroid, 8);
    std::vector<cv::Vec3b> colors(nLabels);
    colors[0] = cv::Vec3b(0, 0, 0);

    for (int lab = 1; lab < nLabels; lab++) {
        if (stat.at<int>(lab, cv::CC_STAT_AREA) > 10)
            lab_final.push_back(lab);
        colors[lab] = cv::Vec3b(rand()%255, rand()%255, rand()%255);
    }
    
    cv::Mat dst(roiImg.size(), CV_8UC3);
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {

            int label = labelImg.at<int>(r, c);
            //cout << "label:  " << label << endl; 
            cv::Vec3b &pixel = dst.at<cv::Vec3b>(r, c);//accesa al elemento 
            pixel = colors[label];
        }
    }
//////////////////////////////////////////////////////////////////////////////////////

    Mat inImg, showMat;
    cvtColor(roiImgG, showMat, COLOR_GRAY2RGB);
    //threshold(roiImgG, inImg, 10, 255, THRESH_BINARY_INV);
    inImg = edge;

    std::vector<sac::Point2D> pCloud2D;
    for (int i = 0; i < inImg.rows; i++)
    {
        uchar* p = inImg.ptr<uchar>(i);
        for (int j = 0; j < inImg.cols; j++)
        {
            if (p[j] != 0)
                pCloud2D.push_back(sac::Point2D(j, i));
        }
    }

    sac::ransacModelEllipse2D ellipse2D;
    std::vector<int> inliers;
    sac::ModelCoefficient parameter;
    ellipse2D.setDistanceThreshold(3);
    ellipse2D.setMaxIterations(2500);
    ellipse2D.setSpecficAxisLength(150, 130, 0.3);
    //ellipse2D.setSpecficAxisLength(135, 85, 0.5);

    while (pCloud2D.size() > 500)
    {
        cout << pCloud2D.size() << endl;
        ellipse2D.setInputCloud(pCloud2D);
        ellipse2D.computeModel();
        ellipse2D.getInliers(inliers);
        ellipse2D.getModelCoefficients(parameter);

        cout << inliers.size() << endl;
        
        cv::Point2f ellipseCenter;
        ellipseCenter.x = (float)parameter.modelParam[0];
        ellipseCenter.y = (float)parameter.modelParam[1];
        cv::Size2f ellipseSize;
        ellipseSize.width = (float)parameter.modelParam[2] * 2;
        ellipseSize.height = (float)parameter.modelParam[3] * 2;

        float ellipseAngle = (float)parameter.modelParam[4];
        cout << "Parameters of ellipse2D: < " << parameter.modelParam[0] << ", " <<
            parameter.modelParam[1] << " > --- ";
        cout << "Long/Short Axis: " << parameter.modelParam[2] << "/" << parameter.modelParam[3] << " --- ";
        cout << "Angle: " << parameter.modelParam[4] << endl;

        cv::ellipse(showMat, cv::RotatedRect(ellipseCenter, ellipseSize, ellipseAngle), cv::Scalar(0, 255, 0), 2, 8);

        imshow("ellipses", showMat);
        waitKey(12);

        ellipse2D.removeInliders(pCloud2D, inliers);
    }



    cv::imshow("edge with component connected ", dst);

    cv::imshow("roi Img", roiImg);
    cv::imshow("my edges",edge);
    cv::waitKey(0);
    
    ///////////////////////////////////////////////////////////////////////// 

    //select contour for image

    // user picking part 
    /*while (userPicked.size() < 5) {
        cv::waitKey(10);
        if (userPicked.size() >= 5) {
            std::cout << "more than 5 picked " << endl;
            break;
        }
    }

    cv::RotatedRect myellipse = cv::fitEllipse(userPicked);
    cv::ellipse(roiImg, myellipse, cv::Scalar(0, 250, 0),2);
   
    cv::imshow("roi Img", roiImg);*/

    // user picking end 

    ////////////////////////////////////////////////////////////////////////
    //ellipse ransac 

    //std::vector<cv::Point> ps; 
    //cv::findNonZero(edge, ps);
    //const int N = 1000;
    //const double min_dist = 5;
    //const float eps = 1;
    ////const float min_coverage = 0.9;
    //for (int i = 0; i < N; i++) {
    //    //select 3 points randomly
    //    cv::Point p1, p2, p3, p4, p5;
    //    std::vector<cv::Point> ellipPt(5);
    //    ellipPt[0] = ps[rand() % ps.size()];
    //    ellipPt[1] = ps[rand() % ps.size()];
    //    ellipPt[2] = ps[rand() % ps.size()];
    //    ellipPt[3] = ps[rand() % ps.size()];
    //    ellipPt[4] = ps[rand() % ps.size()];

    //    cv::RotatedRect myEllipse = cv::fitEllipse(ellipPt);

    //    ////count how many points on this circle
    //    int cnt = 0;
    //    float phi = myEllipse.angle;
    //    cv::Point2f cp = myEllipse.center;
    //    float width = myEllipse.size.width;
    //    float height = myEllipse.size.height;

    //    for (int i = 0; i < ps.size(); i++) {
    //        float error = (pow((ps[i].x - cp.x)*cos(phi) + (ps[i].y - cp.y)*sin(phi), 2) / pow(width, 2)) + pow((ps[i].x - cp.x)*sin(phi) - (ps[i].y - cp.y)*cos(phi), 2) / pow(height, 2) - 1;
    //        if (error <= eps) {
    //            cnt++;
    //        }
    //    }
    //    model_cnt mod{ myEllipse, cnt };
    //    models.push_back(mod);
    //}
    ////std::sort(models.begin(), models.end(), compareByLength);
    //int maxIndex = 0; 
    //int max = models[0].counts;
    //
    //for (int i = 1; i < models.size()-1; i++) {
    //    if (models[i].counts > max) {
    //        maxIndex = i;
    //        max = models[i].counts;
    //    }
    //}
    //cv::ellipse(roiImg, models[maxIndex].ellip, cv::Scalar(0, 255, 0),2);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
   


    //for (;;) {
    //    cv::Mat imgNoLaser;
    //    cv::Mat imgWithLaser;
    //    cv::Mat gray;
    //    cv::Mat img;
    //    cv::Mat rImg;
    //    cv::Mat gaussian;
    //    cv::Mat cany;
    //    cv::Mat binary;

    //    //laser.laserON();
    //    //Sleep(200);
    //    //cap >> imgWithLaser;
    //    //laser.laserOFF();
    //    //Sleep(200);

    //    cap >> imgNoLaser;
    //    rImg = imgNoLaser(r);



    //    cv::resize(rImg, rImg, cv::Size(0, 0),2,2);

    //    cv::cvtColor(rImg, gray, cv::COLOR_RGB2GRAY);

    //           int numOfContour = 6;
    //    int contourSize = 20;
    //    int contourArea = 100;

    //    cv::Canny(gray, cany, cannyMin, cannyMax);
    //    imshow("canny", cany);
    //    cv::waitKey(1);
    //   
    //    cv::GaussianBlur(gray, gaussian,cv::Size(9,9),0,0);
    //    cv::adaptiveThreshold(gaussian, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,9,2);
    //     vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(cany);
    //    vector<int> indexes = laserProject::findCircleContoursIndexes(contour, numOfContour, contourSize, contourArea);

    //    //for (int i = 0; i < indexes.size(); i++) {
    //    //    laserProject::drawWithContourI(contour[indexes[i]], rImg, " ");
    //    //}

    //    int count = 0;
    //    for (int i = 0; i < contour.size(); i++) {
    //        if (contour[i].size() > 50) {
    //            count++;
    //            laserProject::drawWithContourI(contour[i], rImg, " ");
    //        }
    //    }
    //    cv::Mat binResult;
    //    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    //    //cv::erode(binary, binResult, element);
    //    //cout<<"number of contour" << contour.size() << endl;
    //    //cout << "contour size over 30 : " << count <<endl;


    //    //extract all points
    //    vector<cv::Point> ps;
    //    const int minConSize = 50;
    //    for (auto &con : contour) {
    //        if (con.size() < minConSize)
    //            continue;
    //        ps.insert(ps.end(), con.begin(), con.end());
    //    }

    //    //ransac
    //    //const int N = 10000;
    //    //const double min_dist = 5;
    //    //const float eps = 3;
    //    //const float min_coverage = 0.9;
    //    //for (int i = 0; i < N; i++) {
    //    //    //select 3 points randomly
    //    //    cv::Point p1, p2, p3;
    //    //    p1 = ps[rand() % ps.size()];
    //    //    p2 = ps[rand() % ps.size()];
    //    //    p3 = ps[rand() % ps.size()];
    //    //    while (true) {
    //    //        break;
    //    //        p2 = ps[rand() % ps.size()];
    //    //        p3 = ps[rand() % ps.size()];
    //    //        auto v1 = p2 - p1;
    //    //        auto v2 = p3 - p1;
    //    //        if (cv::norm(p1 - p2) > min_dist &&
    //    //            cv::norm(p1 - p3) > min_dist &&
    //    //            cv::norm(p2 - p3) > min_dist) {
    //    //            break;
    //    //        }
    //    //    }
    //    //    //compute circle 
    //    //    auto cir = findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

    //    //    //count how many points on this circle
    //    //    int cnt = std::count_if(ps.begin(), ps.end(), [&](cv::Point const& p)->bool {
    //    //        return std::abs(cv::norm(p - cir.center()) - cir.r) < eps;
    //    //    });

    //    //    float coverage = cnt / (2 * CV_PI*cir.r);
    //    //    if (coverage > min_coverage && coverage < 1) {
    //    //        //draw 
    //    //        cv::circle(rImg, cir.center(), cir.r, { 0,255,0 }, 1);
    //    //        std::cout <<"coverage : "<<coverage << endl;
    //    //        std::cout << "total Points : " << ps.size() << endl;
    //    //        std::cout << "count : " << cnt << endl;

    //    //    }
    //    //}

    //    cv::imshow("color", rImg);
    //    cv::waitKey(1);
    //    cv::imshow("bin", binary);
    //    //cv::imshow("eroded IMG", binResult);


    //    if (cv::waitKey(1) == 27) break;

    //}


}

#else

#endif
