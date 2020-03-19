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
#include "lsdlines.h"
#include <opencv2/line_descriptor/descriptor.hpp>
#include <random>
#include "Circle.h"
using namespace cv;
using namespace line_descriptor;

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


//struct Circle {
//    float x, y, r;
//    cv::Point center()const { return { (int)x,(int)y }; }
//    cv::Point2f centerf()const { return { x,y }; }
//};

// Function to find the circle on 
// which the given three points lie 
//Circle findCircle(int x1, int y1, int x2, int y2, int x3, int y3)
//{
//    float r;
//    cv::Point2f pt;
//    std::vector<cv::Point> arr(3);
//    arr[0] = cv::Point(x1, y1);
//    arr[1] = cv::Point(x2, y2);
//    arr[2] = cv::Point(x3, y3);
//    cv::minEnclosingCircle(arr, pt, r);
//    return { pt.x,pt.y,r };
//}

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


typedef vector<Circle> Circles;

bool compareByInlierCircle(const Circle &a, const Circle &b) {
    return a.inliers.size() > b.inliers.size();
}
bool compareByFitnessCircle(const Circle &a, const Circle &b) {
    return a.fitness > b.fitness;
}

void _show(Mat img) {
    cv::imshow("show Img", img);
}
void _wait() {
    cv::waitKey(1);
}
Circles find_circles(Point2fs& pnts, Mat img = cv::Mat(),
    const float min_distance = 40,
    const int min_rad = 100,
    const int max_rad = 200,
    const float epsilon_rad = 1.5,
    const float match_threshold = 15,
    const float min_fitness = 0.5,
    const int sample_count = 1000) {
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
                            cir = { center,R,fitness, cir.match_cnt,inlierPts};
                        }
                        cir.match_cnt++;
                        break;
                    }
                }
                if (!exist) {
                    circles.push_back({ center,R,fitness ,0,inlierPts});
                    //cout << "Detected circle: " << center << ", R=" << R << endl;
                }
            }
        }
    }
    return circles;
}


int main() {
    std::vector<cv::Point> userPicked;
    cv::VideoCapture cap;
    LaserRangeFinder laser;
    /*  laser.open("\\\\.\\COM27");
      Sleep(10);
      if (!cap.open(0))
          return 0;*/
    cv::Mat img;
    cv::Mat roiImg;
    cv::Mat roiImgG;
    img = cv::imread("C:/Users/oggyu/Pictures/Camera Roll/1.jpg", 1);
    cv::Rect2d r = cv::selectROI(img);
    //laser.laserON();
    int cannyMin = 100;
    int cannyMax = 150;
    cv::Mat edge;
    cv::Mat threshImg;
    cv::Canny(img, edge, cannyMin, cannyMax);

    edge = edge(r);
    roiImg = img(r);
    cv::cvtColor(roiImg,roiImgG,cv::COLOR_RGB2GRAY);
    //cv::GaussianBlur(roiImgG, threshImg,Size(3,3),1);
    //cv::medianBlur(roiImgG, threshImg, 5);
    //cv::adaptiveThreshold(roiImgG, threshImg, 255, ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV,3,5);

    int scale = 2;
    cv::resize(edge, edge, cv::Size(0, 0), scale, scale);
    cv::resize(roiImg, roiImg, cv::Size(0, 0), scale, scale);
    cv::resize(roiImgG, roiImgG, cv::Size(0, 0), scale, scale);
    //cv::resize(threshImg, threshImg, cv::Size(0, 0), scale, 2);

    cv::namedWindow("my edges");
    cv::setMouseCallback("my edges", CallBackFunc, &userPicked);
    cv::imshow("my edges",edge);
    //imshow("thresh", threshImg); 

    cv::waitKey(1);
    
    //vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(edge);

    //for (int i = 0; i < contour.size(); i++) {
    //    if (contour[i].size() < 50)
    //        continue;
    //    cv::RotatedRect rect = cv::fitEllipse(contour[i]);
    //    //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0));
    //    laserProject::drawWithContourI(contour[i], roiImg, " ");
    //}
    //for (int i = contour.size()-1; i>contour.size()-2; i--) {
    //    cv::RotatedRect rect = cv::fitEllipse(contour[i]);
    //    //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0),2);
    //    laserProject::drawWithContourI(contour[i], roiImg, " ");
    //}
    //imshow("roi", roiImg);
    //cv::waitKey(1);
//
    //// make matrix to array 
    //
    //Mat showColor;
    //cvtColor(roiImgG, showColor, COLOR_GRAY2RGB);
    //Ptr<LSDDetector> bd = LSDDetector::createLSDDetector();
    //std::vector<KeyLine> lines;
    //std::vector<Vec4f> lines1;
    //cv::Mat mask = cv::Mat::ones(roiImgG.size(),CV_8UC1);
    ////bd->detect(roiImgG, lines,1,1,mask);
    //bd->detect(roiImgG, lines,1,1,mask);
    //  
//
    //std::vector<vector<cv::Point>> points(lines.size());
    //for (size_t i = 0; i < lines.size(); i++)
    //{
    //    KeyLine kl = lines[i];
    //    if (kl.octave == 0 )
    //    {
    //        Point pt1 = Point2f(kl.startPointX, kl.startPointY);
    //        Point pt2 = Point2f(kl.endPointX, kl.endPointY);
//
    //        cv::LineIterator it(roiImgG, pt1, pt2, 8);
    //        points[i].resize(it.count);
    //        for (int j = 0; j < it.count; j++, it++) {
    //            points[i][j] = it.pos();
    //        }
    //        line(showColor, pt1, pt2, Scalar(rand()%255,rand()%255,rand()%255), 2);
    //    }
//
    //}
    //  
    //cv::SimpleBlobDetector::Params params;
    //*params.minThreshold = 10;
    //params.maxThreshold = 200;*/
    //params.filterByArea = true;
    //params.minArea = 10;
    //
    //imshow("roiImg", roiImgG);
    //cv::waitKey(1);
    //*params.filterByCircularity = true;
    //params.minCircularity = 0.1;*/
    //
    //cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    //std::vector<cv::KeyPoint> keypoints;
    //cv::Mat im_withKeyPoints;
    //cv::Mat edgeWithColor;
    //cv::cvtColor(edge, edgeWithColor, cv::COLOR_GRAY2RGB);
    //detector->detect(edge, keypoints);
    //cv::drawKeypoints(edgeWithColor, keypoints, im_withKeyPoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    std::vector<cv::Point2f> ps;
    std::vector<Circle> circ;
    cv::findNonZero(edge, ps);
    int minDis = 100;
    int minR = 200;
    int maxR = 400;
    float pixDis = 1;
    float matchThresh = 15;
    float fitness = 0.5;
    int iter = 3000;
    circ = find_circles(ps,roiImg,minDis,minR,maxR,pixDis,matchThresh,fitness);

    //std::sort(circ.begin(), circ.end(), compareByFitnessCircle);
    std::sort(circ.begin(), circ.end(), compareByInlierCircle);

    Mat edgeWithColor;
    Mat edgeWithEllipse;
    cv::cvtColor(edge, edgeWithColor, COLOR_GRAY2RGB);
    edgeWithColor.copyTo(edgeWithEllipse);
    std::vector<cv::Point2f> circleInliers;
    int numOfCircleToSample = circ.size();
    //int numOfCircleToSample = 5;
    for (int i = 0; i < numOfCircleToSample; i++) {
        circleInliers.insert(circleInliers.end(), circ[i].inliers.begin(), circ[i].inliers.end());
        cv::circle(edgeWithColor, circ[i].C, circ[i].R,Scalar(0,255,0));
    }
    cv::RotatedRect ellipResult = fitEllipse(circleInliers);
    cv::ellipse(edgeWithEllipse, ellipResult, cv::Scalar(0, 255, 155), 2);
    cv::imshow("detectedCircle",edgeWithColor);
    cv::imshow("detectedEllipse",edgeWithEllipse);
    cv::waitKey(0);


    // Parameters Settings (Sect. 4.2)
       // Other constant parameters settings. 

    // Gaussian filter parameters, in pre-processing
  
        

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

    //cv::Mat showCol;
    //cvtColor(edge, showCol, cv::COLOR_GRAY2RGB);
    //std::vector<cv::Point> ps; 
    //cv::findNonZero(edge, ps);
    //const int N = 1000;
    //const double minDistance = 100;
    //const float eps = 2;
    ////const float min_coverage = 0.9;
    //for (int i = 0; i < N; i++) {
    //    //select 3 points randomly
    //    cout << i <<" :  iteration start" << endl; 
    //    cv::Point p1, p2, p3, p4, p5;
    //    std::vector<cv::Point> ellipPt(5);
    //    ellipPt[0] = ps[rand() % ps.size()];
    //    ellipPt[1] = ps[rand() % ps.size()];
    //    ellipPt[2] = ps[rand() % ps.size()];
    //    ellipPt[3] = ps[rand() % ps.size()];
    //    ellipPt[4] = ps[rand() % ps.size()];
    //    while (1) {
    //        if (laserProject::distancePt(ellipPt[0], ellipPt[1]) < minDistance ||
    //            laserProject::distancePt(ellipPt[0], ellipPt[2]) < minDistance ||
    //            laserProject::distancePt(ellipPt[0], ellipPt[3]) < minDistance ||
    //            laserProject::distancePt(ellipPt[0], ellipPt[4]) < minDistance ||
    //            laserProject::distancePt(ellipPt[1], ellipPt[2]) < minDistance ||
    //            laserProject::distancePt(ellipPt[1], ellipPt[3]) < minDistance ||
    //            laserProject::distancePt(ellipPt[1], ellipPt[4]) < minDistance ||
    //            laserProject::distancePt(ellipPt[2], ellipPt[3]) < minDistance ||
    //            laserProject::distancePt(ellipPt[2], ellipPt[4]) < minDistance ||
    //            laserProject::distancePt(ellipPt[3], ellipPt[4]) < minDistance) {
    //            ellipPt[0] = ps[rand() % ps.size()];
    //            ellipPt[1] = ps[rand() % ps.size()];
    //            ellipPt[2] = ps[rand() % ps.size()];
    //            ellipPt[3] = ps[rand() % ps.size()];
    //            ellipPt[4] = ps[rand() % ps.size()];
    //        }
    //        break;
    //     }
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
    //std::sort(models.begin(), models.end(), compareByLength);
    ////int maxIndex = 0; 
    ////int max = models[0].counts;
    ////
    //for (int i = models.size()-1; i > models.size()-10; i--) {
    //    cv::ellipse(showCol, models[i].ellip, cv::Scalar(0, 255, 0),2);
    //}
    //imshow("ellipse", showCol);
    //cv::waitKey(0);
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
