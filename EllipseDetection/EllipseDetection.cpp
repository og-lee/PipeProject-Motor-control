// EllipseDetection.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
#define _show(img) cv::imshow(#img, img)


//you need some timing mechanism using RAII


class PerformanceTimer {
public:
    PerformanceTimer(string const& msg) {
        m_msg = msg;
        //When the timer is constructed, we mark the starting time
        m_t = clock();
    }
    ~PerformanceTimer() {
        //When the timer gets destroyed BECAUSE going out of scope,
        // measure duration and print out
        auto dt = clock() - m_t;
        cout << m_msg << ": " << (1000.0*dt / CLOCKS_PER_SEC) << "ms" << endl;
    }
private:
    std::string m_msg;
    clock_t m_t;
};


#define CLAMP_(x, min_value, max_value) x=x>max_value?max_value:(x<min_value?min_value:x)

struct Slider {
    cv::Rect2i bounds;
    std::string name;
    int min_value, max_value;
    int value = 50;
    cv::Scalar backColor = { 0, 0, 0 }; //black
    cv::Scalar foreColor = { 0, 255, 0 }; //green 
    cv::Scalar borderColor = { 255, 0, 255 }; //magenta


    inline bool contains(int x, int y)const { return bounds.contains({ x,y }); }
    inline bool contains(cv::Point2i const& p)const { return bounds.contains(p); }
    inline bool contains(cv::Point2f const& p)const { return bounds.contains(p); }

    inline int x() const { return bounds.x; }
    inline int y() const { return bounds.y; }
    inline int width() const { return bounds.width; }
    inline int height() const { return bounds.height; }
    inline int left() const { return bounds.x; }
    inline int rigth() const { return bounds.x + bounds.width; }
    inline int top() const { return bounds.y; }
    inline int bottom() const { return bounds.y + bounds.height; }
    inline int range()const { return max_value - min_value; }

    void draw(cv::Mat & img) {
        //draw background
        cv::rectangle(img, bounds, backColor, -1);
        //fill rect
        //make sure value is in [0,100]
        CLAMP_(value, min_value, max_value);
        int fill_width = value * bounds.width / (max_value - min_value);
        cv::rectangle(img, { bounds.x, bounds.y, fill_width, bounds.height }, foreColor, -1);
        //draw border
        cv::rectangle(img, bounds, borderColor, 1);
        char txt[100];
        sprintf_s(txt, "%s: %d", name.c_str(), value);
        cv::putText(img, txt, { bounds.br() }, cv::FONT_HERSHEY_DUPLEX, 0.5, { 255,0,255 }, 1);
    }
};

struct MyData {
    Mat *img = nullptr;
    std::vector<Slider*> sliders;

    cv::Rect2i rect;
    int click = 0;
};

#define RGB_()  cv::Scalar(rand()%256,rand()%256,rand()%256)


void mouse_callback(
    int event,      //mouse event
    int x,          //
    int y,
    int flags,
    void* data) {

    auto *d = (MyData*)data;
    if (event == EVENT_LBUTTONUP) {
        //if i click on my slider, i need to change my slider value
        //and ignore selection
        for (auto& slider : d->sliders) {
            //mouse on slider
            if (slider->contains(x, y)) {
                //compute value
                slider->value = ((x - slider->left()) * (slider->range())) / slider->width();
                return;
            }
        }

        if (d->click == 0) {
            d->rect = { x,y,0,0 };
            d->click = 1;
        }
        else d->click = 0;
    }
    else if (event == EVENT_LBUTTONUP) {
        d->rect = { x,y,0,0 };
    }

    else if (event == EVENT_MOUSEMOVE) {
        if (d->click == 1) {
            d->rect.width = x - d->rect.x;
            d->rect.height = y - d->rect.y;
        }
    }
}


using Contour = vector<Point>;
using Contours = vector<Contour>;


Contours myFindContours(Mat const& input_img) {
    Mat gimg = input_img.clone();

    Contours contours;
    //scan all pixel
    uchar b[8];
    //paint border black
    cv::line(gimg, { 0,0 }, { 0,gimg.rows - 1 }, cv::Scalar::all(0), 1);
    cv::line(gimg, { 0,0 }, { gimg.cols - 1,0 }, cv::Scalar::all(0), 1);
    cv::line(gimg, { gimg.cols - 1,0 }, { gimg.cols - 1,gimg.rows - 1 }, cv::Scalar::all(0), 1);
    cv::line(gimg, { 0,gimg.rows - 1 }, { gimg.cols - 1,gimg.rows - 1 }, cv::Scalar::all(0), 1);

    bool has_segment = false;
    do {
        has_segment = false;

        for (int y = 1; y < gimg.rows - 1; y++) {
            auto ptr = gimg.ptr<uchar>(y);
            auto tptr = gimg.ptr<uchar>(y - 1);
            auto bptr = gimg.ptr<uchar>(y + 1);
            for (int x = 1; x < gimg.cols - 1; x++) {
                //we need to find the end point and track along
                //the end point must be white pixel
                if (ptr[x] > 0) {
                    //the end-point must have exactly 2 change
                    b[0] = tptr[x - 1];
                    b[1] = tptr[x];
                    b[2] = tptr[x + 1];
                    b[3] = ptr[x + 1];
                    b[4] = bptr[x + 1];
                    b[5] = bptr[x];
                    b[6] = bptr[x - 1];
                    b[7] = ptr[x - 1];
                    //compute sum diff
                    int sum = 0;

                    sum += (b[0] - b[1]) != 0 ? 1 : 0;
                    sum += (b[1] - b[2]) != 0 ? 1 : 0;
                    sum += (b[2] - b[3]) != 0 ? 1 : 0;
                    sum += (b[3] - b[4]) != 0 ? 1 : 0;
                    sum += (b[4] - b[5]) != 0 ? 1 : 0;
                    sum += (b[5] - b[6]) != 0 ? 1 : 0;
                    sum += (b[6] - b[7]) != 0 ? 1 : 0;
                    sum += (b[7] - b[0]) != 0 ? 1 : 0;

                    int crrx = x;
                    int crry = y;
                    if (sum == 2) {
                        has_segment = true;
                        //start new contour
                        Contour new_contour;
                        for (;;) {
                            //trace along
                            //search for next current_point in the neibour
                            new_contour.emplace_back(crrx, crry);

                            //_show(gimg);
                            //waitKey(1);
                            auto fn_cleanup = [&gimg](int crrx, int crry) {
                                //gimg.at<uchar>(crry - 1, crrx - 1) = 0;
                                //gimg.at<uchar>(crry - 1, crrx) = 0;
                                //gimg.at<uchar>(crry - 1, crrx + 1) = 0;
                                //gimg.at<uchar>(crry, crrx - 1) = 0;
                                gimg.at<uchar>(crry, crrx) = 0;
                                //gimg.at<uchar>(crry, crrx + 1) = 0;
                                //gimg.at<uchar>(crry + 1, crrx - 1) = 0;
                                //gimg.at<uchar>(crry + 1, crrx) = 0;
                                //gimg.at<uchar>(crry + 1, crrx + 1) = 0;
                            };

                            if (gimg.at<uchar>(crry - 1, crrx - 1)) { fn_cleanup(crrx, crry);  crry = crry - 1; crrx = crrx - 1; }
                            else if (gimg.at<uchar>(crry - 1, crrx)) { fn_cleanup(crrx, crry); crry = crry - 1; crrx = crrx; }
                            else if (gimg.at<uchar>(crry - 1, crrx + 1)) { fn_cleanup(crrx, crry); crry = crry - 1; crrx = crrx + 1; }

                            else if (gimg.at<uchar>(crry, crrx - 1)) { fn_cleanup(crrx, crry); crry = crry; crrx = crrx - 1; }
                            else if (gimg.at<uchar>(crry, crrx + 1)) { fn_cleanup(crrx, crry); crry = crry; crrx = crrx + 1; }

                            else if (gimg.at<uchar>(crry + 1, crrx - 1)) { fn_cleanup(crrx, crry); crry = crry + 1; crrx = crrx - 1; }
                            else if (gimg.at<uchar>(crry + 1, crrx)) { fn_cleanup(crrx, crry); crry = crry + 1; crrx = crrx; ; }
                            else if (gimg.at<uchar>(crry + 1, crrx + 1)) { fn_cleanup(crrx, crry); crry = crry + 1; crrx = crrx + 1; }
                            else break;
                        }

                        contours.emplace_back(std::move(new_contour));
                    }
                    else if (sum == 0) {
                        ptr[x] = 0;
                    }
                }
            }
        }

    } while (has_segment && cv::sum(gimg)[0] > 0);
    return contours;
}


int main()
{

    srand(clock());
    vector<cv::Scalar> colors(256);
    for (int i = 0; i < colors.size(); i++)colors[i] = RGB_();

    Mat rawimg, img, roi_img;
    Mat gimg, cimg, timg;
    VideoCapture cap(0);

    int yy = 0;
    Slider slider_canny_min{ { 5,5 + (yy++) * 10,200,10 }, "CannyMin" , 0, 255, 50 };
    Slider slider_canny_max{ { 5,5 + (yy++) * 10,200,10 }, "CannyMax" ,0,255, 150 };
    Slider slider_min_contour_size{ { 5,5 + (yy++) * 10,200,10 }, "MinContourLen" ,20, 300,100 };
    Slider slider_curvature_winsz{ { 5,5 + (yy++) * 10,200,10 }, "CurvatureWinSz" ,1, 20,5 };
    Slider slider_max_alpha{ { 5,5 + (yy++) * 10,200,10 }, "MaxAlpha" ,0,90 ,30 };

    MyData data{ &img,
        {
            &slider_canny_min,
            &slider_canny_max,
            &slider_min_contour_size,
            &slider_curvature_winsz,
            &slider_max_alpha,
        }
    };

    namedWindow("img");
    setMouseCallback("img", mouse_callback, &data);


    for (;;) {
        cout << "---------------------------------" << endl;
        PerformanceTimer total("Total time");
        //
        {
            PerformanceTimer pt("Capture time");
            cap >> rawimg;
            if (rawimg.empty())continue;
            img = rawimg.clone();

            //draw selection rectangle
            cv::rectangle(img, data.rect, { 0,255,0 }, 2);
            //draw all slider
            for (auto& slider : data.sliders) { slider->draw(img); }


            //extract roi img , rectt must be big enough
            if (data.rect.height > 20 && data.rect.width > 20) {
                roi_img = rawimg(data.rect);
            }
        }

        if (!roi_img.empty()) {
            //
            Contours contours;
            {
                PerformanceTimer pt("Pre processing time");
                //convert image to gray
                cv::cvtColor(roi_img, gimg, COLOR_RGB2GRAY);
                //blur
                cv::blur(gimg, gimg, { 3,3 });
                //canny
                cv::Canny(gimg, cimg, slider_canny_min.value, slider_canny_max.value);
            }
            //================================================================================
#if 1
            {
                PerformanceTimer pt("Contour extraction");
                //contours
                cv::findContours(cimg, contours, RETR_LIST, CHAIN_APPROX_NONE);

            }
#else 

            {
                PerformanceTimer pt("My Contour extraction");
                //contours
                contours = myFindContours(cimg);

            }
#endif

            //================================================================================
            {
                PerformanceTimer pt("Contour filtering");
                //Filter small contours
                {
                    //visualize all contours
                    //filter out small contours
                    //sort
                    std::sort(contours.begin(), contours.end(),
                        [/*capture*/](Contour const& c1, Contour const& c2) {
                        return c1.size() > c2.size();
                    });

                    //we want to remove all contours with size <100
                    //find the first element with size <100
                    auto first_small_contour_it = std::find_if(contours.begin(), contours.end(),
                        [&slider_min_contour_size](Contour const& c) {
                        return c.size() < slider_min_contour_size.value;
                    });
                    //remove all small contours
                    contours.resize(std::distance(contours.begin(), first_small_contour_it));
                }

                //convert cordinate to original image coords
                for (auto& contour : contours) {
                    for (auto& p : contour) {
                        p += data.rect.tl();
                    }
                }


#if 0
                //Contours new_contours;
                //we need to remove extra points
                for (auto& contour : contours) {
                    Contour new_contour;
                    auto &prev_p = contour[0];
                    new_contour.push_back(prev_p);
                    for (auto& p : contour) {
                        if (cv::norm(p - prev_p) < slider_curvature_winsz.value)continue;
                        new_contour.push_back(p);
                    }
                    new_contours.push_back(new_contour);
                }


#else
                Contours new_contours;
                const int cur_winsz = slider_curvature_winsz.value;
                //break into smooth pieces
                for (auto& contour : contours) {
                    //compute curvature of this contour
                    //create new contour
                    Contour *con = nullptr;
                    //we need to track angle, and side
                    double prev_alpha = 999.0;
                    double prev_side = 0;
                    for (int i = cur_winsz; i < contour.size() - cur_winsz; i += cur_winsz) {
                        auto & p1 = contour[i - cur_winsz];
                        auto & p2 = contour[i];
                        auto & p3 = contour[i + cur_winsz];
                        auto v1 = p2 - p1;
                        auto v2 = p3 - p2;
                        //compute angle
                        double cos_alpha = v1.dot(v2) / cv::norm(v1) / cv::norm(v2);
                        CLAMP_(cos_alpha, -0.99, 0.99);
                        double alpha = std::acos(cos_alpha) * 180.0 / CV_PI;
                        double side = v1.cross(v2);
                        //first calculation --> create new contour

                        //check if same side
                        if (
                            (side * prev_side >= 0 || abs(side) < 3) &&
                            abs(alpha - prev_alpha) < slider_max_alpha.value)
                        {
                            if (con->empty())con->push_back(p1);
                            con->push_back(p2);
                        }
                        else {
                            if (con != nullptr) con->push_back(p3);
                            new_contours.emplace_back();
                            con = &new_contours.back();
                            con->push_back(p2);
                        }
                        //store next
                        prev_alpha = alpha;
                        prev_side = side;
                    }
                }


                //apprximate contours -> polygon
                //for (auto& contour : contours) {
                //    cv::approxPolyDP(contour, contour,
                //        slider_poly_approx_eps.value /100.0f,
                //        false);
                //}

                //draw to original image
                //cv::polylines(img, contours, false, { 255,0,255 }, 1);

                {
                    std::sort(new_contours.begin(), new_contours.end(),
                        [/*capture*/](Contour const& c1, Contour const& c2) {
                        return c1.size() > c2.size();
                    });
                    //we want to remove all contours with size <100
                    //find the first element with size <100
                    auto first_small_contour_it = std::find_if(new_contours.begin(),
                        new_contours.end(),
                        [&slider_min_contour_size](Contour const& c) {
                        return c.size() < 5;
                    });

                    //remove all small contours
                    int new_size = std::distance(new_contours.begin(), first_small_contour_it);
                    CLAMP_(new_size, 0, new_contours.size());
                    new_contours.resize(new_size);
                }
#endif

                int idx = 0;
                for (auto& contour : new_contours) {
                    cv::polylines(img, { contour }, false, colors[idx], 1);
                    for (auto& p : contour) {
                        cv::circle(img, p, 2, colors[idx], -1);
                    }
                    idx++;
                }

            }
            _show(gimg);
            _show(cimg);
        }


        _show(img);
        cv::waitKey(1);
    }

    std::cout << "Hello World!\n";
}
