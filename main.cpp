#include <iostream>
#include <cmath>
#include <ctype.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

double eps = 1e-10;
using namespace cv;
using namespace std;
const double PI = M_PI;
const int MAX_COUNT = 500;
double diff(Point2f & point, Mat & cur,bool x)
{
    return (cur.at<double>(point.y + (int)!x ,point.x + (int)x) - cur.at<double>(point.y - (int)!x,point.x - (int)x))/2;
}
double diffTime(Point2f & point, Mat & cur, Mat & prev)
{
    return (cur.at<double>(point.y,point.x) - prev.at<double>(point.y,point.x))/2;
}
void opticalFlowOpenCV(VideoCapture & cap)
{
    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS,20,0.001);
    Size subPixWinSize(10,10);
    Size winSize(40,40);
    namedWindow("OpticalFlow");
    Mat gray,prevGray,image,frame;
    vector<Point2f> points[2];
    vector<Point2f> pointsTmp;

    while(1)
    {
        cap >> frame;
        if(frame.empty())
            break;
        image = frame.clone();
        cvtColor(image,gray,COLOR_RGB2GRAY);
        if(points[0].size() < 150)
        {
            goodFeaturesToTrack(gray,pointsTmp,150,0.01,10,Mat(),3,3,0,0.4);
            cornerSubPix(gray,pointsTmp,subPixWinSize,Size(-1,-1),termcrit);
            for(int i = 0; i < 150; i++)
                points[0].push_back(pointsTmp[i]);

        }
        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            prevGray = gray.clone();
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,1, termcrit, 10, 0.001);
        int k = 0;
        for(int i = k = 0; i < points[1].size(); i++)
        {
            if(!status[i])
                continue;
            points[1][k++] = points[1][i];
            if(norm((points[1][i] - points[0][i])) < 0.1)
                circle(image,points[1][i],2,Scalar(10,255,10),2);
            else
            {
                Point2f v = (points[1][i] - points[0][i])/norm((points[1][i] - points[0][i]));
                Point2f begin = points[1][i] + 15*v;
                line(image,points[1][i],begin,Scalar(30,255,100),1.5);
                line(image,begin,Point2f(begin.x -2*( v.x*cos(PI/4) + v.y*sin(PI/4)),begin.y -2*( -v.x*sin(PI/4) + v.y*cos(PI/4))),Scalar(100,255,100),1.5);
                line(image,begin,Point2f(begin.x -2*( v.x*cos(PI/4) - v.y*sin(PI/4)),begin.y -2*( v.x*sin(PI/4) + v.y*cos(PI/4))),Scalar(100,255,100),1.5);
             }
        }
        points[1].resize(k);
        std :: cout << points[1].size() << " points size" << std :: endl;
        imshow("OpticalFlow",image);
        waitKey(10);
        std :: swap(points[1],points[0]);
        cv :: swap(prevGray,gray);
    }
}
void calcOpticalFlowLK(Mat & prevGray, Mat & gray, vector<Point2f> & pointsCurrent, vector<Point2f> & pointsNext, Size & winSize)
{
    pointsNext.clear();
    for(vector<Point2f> :: iterator i = pointsCurrent.begin(); i < pointsCurrent.end(); i++)
    {
        Point2f current = *i;
        Mat A(winSize.area(),2,CV_64F);
        Mat b(winSize.area(),1,CV_64F);
        Mat tmp;
        Mat B;
        for(int i1 = 0; i1 < winSize.height; i1++)
        {
            for(int  j = 0; j < winSize.width; j++)
            {
                A.at<double>(i1*j,0) = diff(current,gray,true);
                A.at<double>(i1*j,1) = diff(current,gray,false);
                b.at<double>(i1*j,0) = diffTime(current,gray,prevGray);
            }
        }
        transpose(A,B);
        multiply(B,A,tmp);
        B = tmp.clone();
        if(fabs(determinant(B)) > eps)
        {
            tmp = B.inv();
            B = tmp.clone();
            multiply(B,A,tmp);
            B = tmp.clone();
            multiply(B,b,tmp);
            B = tmp.clone();
            Point2f pt(B.at<double>(1,0),B.at<double>(0,0));
            pointsNext.push_back(*i + pt);
        }
        else
        {
            pointsNext.push_back(*i);
        }
    }
}
void opticalFlowMyOwn(VideoCapture cap)
{
    Mat frame;
    Mat framePrev;
    vector<Point2f> pointsCurrent,pointsPrev;

    while(1)
    {

    }
}
int main(int argc, char ** argv)
{
    VideoCapture cap;
    if(argc > 2 )
    {
        std :: cout << "Unknown input " << std :: endl;
        return 0;
    }
    else if(argc == 2)
    {
        cap.open(argv[1]);
    }
    else
    {
        cap.open(0);
    }
    if(!cap.isOpened())
    {
        std :: cout << "Cannot open video flow" << std :: endl;
        return 0;
    }
    opticalFlowOpenCV(cap);
    return 0;
}
