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
void calcOpticalFlowLK(Mat & prevGray, Mat & gray, vector<Point2f> & pointsCurrent, vector<Point2f> & pointsNext, Size & winSize);
double diff(Point2f & point, Mat & cur,bool x)
{
    //std :: cout << point.x  << " points      " << point.y << std :: endl;
    return ((double)((double)cur.at<uchar>(point.y + (int)!x ,point.x + (int)x) - (double)cur.at<uchar>(point.y - (int)!x,point.x - (int)x)))/2;
}
double diffTime(Point2f & point, Mat & cur, Mat & prev)
{
    return ((double)cur.at<uchar>(point.y,point.x) - (double)prev.at<uchar>(point.y,point.x))/2;
}
void opticalFlowOpenCV(VideoCapture & cap)
{
    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS,20,0.001);
    Size subPixWinSize(10,10);
    Size winSize(10,10 );
    namedWindow("OpticalFlow");
    //namedWindow("OpticalFlow1");
    Mat gray,prevGray,image,frame;
    vector<Point2f> points[2];
    vector<Point2f> pointsTmp;
    vector<Point2f> myOwnVector;

    while(1)
    {
        cap >> frame;
        if(frame.empty())
            break;
        image = frame.clone();
        cvtColor(image,gray,COLOR_RGB2GRAY);
        if(points[0].size() < 150)
        {
            goodFeaturesToTrack(gray,pointsTmp,100,0.01,10,Mat(),3,3,0,0.4);
            //cornerSubPix(gray,pointsTmp,subPixWinSize,Size(-1,-1),termcrit);
            for(int i = 0; i < 100  ; i++)
                points[0].push_back(pointsTmp[i]);

        }
        //for(int i = 0; i < pointsTmp.size(); i++)
        //    std :: cout << points[0] << " " ;
        std :: cout << std :: endl;
        vector<uchar> status;
        vector<float> err;
        if(prevGray.empty())
            prevGray = gray.clone();
        myOwnVector = points[0];
        for(int i = 0; i < points[0].size(); i++)
        {
                points[0][i].x += (int)(points[0][i].x < 40)*(40) - (int)(points[0][i].x > gray.cols - 40)*(40);
                points[0][i].y += (int)(points[0][i].y < 40)*(40) - (int)(points[0][i].y > gray.rows - 40)*(40);
        }
         calcOpticalFlowLK(prevGray,gray,points[0],myOwnVector,winSize);
        calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,1, termcrit, 10, 0.001);
        //std :: cout << prevGray.size() << " " <<  gray.size() << " " << points[0].size() << " " << myOwnVector.size() << std :: endl;
        calcOpticalFlowLK(prevGray,gray,points[0],myOwnVector,winSize);
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
            if(norm(myOwnVector[i]) < 1e-2)
                circle(image,points[0][i],2,Scalar(100,100,100));
            else
            {
               //double tmp = myOwnVector[i].x;
               //myOwnVector[i].x = -myOwnVector[i].y;
               //myOwnVector[i].y = tmp;
               line(image,points[0][i],points[0][i] - 20*myOwnVector[i],Scalar(200,100,180),2);
            }
        }
        points[1].resize(k);
        //std :: cout << points[1].size() << " points size" << std :: endl;
        imshow("OpticalFlow",image);
        waitKey(10);
        std :: swap(points[1],points[0]);
        cv :: swap(prevGray,gray);
    }
}
void printMatrix(const Mat & matrix)
{
    for(int i = 0; i < matrix.rows; i++)
    {
        for(int j = 0; j < matrix.cols; j++)
            std :: cout << (double)matrix.at<double>(i,j) << " " ;
        std :: cout << std :: endl;
    }
    std :: cout << std :: endl;
}
void calcOpticalFlowLK(Mat & prevGray, Mat & gray, vector<Point2f> & pointsCurrent, vector<Point2f> & pointsNext, Size & winSize)
{
    for(int i = 0; i < pointsCurrent.size(); i++)
    {
        Point2f current = pointsCurrent[i];
        Mat A(winSize.area(),2,CV_64F);
        Mat b(winSize.area(),1,CV_64F);
        Mat tmp;
        Mat B;
        //
        //
        //
        //std :: cout << pointsCurrent[i] << " current points " << std :: endl;
        for(int i1 = 0; i1 < winSize.height; i1++)
        {
            for(int j = 0; j < winSize.width; j++)
            {
                Point2f tpq;
                tpq.y = current.y + i1 - winSize.height/2;
                tpq.x = current.x + j - winSize.width/2;
                //std :: cout << A.at<double>(i1*winSize.width + j,0) << " " << std :: endl;
                //std :: cout << tpq << " points 111" << std :: endl;
                A.at<double>(i1*winSize.width + j,1) = diff(tpq,gray,true);
                A.at<double>(i1*winSize.width + j,0) = diff(tpq,gray,false);
                b.at<double>(i1*winSize.width + j,0) = diffTime(tpq,gray,prevGray);
            }
        }
        //std :: cout << "Hellow I am here" << std :: endl;
        //std :: cout << prevGray.size() << " " << gray.size() << " " << pointsCurrent.size() << " " << pointsNext.size() << std :: endl;
        //printMatrix(A);
        transpose(A,B);
        //printMatrix(B);
        //std :: cout << B.cols << " cols rows " << B.rows << std :: endl;
        //std :: cout << A.cols << " cols rows " << A.rows << std :: endl;
        tmp = B*A;
        B = tmp.clone();
        //printMatrix(B);
        //std :: cout << "I came here " << std :: endl;
        if(fabs(determinant(B)) > eps )
        {
            //std :: cout << " and here " << std :: endl;
            transpose(A,tmp);
            A = tmp.clone();
            //std :: cout << " i came and here" << std :: endl;
            //printMatrix(B);
              tmp = B.inv();
            //std :: cout << "inverse matrix" << std :: endl;
            B = tmp.clone();
            tmp = B*A;
            B = tmp.clone();
            tmp = B*b;
            //printMatrix(tmp);
            //printMatrix(tmp);
            Point2f pt(tmp.at<double>(0,1),tmp.at<double>(0,0));
            pt.x /= sqrt(pow(pt.x,2) + pow(pt.y,2));
            pt.y /= sqrt(pow(pt.x,2) + pow(pt.y,2));
            pointsNext[i] = pt;
        }
        else
        {
            //std :: cout << " and here 1 "  << std :: endl;
            pointsNext[i] = pointsCurrent[i];
        }
    }
}
void opticalFlowMyOwn(VideoCapture cap)
{
    Mat frame;
    Mat grayCurrent,grayPrev;
    vector<Point2f> pointsTmp;
    vector<Point2f> pointsCurrent,pointsNext;
    TermCriteria termcrit(TermCriteria::COUNT |  TermCriteria::EPS,20,0.001);
    Size subPixWinSize(10,10);
    Size winSize(30,30);
    //Size winSize(10,10);
    bool first = true;
    namedWindow("MLC");
    while(1)
    {
        cap >> frame;
        if(frame.empty())
        {
            std :: cout << " end of file" << std :: endl;
            return ;
        }
        cvtColor(frame,grayCurrent,CV_RGB2GRAY);
        if(grayPrev.empty())
            grayPrev = grayCurrent.clone();
        if(first)
        {
            for(int i = 1; i < 10; i++)
            {
                for(int j = 1; j < 10; j++)
                {
                    Point2f tmp;
                    tmp.x = j*(grayCurrent.cols/10);
                    tmp.y = i*(grayCurrent.rows/10);
                    pointsCurrent.push_back(tmp);
                }
            }
            pointsNext = pointsCurrent;
            first = false;
        }
        //pointsTmp = pointsCurrent;
        //pointsCurrent.clear();
        goodFeaturesToTrack(grayCurrent,pointsTmp,100,0.01,10,Mat(),3,3,0,0.4);
        cornerSubPix(grayCurrent,pointsTmp,subPixWinSize,Size(-1,-1),termcrit);
        for(int i = 0; i < pointsTmp.size(); i++)
        {
                pointsTmp[i].x += (int)(pointsTmp[i].x < 40)*(40) - (int)(pointsTmp[i].x > grayCurrent.cols - 40)*(40);
                pointsTmp[i].y += (int)(pointsTmp[i].y < 40)*(40) - (int)(pointsTmp[i].y > grayCurrent.rows - 40)*(40);
        }
        pointsNext = pointsTmp;
        calcOpticalFlowLK(grayPrev,grayCurrent,pointsTmp,pointsNext,winSize);
        //std :: cout << pointsTmp.size() << std :: endl;
        for(int i = 0; i < pointsTmp.size(); i++)
        {
            if(norm(pointsNext[i]) < 1e-2)
                circle(frame,pointsTmp[i],2,Scalar(100,100,100));
            else
                line(frame,pointsTmp[i],pointsTmp[i] + 10*pointsNext[i],Scalar(200,100,180),2);
       }
        imshow("MLC",frame);
        waitKey(10);
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
    //opticalFlowMyOwn(cap);
    return 0;
}
