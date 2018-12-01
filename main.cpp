#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "utils.hpp"
using namespace std;
using namespace cv;
/**
 * author: hyx
 * date: 2018/8/24
 * a visual odometry using LK
 * input: a squence of images, calibrations of camera
 * output: rotation matrix R and translation vector t of every image.
 */
int main(int argc, char **argv) {
	//read intrinsic parameters of the camera
	string strSettingPath="config.yaml";	
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingPath << endl;
       exit(-1);
    }
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
	
	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
	
	/*cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }*/
    int MIN_NUM=fSettings["MIN_NUM"];		
	int frame_num=fSettings["frame_num"];
	//
	//vector<string> vstrImageFilenames;
	//LoadImages(datasetPath,vstrImageFilenames);
	//compute each frame's scale
	vector<double> scale;
	getAbusoluteScale(scale,"05.txt");
	
	//the file to save trajectory
	ofstream out;
	out.open("Trajectory.txt");
    //read first frame
	char imageFile[1000];
	cv::Mat ref,curr;	
	Mat R_f = Mat::eye(3,3,CV_64F);
	Mat t_f=Mat::zeros(3,1,CV_64F);
	cout<<"R_f="<<endl<<R_f<<endl;
	cout<<"t_f="<<endl<<t_f<<endl;
	sprintf(imageFile, "image_0/%06d.png", 0);
	ref=imread(imageFile);
	//imshow("image",ref);	
	//convert to gray image,undistort
	cvtColor(ref,ref,CV_BGR2GRAY);
	//undistort(ref,ref,K,DistCoef);
	//extract features
	vector<Point2f> ref_kp,cur_kp;
	featureDetection(ref,ref_kp);
	out<<0<<"	"<<0<<"	"<<0<<endl;
	//int frame_id=1;
	
	//display
	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;  
	cv::Point textOrg(10, 50);
	namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
    namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	
	for(int frame_id=1;frame_id<frame_num;frame_id++){
		sprintf(imageFile,"image_0/%06d.png", frame_id);
		curr=imread(imageFile);
		cvtColor(curr,curr,CV_BGR2GRAY);
		//undistort(curr,curr,K,DistCoef);
		LKFeatureTracking(ref,curr,ref_kp,cur_kp);
		Mat R,t,E;
		E=findEssentialMat(cur_kp,ref_kp,K,RANSAC);
		recoverPose(E, cur_kp,ref_kp,K,R, t);
		if(scale[frame_id]>0.1){
			t_f=scale[frame_id]*(R_f*t)+t_f;
			R_f=R_f*R;
		}
		out<<t_f.at<double>(0)<<"	"<<t_f.at<double>(1)<<"	"<<t_f.at<double>(2)<<endl;
		
		if(cur_kp.size()<MIN_NUM){
			featureDetection(curr,cur_kp);
		}
		ref=curr;
		ref_kp=cur_kp;
		//frame_id++;
		//display
	    int x = int(t_f.at<double>(0)) + 300;
        int y = int(t_f.at<double>(2))+100;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

        imshow( "Road facing camera", curr );
        imshow( "Trajectory", traj );
        waitKey(1);	      
	}
	out.close();
    
}
