#include <opencv2/opencv.hpp>
#include <iostream>
#include<fstream>
#include <boost/concept_check.hpp>
using namespace std;
using namespace cv;
void featureDetection(Mat& frame,vector<Point2f>& kp){
	vector<KeyPoint> keypoints;
	FAST(frame,keypoints,40);
// 	KeyPoint::convert(keypoints,kp);
}

void LKFeatureTracking(Mat& ref,Mat& curr,vector<Point2f>& ref_kp,vector<Point2f>& cur_kp){
	vector<uchar> status;
	vector<float> err;
	calcOpticalFlowPyrLK(ref,curr,ref_kp,cur_kp,status,err);
	vector<Point2f>::iterator ref_iter=ref_kp.begin();
	vector<Point2f>::iterator cur_iter=cur_kp.begin();
	for(auto state:status){
		if(state==0){
			ref_kp.erase(ref_iter);
			cur_kp.erase(cur_iter);			
		}else{
			ref_iter++;
			cur_iter++;
		}
	}	
}

void getAbusoluteScale(vector<double>& scale,string ground_truth){
	ifstream in;
	in.open(ground_truth);
	scale.push_back(1);
	string line="";
	getline(in,line);
	istringstream item(line);
	double t,x_pre=0,y_pre=0,z_pre=0;
	double x=0,y=0,z=0;
	for(int i=0;i<12;i++){
		if(i==3) 
			item>>x_pre;
		else if(i==7)
			item>>y_pre;
		else if(i==11)
			item>>z_pre;
		else
			item>>t;
	}
	//item>>t>>x_pre>>y_pre>>z_pre;
	while(getline(in,line)&&!line.empty()){
		istringstream nextItem(line);
		//nextItem>>t>>x>>y>>z;
		for(int i=0;i<12;i++){
			if(i==3) 
				nextItem>>x;
			else if(i==7)
				nextItem>>y;
			else if(i==11)
				nextItem>>z;
			else
				nextItem>>t;
		}
		scale.push_back(sqrt((x - x_pre)*(x - x_pre) + (y - y_pre)*(y - y_pre) + (z - z_pre)*(z - z_pre)));
		x_pre=x;
		y_pre=y;
		z_pre=z;
	}
	in.close();
}

/*void LoadImages(const string &strFile, vector<string> &vstrImageFilenames)
{
    ifstream f;
    f.open(strFile.c_str());

    string s0;
	double t;
    string sRGB;
	// skip first three lines
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(getline(f,s0)&&!s0.empty())
    {        
        istringstream in(s0); 		
        in>>t;
		in>>sRGB;
        vstrImageFilenames.push_back(sRGB);        
    }
}*/