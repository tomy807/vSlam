#include <stdio.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>

using namespace std;
using namespace cv::dnn;

struct detectionResult
{
	cv::Rect plateRect;
	double confidence;
	int type;
};

void NMS(std::vector<detectionResult>& vResultRect);

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}



int main() {

	vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
	const float confidenceThreshold = 0.24f;
	Net m_net;
	std::string yolo_cfg = "/home/tomy807/vslam/config/yolo4.cfg";
	std::string yolo_weights = "/home/tomy807/vslam/config/yolov4.weights";
	m_net = readNetFromDarknet(yolo_cfg, yolo_weights);
	m_net.setPreferableBackend(DNN_BACKEND_OPENCV);
	m_net.setPreferableTarget(DNN_TARGET_CPU);
    
	LoadImages("/home/tomy807/dataset/sequences/00", vstrImageFilenames, vTimestamps);

	std::ifstream fin ("/home/tomy807/vslam/config/coco.names" );
    if ( !fin )
    {
        std::cout<<"please generate the coco file called coco.names!"<<std::endl;
    }

    std::vector<std::string> names;
    while ( !fin.eof() )
    {
        std::string name;
        fin>>name;
        names.push_back ( name );
        if ( fin.good() == false )
            break;
    }
	cv::namedWindow( "Trajectory", cv::WINDOW_AUTOSIZE );

	for(int i=0;i<vstrImageFilenames.size();i++){

		cv::Mat img = cv::imread(vstrImageFilenames[i]);
		

		cv::Mat inputBlob = blobFromImage(img, 1 / 255.F, cv::Size(320, 320), cv::Scalar(), true, false); //Convert Mat to batch of images

		m_net.setInput(inputBlob);


		cv::Mat detectionMat = m_net.forward();
		std::vector<cv::String> layer_names=m_net.getLayerNames();

		std::vector<detectionResult> vResultRect;

		

		for (int i = 0; i < detectionMat.rows; i++)
		{
			const int probability_index = 5;
			const int probability_size = detectionMat.cols - probability_index;
			float* prob_array_ptr = &detectionMat.at<float>(i, probability_index);
			size_t objectClass = std::max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
			float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
			if (confidence > confidenceThreshold)
			{
				float x_center = detectionMat.at<float>(i, 0) * (float)img.cols;
				float y_center = detectionMat.at<float>(i, 1) * (float)img.rows;
				float width = detectionMat.at<float>(i, 2) * (float)img.cols;
				float height = detectionMat.at<float>(i, 3) * (float)img.rows;
				cv::Point2i p1(round(x_center - width / 2.f), round(y_center - height / 2.f));
				cv::Point2i p2(round(x_center + width / 2.f), round(y_center + height / 2.f));
				cv::Rect2i object(p1, p2);
				detectionResult tmp;
				tmp.plateRect = object;
				tmp.confidence = confidence;
				tmp.type = objectClass;
				vResultRect.push_back(tmp);
			}
		}


		if(vResultRect.size()==0){
			cv::imshow("Trajectory", img);
			cv::waitKey(1);
			continue;
		} 

		NMS(vResultRect);

		

		for (int i = 0; i < vResultRect.size(); i++)
		{
			cv::Rect tmp=vResultRect[i].plateRect & cv::Rect(0,0,img.size[1],img.size[0]);
			cv::rectangle(img, tmp, cv::Scalar(0, 0, 255), 2);
			int label=vResultRect[i].type;
			cv::putText(img,names[label],cv::Point(tmp.x,tmp.y),cv::FONT_HERSHEY_DUPLEX,2,cv::Scalar(0,0,255),2);
			printf("index: %d, confidence: %g\n", vResultRect[i].type, vResultRect[i].confidence);
		}
		cv::imshow("Trajectory", img);
		cv::waitKey(1);

	}

	return 0;
}

void NMS(std::vector<detectionResult>& vResultRect)
{
	
	for (int i = 0; i < vResultRect.size() - 1; i++)
	{
		for (int j = i + 1; j < vResultRect.size(); j++)
		{
			double IOURate = (double)(vResultRect[i].plateRect & vResultRect[j].plateRect).area() / (vResultRect[i].plateRect | vResultRect[j].plateRect).area();
			if (IOURate >= 0.5)
			{
				if (vResultRect[i].confidence > vResultRect[j].confidence) {
					vResultRect.erase(vResultRect.begin() + j);
					j--;
				}
				else {
					vResultRect.erase(vResultRect.begin() + i);
					i--;
					break;
				}
			}
		}
	}
}