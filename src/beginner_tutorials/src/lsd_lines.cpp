//#include <iostream>
//#include <string>
//#include <sstream>
//#include "opencv2/core/core.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//
//#include "lsd_opencv.hpp"
//
//using namespace std;
//using namespace cv;
//
//
//int main(int argc, char** argv)
//{
///*    if (argc != 2)
//    {
//        std::cout << "lsd_lines [input image]" << std::endl;
//        return false;
//    }
//
//    std::string in = argv[1];*/
//	int index=30;
//
//	while(1){
//
//		Mat x2=cv::imread(("F:\\华为\\image\\I ("+num2str(index)+").png").c_str());
//		Rect roi(0, 359, 640, 120);
//		Mat roiImage=x2(roi);
//		imshow("测试图像roi",roiImage);
//		waitKey(30);
//
//		Mat roiImageH;
//		cvtColor(roiImage,roiImageH,CV_RGB2HSV);
//		vector<Mat> splited;
//		split(roiImageH,splited);
//
//		Mat image=splited[2];
//		// Create and LSD detector with std refinement.
//		Ptr<LineSegmentDetector> lsd_std = createLineSegmentDetectorPtr(LSD_REFINE_STD);
//		double start = double(getTickCount());
//		vector<Vec4i> lines_std;
//		lsd_std->detect(image, lines_std);
//		double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
//		std::cout << "OpenCV STD (blue) - " << duration_ms << " ms." << std::endl;
//
//		// Create an LSD detector with no refinement applied.
//		Ptr<LineSegmentDetector> lsd_none = createLineSegmentDetectorPtr(LSD_REFINE_NONE);
//		start = double(getTickCount());
//		vector<Vec4i> lines_none;
//		lsd_none->detect(image, lines_none);
//		duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
//		std::cout << "OpenCV NONE (red)- " << duration_ms << " ms." << std::endl;
//		std::cout << "Overlapping pixels are shown in purple." << std::endl;
//
//		/*Mat difference = Mat::zeros(image.size(), CV_8UC1);
//		lsd_none->compareSegments(image.size(), lines_std, lines_none, difference);
//		imshow("Line difference", difference);*/
//
//		Mat drawnLines(roiImage);
//		lsd_std->drawSegments(drawnLines, lines_std);
//		imshow("Standard refinement", drawnLines);
//		waitKey(30);
//
//		Mat drawnLines2(roiImage);
//		lsd_none->drawSegments(drawnLines2, lines_none);
//		imshow("NONE Standard refinement", drawnLines2);
//		waitKey(30);
//
//		imwrite(("F:\\华为\\image\\resultLSD\\"+num2str(index)+".jpg").c_str(),drawnLines);
//		index+=2;
//	}
//    return 0;
//}
