#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <windows.h>
#include <iostream>

using namespace cv;
using namespace std;

void display(Mat& im, Mat& bbox) {
    int n = bbox.rows;
    for (int i = 0; i < n; i++)
    {
        line(im, Point2i(bbox.at<float>(i, 0), bbox.at<float>(i, 1)), Point2i(bbox.at<float>((i + 1) % n, 0), bbox.at<float>((i + 1) % n, 1)), Scalar(255, 0, 0), 3);
    }
    imshow("Result", im);
}


int main(void)
{
    while (1) {


        // Get the image // 

        cv::VideoCapture camera(0);

        if (!camera.isOpened()) {
            std::cerr << "ERROR: Could not open camera" << std::endl;
            return 1;
        }

        // this will contain the image from the webcam
        cv::Mat frame;

        // capture the next frame from the webcam
        camera >> frame;

        // save frame to input image
        Mat inputImage = frame;

        QRCodeDetector qrDecoder = QRCodeDetector::QRCodeDetector();

        Mat bbox, rectifiedImage;

        std::string data = qrDecoder.detectAndDecode(inputImage, bbox, rectifiedImage);
        if (data.length() > 0)
        {
            cout << "Decoded Data : " << data << endl;

            display(inputImage, bbox);
            rectifiedImage.convertTo(rectifiedImage, CV_8UC3);
            imshow("Rectified QRCode", rectifiedImage);

            
        }
        else {
            display(inputImage, bbox);
            cout << "QR Code not detected" << endl;
           
        }

        waitKey(0);
    }

        
}






