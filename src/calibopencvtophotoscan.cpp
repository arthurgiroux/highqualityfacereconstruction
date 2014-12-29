#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


string printMatrix(Mat mat) {
   std::ostringstream o;

    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            o << mat.at<float>(i, j);
        }
    }
    return o.str();
}


int main(int argc, char* argv[])
{
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }

    int nrOfFrames;
    Mat cameraMatrix;
    Mat extrinsics;
    fs["nrOfFrames"] >> nrOfFrames; 
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Extrinsic_Parameters"] >> extrinsics;
    fs.release();

    cout << cameraMatrix << endl << endl << endl;

    for (int i  = 0; i < nrOfFrames; ++i) {
        Mat rrod = extrinsics(Range(i, i+1), Range(0,3));
        Mat t = extrinsics(Range(i, i+1), Range(3,6));

        t = t.t();
        Mat r;
        Rodrigues(rrod, r);

        Mat C = -r.inv()*t;
        Mat P;
        Mat homo = (Mat_<double>(1, 4) << 0, 0, 0, 1);
        hconcat(r.t(), C, P);
        Mat Phomo;
        vconcat(P, homo, Phomo);

        cout << "<transform> " << printMatrix(Phomo) << "</transform>" << endl;

    }
}
