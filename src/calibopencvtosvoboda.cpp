#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

std::string stringify(float x)
 {
   std::ostringstream o;
   if (!(o << x))
     return "";
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
    int board_Width;
    int board_Height;
    int board_Size;
    Mat image_points;
    Mat extrinsics;
    fs["nrOfFrames"] >> nrOfFrames;
    fs["board_Width"] >> board_Width;
    fs["board_Height"] >> board_Height; 
    fs["Image_points"] >> image_points;


    nrOfFrames = 4;

    board_Size = board_Width * board_Height;
    fs.release();

    std::map<int, std::vector<string> > pointsdat;
    std::map<int, std::vector<string> > idmat;

    for (int i = 0; i < nrOfFrames; i++) {
        for (int j = 0; j < image_points.cols; j++) {
            Point2f pt = image_points.at<Point2f>(i, j);
            pointsdat[i*3].push_back(stringify(pt.x));
            pointsdat[i*3 + 1].push_back(stringify(pt.y));
            pointsdat[i*3 + 2].push_back("1");
        }
    }

    for (int i = 0; i < nrOfFrames; i++) {
        for (int j = 0; j < board_Size; j++) {
            idmat[i].push_back("1");
        }
    }

    std::ofstream file;
    file.open("Res.dat");

    for (int i = 0; i < nrOfFrames; i++) {
        file << "5472 3648" << std::endl;
    }

    file.close();

    file.open("points.dat");

    for (std::map<int, std::vector<string> >::iterator i = pointsdat.begin(); i != pointsdat.end(); i++) {
        for (std::vector<string>::iterator j = i->second.begin(); j != i->second.end(); j++) {
            file << j->c_str() << "\t";
        }

        file << std::endl;
    }
    file.close();

    file.open("IdMat.dat");

    for (std::map<int, std::vector<string> >::iterator i = idmat.begin(); i != idmat.end(); i++) {
        for (std::vector<string>::iterator j = i->second.begin(); j != i->second.end(); j++) {
            file << j->c_str() << "\t";
        }

        file << std::endl;
    }
    file.close();
}
