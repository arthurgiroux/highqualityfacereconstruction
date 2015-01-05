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

    std::map<int, std::vector<Point2f> > pointsdat;
    std::map<int, std::vector<string> > idmat;


    for (int cam = 0; cam < argc - 1; cam++) {
        const string inputSettingsFile = argc > cam ? argv[cam+1] : "default.xml";
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


        board_Size = board_Width * board_Height;
        fs.release();

        for (int i = 0; i < 1; i++) {
            for (int j = 0; j < image_points.cols; j++) {
                Point2f pt = image_points.at<Point2f>(i, j);
                pointsdat[j].push_back(pt);
            }
        }
    }

    ofstream outputfile;
    outputfile.open("markers.xml");

    outputfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n \
    <document version=\"0.9.1\">\n \
      <chunk>\n \
        <cameras>\n \
          <camera id=\"0\" label=\"0038__Canon 7D (1180706201).jpg\">\n \
            <resolution width=\"5184\" height=\"3456\"/>\n \
          </camera>\n \
          <camera id=\"1\" label=\"0038__Canon EOS 1100D (123062013458).jpg\">\n \
            <resolution width=\"4272\" height=\"2848\"/>\n \
          </camera>\n \
          <camera id=\"2\" label=\"0038__Canon EOS 50D (-2034246175).jpg\">\n \
            <resolution width=\"4752\" height=\"3168\"/>\n \
          </camera>\n \
        </cameras>\n \
        <markers>\n";

        for (int i = 0; i < pointsdat.size(); i++) {
            outputfile << "<marker id=\"" << i << "\" label=\"point " << (i + 1) << "\">";
            for (int j = 0; j < pointsdat[i].size(); j++) {
                Point2f pt = pointsdat[i][j];
                outputfile << "<location camera_id=\"" << j << "\" frame=\"0\" pinned=\"true\" x=\"" << pt.x << "\" y=\"" << pt.y << "\"/>";
            }

            outputfile << "</marker>";
        }

        outputfile << "</markers>\n \
      </chunk>\n \
    </document>\n";
}
