#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
 
int main(int argc, char* argv[])
{


    int mapping[5] = {0, 1, 2, 3, 4};
    std::map<int, std::map<int, Point2f> > pointsdat;

    std::ifstream infile(argv[1]);
    std::string line;
    int i = 1;
    Point2f tmpPoint;
    while (std::getline(infile, line))
    {
        if ((i % 3) == 0) {
            i++;
            continue;
        }

        std::vector<string> tokens;
        boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);
        for (size_t k = 0; k < tokens.size() - 1; k++) {

            double value = -1;
            if (tokens[k].compare("nan") != 0) {
                value = boost::lexical_cast<double>(tokens[k]);
            }

            if ((i % 3) == 1) {
                pointsdat[k][i / 3] = Point2f(value, 0);
            }
            else {
                pointsdat[k][i / 3].y = value;
            }
        }

        i++;
    }

    ofstream outputfile;
    outputfile.open("markerssvoboda.xml");

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
                if (pt.x > 0) {
                    outputfile << "<location camera_id=\"" << mapping[j] << "\" frame=\"0\" pinned=\"true\" x=\"" << pt.x << "\" y=\"" << pt.y << "\"/>" << endl;
                }
            }

            outputfile << "</marker>";
        }

        outputfile << "</markers>\n \
      </chunk>\n \
    </document>\n";
}
