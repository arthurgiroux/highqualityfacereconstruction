#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace cv;
using namespace std;

void writeIntrinsic(Mat intrinsic, ofstream &outputfile) {
	outputfile << "<fx>" << intrinsic.at<double>(0, 0) << "</fx>"
	<< "<fy>" << intrinsic.at<double>(1, 1) << "</fy>"
	<< "<cx>" << intrinsic.at<double>(0, 2) << "</cx>"
	<< "<cy>" << intrinsic.at<double>(1, 2) << "</cy>";
}

void writeExtrinsic(Mat extrinsic, ofstream &outputfile) {

	for (int i = 0; i < extrinsic.rows; ++i) {
		for (int j = 0; j < extrinsic.cols; ++j) {
			outputfile << " " << extrinsic.at<double>(i, j);
		}	
	}
}

int main(int argc, char **argv)
{

	if (argc < 3) {
		std::cerr << "not enough argument" << std::endl;
		exit(0);
	}


    std::vector<Mat> projs;

	int nbr_cam = boost::lexical_cast<int>(argv[1]);

    for (int i = 1; i <= nbr_cam; i++) {
    	std::ostringstream filename;
    	filename << argv[2] << "/camera" << i << ".Pmat.cal";
    	ifstream file(filename.str().c_str());
    	if (file.is_open()) {
    		Mat proj(3, 4, CV_64F);
    		string line;
    		for (int j = 0; j < 8; j++) {
    			getline(file, line);
    			if (j >= 5) {
    				std::vector<string> tokens;
					boost::split(tokens, line, boost::is_any_of(" "), boost::token_compress_on);
					tokens.erase(tokens.begin());
					for (size_t k = 0; k < tokens.size(); k++) {
					    proj.at<double>(j - 5, k) = boost::lexical_cast<double>(tokens[k]);
					}
    			}
    		}
    		projs.push_back(proj);
    		std::cout << proj << std::endl;
    	}
    }

    /// Create a window
    viz::Viz3d myWindow("Coordinate Frame");

    /// Add coordinate axes
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());



    std::vector<Mat> intrinsics;
    std::vector<Mat> extrinsics;


    for (int i = 0; i < projs.size(); i++) {
	    Mat K, R;
	    Vec4d t;

        decomposeProjectionMatrix(projs[i], K, R, t);

		Mat transf = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, -1);
		transf.convertTo(transf, CV_64FC1);

		//R =  transf * R;
		//K = K * transf;

		cv::Matx33d RR = R.clone();

        std::cout << K << std::endl;

		intrinsics.push_back(K);

		cv::Vec3d thomo(t[0] / t[3], t[1] / t[3], t[2] / t[3]);

		Vec3d C = -RR.inv()*thomo;
    	Affine3f cam_pose = Affine3f(RR.t(), thomo);

    	Mat tmp, extr;
    	hconcat(RR.t(), thomo, tmp);
    	Mat homo = (Mat_<float>(1, 4) << 0, 0, 0, 1);
    	homo.convertTo(homo, CV_64FC1);
    	vconcat(tmp, homo, extr);
    	extrinsics.push_back(extr);

    	viz::WCameraPosition cpw(0.5); // Coordinate axes
    	viz::WCameraPosition cpw_frustum(Vec2f(0.889484, 0.523599)); // Camera frustum
    	stringstream camwidgetname;
    	camwidgetname << "CPW" << i;
    	myWindow.showWidget(camwidgetname.str().c_str(), viz::WCameraPosition(cpw), Affine3f(cam_pose));
    	stringstream frustrumwidgetname;
    	frustrumwidgetname << "CPW_FRUSTUM" << i;
    	myWindow.showWidget(frustrumwidgetname.str().c_str(), viz::WCameraPosition(cpw_frustum), Affine3f(cam_pose));
    	std::cout << i << std::endl;
    }

    //myWindow.setViewerPose(cam_pose);


 //    std::vector<Point2f> img1;
 //    std::vector<Point2f> img3;
 //    Mat points4D;

 //    img1.push_back(Point2f(3453.28, 2304.57));
 //    img3.push_back(Point2f(2301.31, 2044.03));

 //    img1.push_back(Point2f(3274.36, 2287.78));
 //    img3.push_back(Point2f(2010.2, 2031.13));

 //    img1.push_back(Point2f(2994.22, 2908.16));
 //    img3.push_back(Point2f(1922.4, 2584.57));

 //    img1.push_back(Point2f(3240.06, 2778.9));
 //    img3.push_back(Point2f(2204.8, 2472.51));

 //    triangulatePoints(Proj1, Proj3, img1, img3, points4D);

 //    std::cout << points4D << std::endl;

 //    /// We can get the transformation matrix from camera coordinate system to global using
 //    /// - makeTransformToGlobal. We need the axes of the camera
 //    Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,-1.0f,0.0f), Vec3f(-1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), t);


 //    std::vector<Point3f> cloud;

 //    for (int i = 0; i < points4D.cols; i++) {
 //    	float homoG = points4D.at<float>(3, i);
 //    	cloud.push_back(Point3f(points4D.at<float>(0, i) / homoG, points4D.at<float>(1, i) / homoG, points4D.at<float>(2, i) / homoG));
 //    }

 //    /// Create a cloud widget.
 //    viz::WCloud cloud_widget(cloud, viz::Color::green());

 //    /// Pose of the widget in camera frame
 //    Affine3f cloud_pose = Affine3f().translate(Vec3f(0.0f,0.0f,0.0f));
 //    /// Pose of the widget in global frame
 //    Affine3f cloud_pose_global = transform * cloud_pose;

	// myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);
 //    /// Start event loop.

 //    Mat K, RR, tt, RRR;

 //    decomposeProjectionMatrix(Proj1, K, RR, tt);
 //    std::cout << K << std::endl;
 //    std::cout << RR.t() << std::endl;
 //    std::cout << tt / tt.at<double>(3, 0) << std::endl;

 //    decomposeProjectionMatrix(Proj2, K, RR, tt);
 //    std::cout << K << std::endl;
 //    std::cout << RR.t() << std::endl;
 //    std::cout << tt / tt.at<double>(3, 0) << std::endl;

 //    decomposeProjectionMatrix(Proj3, K, RR, tt);
 //    std::cout << K << std::endl;
 //    std::cout << RR.t() << std::endl;
 //    std::cout << tt / tt.at<double>(3, 0) << std::endl;
 
//     ofstream outputfile;
//     outputfile.open("testphotoscan.xml");

//     outputfile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?> \n\
// <document version=\"0.9.1\">\n \
//   <chunk>\n \
//     <sensors>\n \
//       <sensor id=\"0\" label=\"Canon EOS 7D (39 mm)\" type=\"frame\">\n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <property name=\"pixel_width\" value=\"4.4084972993827173e-03\"/>\n \
//         <property name=\"pixel_height\" value=\"4.4084972993827173e-03\"/>\n \
//         <property name=\"focal_length\" value=\"3.9000000000000000e+01\"/>\n \
//         <property name=\"fixed\" value=\"false\"/>\n \
//         <calibration type=\"frame\" class=\"adjusted\">\n \
//           <resolution width=\"5184\" height=\"3456\"/>\n";

//     writeIntrinsic(intrinsics.at(1), outputfile);

// //           <k1>2.3606948209791326e-01</k1>
// //           <k2>5.1774695402929121e+00</k2>
// //           <k3>-9.2677410490489663e+01</k3>

//     outputfile << "</calibration> \n \
//       </sensor> \n \
//       <sensor id=\"1\" label=\"Canon EOS 7D (46 mm)\" type=\"frame\"> \n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <property name=\"pixel_width\" value=\"4.4084972993827173e-03\"/> \n \
//         <property name=\"pixel_height\" value=\"4.4084972993827173e-03\"/> \n \
//         <property name=\"focal_length\" value=\"4.6000000000000000e+01\"/> \n \
//         <property name=\"fixed\" value=\"false\"/> \n \
//         <calibration type=\"frame\" class=\"adjusted\"> \n \
//           <resolution width=\"5184\" height=\"3456\"/> \n";
//     	writeIntrinsic(intrinsics.at(0), outputfile);

//         outputfile << "</calibration> \n \
//       </sensor> \n \
//       <sensor id=\"2\" label=\"Canon EOS 7D (38 mm)\" type=\"frame\"> \n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <property name=\"pixel_width\" value=\"4.4084972993827173e-03\"/> \n \
//         <property name=\"pixel_height\" value=\"4.4084972993827173e-03\"/> \n \
//         <property name=\"focal_length\" value=\"3.8000000000000000e+01\"/> \n \
//         <property name=\"fixed\" value=\"false\"/> \n \
//         <calibration type=\"frame\" class=\"adjusted\"> \n \
//           <resolution width=\"5184\" height=\"3456\"/> \n";

//           writeIntrinsic(intrinsics.at(3), outputfile);

//          outputfile << "</calibration> \n \
//       </sensor> \n \
//       <sensor id=\"3\" label=\"Canon EOS 50D (43 mm)\" type=\"frame\"> \n \
//         <resolution width=\"4752\" height=\"3168\"/> \n \
//         <property name=\"pixel_width\" value=\"4.7825441919192024e-03\"/> \n \
//         <property name=\"pixel_height\" value=\"4.7825441919192024e-03\"/> \n \
//         <property name=\"focal_length\" value=\"4.3000000000000000e+01\"/> \n \
//         <property name=\"fixed\" value=\"false\"/> \n \
//         <calibration type=\"frame\" class=\"adjusted\"> \n \
//           <resolution width=\"4752\" height=\"3168\"/> \n";
//           writeIntrinsic(intrinsics.at(4), outputfile);

//           outputfile << "</calibration> \n \
//       </sensor> \n \
//       <sensor id=\"4\" label=\"Canon EOS 1100D (44 mm)\" type=\"frame\"> \n \
//         <resolution width=\"4272\" height=\"2848\"/> \n \
//         <property name=\"pixel_width\" value=\"5.3436914794007424e-03\"/> \n \
//         <property name=\"pixel_height\" value=\"5.3436914794007424e-03\"/> \n \
//         <property name=\"focal_length\" value=\"4.4000000000000000e+01\"/> \n \
//         <property name=\"fixed\" value=\"false\"/> \n \
//         <calibration type=\"frame\" class=\"adjusted\"> \n \
//           <resolution width=\"4272\" height=\"2848\"/> \n";
//           writeIntrinsic(intrinsics.at(2), outputfile);

//           outputfile << "</calibration> \n \
//       </sensor> \n \
//     </sensors> \n \
//     <cameras> \n \
//       <camera id=\"0\" label=\"0001__Canon EOS 1100D (123062013458).JPG\" sensor_id=\"4\" enabled=\"true\"> \n \
//         <resolution width=\"4272\" height=\"2848\"/> \n \
//         <transform>";
//         writeExtrinsic(extrinsics.at(2), outputfile);

//         outputfile << "</transform> \n \
//       </camera> \n \
//       <camera id=\"1\" label=\"0001__Canon EOS 50D (-2034246175).JPG\" sensor_id=\"3\" enabled=\"true\"> \n \
//         <resolution width=\"4752\" height=\"3168\"/> \n \
//         <transform>";
//         writeExtrinsic(extrinsics.at(4), outputfile);
//         outputfile << "</transform> \n \
//       </camera> \n \
//       <camera id=\"2\" label=\"0001__Canon EOS 7D (1180706201).JPG\" sensor_id=\"1\" enabled=\"true\"> \n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <transform>";
//         writeExtrinsic(extrinsics.at(0), outputfile);
//         outputfile << "</transform> \n \
//       </camera> \n \
//       <camera id=\"3\" label=\"0001__Canon EOS 7D (1280816619).JPG\" sensor_id=\"2\" enabled=\"true\"> \n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <transform>";
//         writeExtrinsic(extrinsics.at(3), outputfile);
//         outputfile << "</transform> \n \
//       </camera> \n \
//       <camera id=\"4\" label=\"0001__Canon EOS 7D (530501262).JPG\" sensor_id=\"0\" enabled=\"true\"> \n \
//         <resolution width=\"5184\" height=\"3456\"/> \n \
//         <transform>";
//         writeExtrinsic(extrinsics.at(1), outputfile);
//         outputfile << "</transform> \n \
//       </camera> \n \
//     </cameras> \n \
//   </chunk> \n \
// </document> \n";

//       outputfile.close();


    ofstream outputfile;
    outputfile.open("testphotoopencv.xml");
    for (int i = 0; i < nbr_cam; i++) {
        writeIntrinsic(intrinsics.at(i), outputfile);
        writeExtrinsic(extrinsics.at(i), outputfile);
    }

    outputfile.close();
     myWindow.spin();

    return 0;
}