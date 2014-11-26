#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry> 


using namespace cv;



int ransac_id = 1;
int red_threshold = 90;
int red_threshold_up = 50;
int circle_resolution = 1;
int min_radius = 100;
int max_radius = 0;



int MEAN_DEVIATION_DISTANCE_THRESH = 13;
#define MAX_ERROR 50

// structure that represents a fiducial
struct MarkerPair
{
    int first_index; 
    int second_index;
    int first_id; // used for matching on all the cameras
    int second_id; 
    Point2f   first; // coordinate of the center of one of the marker
    Point2f   second;
    float         distance; // distance between the two center of the marker


    // For the euclidian projection
    Point3f   euclid_first;
    Point3f   euclid_second;

    MarkerPair(int first_index_, int second_index_, Point2f first_, Point2f second_, float distance_): first_index(first_index_), second_index(second_index_), first(first_), second(second_), distance(distance_), first_id(0), second_id(0) {}

    bool operator < (const MarkerPair& other) const
    {
        return (distance < other.distance);
    }
};

std::string stringify(float x)
 {
   std::ostringstream o;
   if (!(o << x))
     return "";
   return o.str();
 }

// Compute the distance between two 2D points
double distanceBetweenPoints(Point2f p1, Point2f p2) {
    // sqrt((x1 - x2)^2 + (y1 - y2)^2)
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

// Compute the distance between two 3D points
double distanceBetweenPoints(Eigen::Vector3d p1, Eigen::Vector3d p2) {
    // sqrt((x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2)
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]));
}

std::vector<MarkerPair> good_pairs;
std::vector<Point2f> corners;

Point2f tmpPoint;
bool first = true;

void onMouse(int event, int x, int y, int flag, void* param)
{

    if (event == EVENT_LBUTTONDOWN) {

        double minDist = INFINITY;
        Point2f closestPoint;
        for (int i = 0; i < corners.size(); i++)  {
            double dist = distanceBetweenPoints(corners[i], Point2f(x, y));

            if (dist < minDist) {
                minDist = dist;
                closestPoint = corners[i];
            }
        }

        std::cout << "closestPoint " << closestPoint << std::endl;

        if (first) {
            std::cout << "tmp point" << std::endl;
            tmpPoint = closestPoint;
            first = false;
        }
        else {
            good_pairs.push_back(MarkerPair(0, 0, closestPoint, tmpPoint, distanceBetweenPoints(closestPoint, tmpPoint)));
            std::cout << "adding pair" << closestPoint << ", " << tmpPoint << std::endl;
            first = true;
        }
    }

    else if (event == EVENT_RBUTTONDOWN) {

        double minDist = INFINITY;
        Point2f closestPoint;
        for (int i = 0; i < corners.size(); i++)  {
            double dist = distanceBetweenPoints(corners[i], Point2f(x, y));

            if (dist < minDist) {
                minDist = dist;
                closestPoint = corners[i];
            }
        }

        std::cout << "closestPoint " << closestPoint << std::endl;

        for (int i = 0; i < good_pairs.size(); i++) {
            if (good_pairs[i].first == closestPoint || good_pairs[i].second == closestPoint) {
                good_pairs.erase(good_pairs.begin() + i);
                break;
            }
        }
    }
}

std::vector<MarkerPair> matchPoints(std::vector<Point2f> circles) {

    std::pair<Point2f, Point2f> circle_pair;

    std::vector<MarkerPair> pairs;

    for (size_t i = 0; i < circles.size(); i++) {

        // For all pairs of markers compute the distance between them
        for (size_t j = 0; j < circles.size(); j++) {

            if (j == i) {
                continue;
            }

            double current_dist = distanceBetweenPoints(circles[i], circles[j]);

            pairs.push_back(MarkerPair(i, j, circles[i], circles[j], current_dist));
        }

    }

    // sort by ascending distance
    std::sort(pairs.begin(), pairs.end());

    float mean_distance = 0;
    // Set of markers already taken
    std::set<int> treated;

    // Good pairs
    std::vector<MarkerPair> good_pairs;
    int nbr_good_pairs = 0;



    for (std::vector<MarkerPair>::iterator i = pairs.begin(); i != pairs.end(); i++)  {
        std::cout << i->distance << std::endl;

        // if we didn't take the markers already then we treat it
        if (!treated.count(i->first_index) && !treated.count(i->second_index)) {

            // We only take the pair if it doesn't deviate too much from the
            // mean of the distance between the markers that we already took
            if ((mean_distance == 0 && i->distance >= 20)  || i->distance <= mean_distance * MEAN_DEVIATION_DISTANCE_THRESH * 0.1) {
                nbr_good_pairs++;
                mean_distance = ((mean_distance * (nbr_good_pairs - 1)) + i->distance) / (float) nbr_good_pairs;
                treated.insert(i->first_index);
                treated.insert(i->second_index);
                good_pairs.push_back(*i);
            }
            else {
                std::cout << "too much deviation " << i->distance << " " << mean_distance << " " << mean_distance* MEAN_DEVIATION_DISTANCE_THRESH * 0.1 << std::endl;
            }
        }
    }

    for (std::vector<MarkerPair>::iterator i = good_pairs.begin(); i != good_pairs.end(); i++) {
        std::cout << "good pair " << std::endl;
    }

    return good_pairs;
}


std::vector<MarkerPair> euclidianProject(std::vector<MarkerPair> good_pairs, Point center, float radius) {

    std::cout << " radius " << radius << std::endl;

    for (std::vector<MarkerPair>::iterator i = good_pairs.begin(); i != good_pairs.end(); i++) {
        // We project into the euclidian space centered at the ball
        // for x and y we just destroyWindowproject into the vertical/horizontal axis
        // that goes from the center of the ball to the radius

        // For z we know that x^2 + y^2 + z^2 = r^2 where r^2 is the radius of the ball in pixel
        // So z = sqrt(r^2 - x^2 - y^2)

        float first_x = i->first.x - center.x;
        float first_y = i->first.y - center.y;

        float second_x = i->second.x - center.x;
        float second_y = i->second.y - center.y;
        Point3f first = Point3f(first_x, first_y, -sqrt(abs(radius * radius - first_x * first_x - first_y * first_y)));
        Point3f second = Point3f(second_x, second_y, -sqrt(abs(radius * radius - second_x * second_x - second_y * second_y)));

        //std::cout << first << " " << second << std::endl;
       // std::cout << "plot3([" << first.x << ", " << first.y << ", " << first.z << ", '.')" << std::endl;

        std::cout << "plot3([" << first.x << " , " << second.x << "], [" << first.y << ", " << second.y << "], [" << first.z << ", " << second.z << "])" << std::endl;

        i->euclid_first = first;
        i->euclid_second = second;
    }

    return good_pairs;
}


// RANSAC 

void RANSACCorespondances(std::map<int, std::map<int, Point2f> > &correspondances, int pass, std::vector<MarkerPair> &first_markers_, std::vector<MarkerPair> &second_markers_) {

    // Let's take a point from the first markers

    int max_matches = 0;
    std::map<int, std::map<int, Point2f> > corres;
	
	std::vector<MarkerPair> first_tmp_markers;
	std::vector<MarkerPair> second_tmp_markers;
	
	

    for (std::vector<MarkerPair>::iterator i = first_markers_.begin(); i != first_markers_.end(); i++) {

        // ball center
        Eigen::Vector3d a1(0, 0, 0);
        Eigen::Vector3d a2(i->euclid_first.x, i->euclid_first.y, i->euclid_first.z);
        Eigen::Vector3d a3(i->euclid_second.x, i->euclid_second.y, i->euclid_second.z);



        for (std::vector<MarkerPair>::iterator j = second_markers_.begin(); j != second_markers_.end(); j++) {

            int tmpMatches = 0;

            std::map<int, std::map<int, Point2f> > tmpCorrespondances;

            Eigen::Vector3d b1(0, 0, 0);
            Eigen::Vector3d b2(j->euclid_first.x, j->euclid_first.y, j->euclid_first.z);
            Eigen::Vector3d b3(j->euclid_second.x, j->euclid_second.y, j->euclid_second.z);

            Eigen::Matrix<double, 3, 3> start, end;
            start.col(0) = a1;
            start.col(1) = a2;
            start.col(2) = a3;

            end.col(0) = b1;
            end.col(1) = b2;
            end.col(2) = b3;


            Eigen::Matrix3d transf = Eigen::umeyama(start, end).topLeftCorner(3, 3);


            std::set<MarkerPair> treated;
			
			std::vector<MarkerPair> first_markers(first_markers_);
			std::vector<MarkerPair> second_markers(second_markers_);

            for (std::vector<MarkerPair>::iterator k = first_markers.begin(); k != first_markers.end(); k++) {

                for (std::vector<MarkerPair>::iterator l = second_markers.begin(); l != second_markers.end(); l++) {

                    if (!treated.count(*l) && !treated.count(*k) && (((k != i) && (l != j)) || ((k == i) && (l == j)))) {
                        Eigen::Vector3d c1(k->euclid_first.x, k->euclid_first.y, k->euclid_first.z);
                        Eigen::Vector3d c2(k->euclid_second.x, k->euclid_second.y, k->euclid_second.z);

                        Eigen::Vector3d c3(l->euclid_first.x, l->euclid_first.y, l->euclid_first.z);
                        Eigen::Vector3d c4(l->euclid_second.x, l->euclid_second.y, l->euclid_second.z);

                        Eigen::Vector3d reproj1 = transf * c1;
                        Eigen::Vector3d reproj2 = transf * c2;

                        double dist1 = distanceBetweenPoints(reproj1, c3);
                        double dist2 = distanceBetweenPoints(reproj1, c4);
                        double dist3 = distanceBetweenPoints(reproj2, c3);
                        double dist4 = distanceBetweenPoints(reproj2, c4);

                        
                        double curError = dist1 + dist4;


                        if (dist1 > dist2) {
                            curError = dist2 + dist3;
                        }

                        if (curError < MAX_ERROR) {

                            std::cout << "taking point " << l->first << ", " << l->second << " | " << k->first << ", " << k->second << " error : " << curError << std::endl;

                            tmpMatches++;

                            treated.insert(*l);
                            treated.insert(*k);

                            // If the correspondance was not found yet, we add both points

                            // c1 + c4 & c2 + c3

                            if (dist1 > dist2) {
                                if (k->first_id != 0) {
                                    l->second_id = k->first_id;
                                }
                                else {
                                    k->first_id = ransac_id;
                                    l->second_id = ransac_id;
                                    ransac_id++;
                                    tmpCorrespondances[k->first_id][pass] = k->first;
                                }
                                tmpCorrespondances[k->first_id][pass + 1] = l->second;


                                if (k->second_id != 0) {
                                    l->first_id = k->second_id;
                                }
                                else {
                                    k->second_id = ransac_id;
                                    l->first_id = ransac_id;
                                    ransac_id++;
                                    tmpCorrespondances[k->second_id][pass] = k->second;
                                }
                                tmpCorrespondances[k->second_id][pass + 1] = l->first;

                            }
                            // c1 + c3 & c2 + c4
                            else {

                                if (k->first_id != 0) {
                                    l->first_id = k->first_id;
                                }
                                else {
                                    k->first_id = ransac_id;
                                    l->first_id = ransac_id;
                                    ransac_id++;
                                    tmpCorrespondances[k->first_id][pass] = k->first;
                                }
                                tmpCorrespondances[k->first_id][pass + 1] = l->first;


                                if (k->second_id != 0) {
                                    l->second_id = k->second_id;
                                }
                                else {
                                    k->second_id = ransac_id;
                                    l->second_id = ransac_id;
                                    ransac_id++;
                                    tmpCorrespondances[k->second_id][pass] = k->second;
                                }
                                tmpCorrespondances[k->second_id][pass + 1] = l->second;

                            }
                        }
                        else {
                            //std::cout << "discarding point, too much error " << curError << std::endl;
                        }
                    }
                }

            }

            if (tmpMatches > max_matches) {

				first_tmp_markers = first_markers;
				second_tmp_markers = second_markers;
                corres = tmpCorrespondances;
                max_matches = tmpMatches;
            }
        }

    }
	
	first_markers_ = first_tmp_markers;
	second_markers_ = second_tmp_markers;

    for (std::map<int, std::map<int, Point2f> >::iterator m = corres.begin(); m != corres.end(); ++m) { 

        for (std::map<int, Point2f>::iterator n = m->second.begin(); n != m->second.end(); ++n) { 

            correspondances[m->first][n->first] = n->second;

            std::cout << "corres " << m->first << std::endl;
            std::cout << "camera : " << n->first << "point: " << n->second << std::endl;
        }
    }
        

    std::cout << "found " << correspondances.size() << " correspondances for camera " << pass << " and " << (pass+1) << std::endl;

}



std::vector<MarkerPair> treatImage(char* file) {
    // Load the image
    Mat image;
    image = imread(file, 1);

    if (!image.data)
    {
        printf("No image data \n");
        return std::vector<MarkerPair>();
    }


    //resize(image, image, Size(), 0.3, 0.3);

    // Go into HSV space
    Mat hsv;
    cvtColor(image, hsv, CV_BGR2HSV);

    Mat reddot;
    // select red
    cvNamedWindow("Red dot", CV_WINDOW_NORMAL);

    cvCreateTrackbar("Red threshold", "Red dot", &red_threshold, 255, NULL);
    cvCreateTrackbar("Red threshold up", "Red dot", &red_threshold_up, 255, NULL);

    while (cvWaitKey(50) != ' ') {

        Mat other;
        inRange(hsv, Scalar(0, red_threshold, red_threshold), Scalar(10, 255, 255), reddot);
        inRange(hsv, Scalar(170, red_threshold_up, red_threshold_up), Scalar(180, 255, 255), other);
        reddot = reddot | other;
        Mat copy = reddot.clone();
        imshow("Red dot", copy);
    }

    destroyWindow("Red dot");
    //morphologyEx(reddot, reddot, MORPH_OPEN, Mat());
    

    Mat src_gray;
    cvtColor(image, src_gray, CV_BGR2GRAY);

    corners = std::vector<Point2f>();

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);

    /// Detect edges using canny
    Canny( reddot, canny_output, 100, 100*2, 3 );
    /// Find contours
    findContours( reddot, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
    /// Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++)
     {
        mu[i] = moments(contours[i], false);
    }

    ///  Get the mass centers:
    vector<Point2f> mc(contours.size());
    for (int i = 0; i < contours.size(); i++)
     {
        if (mu[i].m00 != 0) {
            corners.push_back(Point2f(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00));
        }
    }

    /// Draw contours
    for( int i = 0; i< corners.size(); i++)
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       circle(image, corners[i], 3, color, -1, 8, 0 );
       std::cout << corners[i] << std::endl;
     }


    // Let's match the points together
    //good_pairs = std::vector<MarkerPair>();


    cvNamedWindow("good_pairs", CV_WINDOW_NORMAL);
    setMouseCallback("good_pairs", onMouse, 0);

    cvCreateTrackbar("Max deviation", "good_pairs", &MEAN_DEVIATION_DISTANCE_THRESH, 100, NULL);

    int latest_mean_dev = 0;
    good_pairs = std::vector<MarkerPair>();
    while (cvWaitKey(50) != ' ') {
        if (latest_mean_dev != MEAN_DEVIATION_DISTANCE_THRESH) {
            good_pairs = matchPoints(corners);
            latest_mean_dev = MEAN_DEVIATION_DISTANCE_THRESH;
        }

        std::cout << "# pairs " << good_pairs.size() << std::endl;


        Mat copy = image.clone();
        for (size_t idx = 0; idx < good_pairs.size(); idx++) {
            std::cout << " pair : " <<  good_pairs.at(idx).first << ", " << good_pairs.at(idx).second << std::endl;
            line(copy, good_pairs.at(idx).first, good_pairs.at(idx).second, 255);
        }

        imshow("good_pairs", copy);
    }

    destroyWindow("good_pairs");

    // Setting up the euclidian coordinate system with the origin at the center of the ball


    // finding the center of the ball
    vector<Vec3f> circles_ball;

    Vec3f ballcenter;


    cvNamedWindow("centers", CV_WINDOW_NORMAL);

    cvCreateTrackbar("Circle resolution", "centers", &circle_resolution, 20, NULL);

    cvCreateTrackbar("Minimum radius", "centers", &min_radius, 500, NULL);

    cvCreateTrackbar("Maximum radius", "centers", &max_radius, 500, NULL);

    while (cvWaitKey(50) != ' ') {

        ballcenter = Vec3f(0, 0, 0);


        int morph_elem = 0;
        int morph_size = 10;
        Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        Mat copy = image.clone();
        //morphologyEx(src_gray, src_gray, MORPH_CLOSE, element);
        //imshow("finding ball", src_gray);
        HoughCircles(src_gray, circles_ball, CV_HOUGH_GRADIENT, (circle_resolution != 0 ? circle_resolution : 1), src_gray.rows/8, 200, 100, min_radius, max_radius);



        // taking biggest radius

        for (int i = 0; i < circles_ball.size(); i++) {
            if (circles_ball[i][2] > ballcenter[2]) {
                ballcenter = circles_ball[i];
                std::cout << "new biggest ball " << ballcenter[2] << std::endl;
            }


            Point center(cvRound(circles_ball[i][0]), cvRound(circles_ball[i][1]));
            float radius = cvRound(circles_ball[i][2]);
            // circle center
            circle(copy, center, 3, Scalar(0,255,0), -1, 8, 0);
            // circle outline
            circle(copy, center, radius, Scalar(0,0,255), 3, 8, 0);

        }

        Point center(cvRound(ballcenter[0]), cvRound(ballcenter[1]));
        float radius = cvRound(ballcenter[2]);
        // circle center
        circle(copy, center, 3, Scalar(0,255,0), -1, 8, 0);
        // circle outline
        circle(copy, center, radius, Scalar(255,0,0), 3, 8, 0);


        imshow("centers", copy);

        std::cout << "found " << circles_ball.size() << " balls " << std::endl;

    }
    if (circles_ball.size() == 0) {
        std::cerr << "Could'nt find the ball, aborting" << std::endl;
        exit(1);
    }

    destroyWindow("centers");

    Point center(cvRound(ballcenter[0]), cvRound(ballcenter[1]));
    float radius = cvRound(ballcenter[2]);
    // circle center
    circle(image, center, 3, Scalar(0,255,0), -1, 8, 0);
    // circle outline
    circle(image, center, radius, Scalar(0,0,255), 3, 8, 0);


    Point unit_y = center - Point(0, radius);

    line(image, center, unit_y, 255);

    std::vector<MarkerPair> markers_euclidians = euclidianProject(good_pairs, center, radius);



    return markers_euclidians;

}


int main(int argc, char** argv)
{
    if (argc < 3)
    {
        printf("usage: calibration <image1> <image2> <imageN>\n");
        return -1;
    }

    std::vector<std::vector<MarkerPair> > imageData = std::vector<std::vector<MarkerPair> >(argc - 1); 

    for (int i = 0; i < argc - 1; i++) {
        imageData[i] = treatImage(argv[i + 1]);
    }


    std::map<int, std::map<int, Point2f> > points;


    for (int i = 0; i < argc - 2; i++) {
        RANSACCorespondances(points, i, imageData[i], imageData[i+1]);
    }

    std::cout << "# of points : " << points.size() << std::endl;

    std::map<int, std::vector<string> > pointsdat;
    std::map<int, std::vector<string> > idmat;

    RNG rng(12345);

    std::vector<Mat> testoutput = std::vector<Mat>(argc - 1); 

    std::ofstream file;
    file.open("Res.dat");

    for (int i = 0; i < argc - 1; i++) {
        testoutput[i] = imread(argv[i + 1], 1);
        //resize(testoutput[i], testoutput[i], Size(), 0.3, 0.3);
        file << testoutput[i].cols << " " << testoutput[i].rows << std::endl;
    }

    file.close();

    for (std::map<int, std::map<int, Point2f> >::iterator i = points.begin(); i != points.end(); i++) {

        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        for (int k = 0; k < argc - 1; k++) {
            if (i->second.count(k) == 1) {
                idmat[k].push_back("1");
                pointsdat[k*3].push_back(stringify(i->second[k].x));
                pointsdat[k*3+1].push_back(stringify(i->second[k].y));
                pointsdat[k*3+2].push_back("1");
                circle(testoutput[k], i->second[k], 3, color, -1, 8, 0);


            }
            else {
                idmat[k].push_back("0");
                pointsdat[k*3].push_back("nan");
                pointsdat[k*3+1].push_back("nan");
                pointsdat[k*3+2].push_back("nan");
            }

        }
    }

    for (int i = 0; i < argc - 1; i++) {
        std::stringstream s;
        s << "camera " << i;
        imshow(s.str(), testoutput[i]);
    }

    file.open("points.dat");

    for (std::map<int, std::vector<string> >::iterator i = pointsdat.begin(); i != pointsdat.end(); i++) {
        for (std::vector<string>::iterator j = i->second.begin(); j != i->second.end(); j++) {
            file << j->c_str() << " ";
        }

        file << std::endl;
    }
    file.close();

    file.open("IdMat.dat");

    for (std::map<int, std::vector<string> >::iterator i = idmat.begin(); i != idmat.end(); i++) {
        for (std::vector<string>::iterator j = i->second.begin(); j != i->second.end(); j++) {
            file << j->c_str() << " ";
        }

        file << std::endl;
    }
    file.close();


    while (true) {
        cvWaitKey(500);
    }

    return 0;
}