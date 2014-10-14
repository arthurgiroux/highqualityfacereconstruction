#include <cstdio>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry> 


using namespace cv;


#define MEAN_DEVIATION_DISTANCE_THRESH 1.30

// structure that represents a fiducial
struct MarkerPair
{
    int first_index; 
    int second_index;
    Point2f   first; // coordinate of the center of one of the marker
    Point2f   second;
    float         distance; // distance between the two center of the marker

    MarkerPair(int first_index_, int second_index_, Point2f first_, Point2f second_, float distance_): first_index(first_index_), second_index(second_index_), first(first_), second(second_), distance(distance_) {}

    bool operator < (const MarkerPair& other) const
    {
        return (distance < other.distance);
    }
};

// structure that represents a marker in the euclidian space
struct MarkerEuclidian
{
    Point3f   first; // coordinate of the center of one of the marker
    Point3f   second;

    MarkerEuclidian(Point3f first_, Point3f second_): first(first_), second(second_) {}

};

// Compute the distance between two 2D points
double distanceBetweenPoints(Point2f p1, Point2f p2) {
    // sqrt((x1 - x2)^2 + (y1 - y2)^2)
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
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
            if (mean_distance == 0 || i->distance <= mean_distance * MEAN_DEVIATION_DISTANCE_THRESH) {
                nbr_good_pairs++;
                mean_distance = ((mean_distance * (nbr_good_pairs - 1)) + i->distance) / nbr_good_pairs;
                treated.insert(i->first_index);
                treated.insert(i->second_index);
                good_pairs.push_back(*i);
            }
            else {
                std::cout << "too much deviation " << i->distance << " " << mean_distance << " " << mean_distance* MEAN_DEVIATION_DISTANCE_THRESH << std::endl;
            }
        }
    }

    for (std::vector<MarkerPair>::iterator i = good_pairs.begin(); i != good_pairs.end(); i++) {
        std::cout << "good pair " << std::endl;
    }

    return good_pairs;
}


std::vector<MarkerEuclidian> euclidianProject(std::vector<MarkerPair> good_pairs, Point center, float radius) {
    std::vector<MarkerEuclidian> markers_euclidians;

    std::cout << " radius " << radius << std::endl;

    for (std::vector<MarkerPair>::iterator i = good_pairs.begin(); i != good_pairs.end(); i++) {
        // We project into the euclidian space centered at the ball
        // for x and y we just project into the vertical/horizontal axis
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
        markers_euclidians.push_back(MarkerEuclidian(first, second));
    }

    return markers_euclidians;
}


// RANSAC 

std::vector<MarkerEuclidian> RANSACCorespondances(std::vector<MarkerEuclidian> first_markers, std::vector<MarkerEuclidian> second_markers) {

    // Let's take a point from the first markers

    float min_error = INFINITY;

    std::vector<MarkerEuclidian> correspondances;

    for (std::vector<MarkerEuclidian>::iterator i = first_markers.begin(); i != first_markers.end(); i++) {

        for (std::vector<MarkerEuclidian>::iterator j = second_markers.begin(); j != second_markers.end(); j++) {

            Eigen::Vector3d a1(0, 0, 0);
            Eigen::Vector3d a2(i->first.x, i->first.y, i->first.z);
            Eigen::Vector3d a3(i->second.x, i->second.y, i->second.z);
                    
            Eigen::Vector3d b1(0, 0, 0);
            Eigen::Vector3d b2(j->first.x, j->first.y, j->first.z);
            Eigen::Vector3d b3(j->second.x, j->second.y, j->second.z);

            Eigen::Matrix<double, 3, 3> start, end;
            start.col(0) = a1;
            start.col(1) = a2;
            start.col(2) = a3;

            end.col(0) = b1;
            end.col(1) = b2;
            end.col(2) = b3;

            std::cout << Eigen::umeyama(start,end) << std::endl;
        }
    }

    return correspondances;
}



std::vector<MarkerEuclidian> treatImage(char* file) {
        // Load the image
    Mat image;
    image = imread(file, 1);

    if (!image.data)
    {
        printf("No image data \n");
        return std::vector<MarkerEuclidian>();
    }


    resize(image, image, Size(), 0.1, 0.1);

    // Go into HSV space
    Mat hsv;
    cvtColor(image, hsv, CV_BGR2HSV);

    Mat reddot;
    // select red
    inRange(hsv, Scalar(0, 90, 90), Scalar(7, 255, 255), reddot);
    //imshow("original", image);
    //imshow("HSV", hsv);
    //imshow("Red dot", reddot);


    Mat src_gray;
    cvtColor(image, src_gray, CV_BGR2GRAY);

    std::vector<Point2f> corners;

 
    // retrieve all the points
    goodFeaturesToTrack(reddot, corners, 500, 0.01, 10);

    /// Set the neeed parameters to find the refined corners
    Size winSize = Size(5, 5);
    Size zeroZone = Size(-1, -1);
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

    // Refine the point center in subpixel accuracy
    cornerSubPix(src_gray, corners, winSize, zeroZone, criteria);

    for (size_t idx = 0; idx < corners.size(); idx++) {
            circle(image, corners.at(idx), 3, 255, -1);
            std::cout << corners.at(idx) << std::endl;
    }


    // Let's match the points together
    std::vector<MarkerPair> good_pairs = matchPoints(corners);


    for (size_t idx = 0; idx < good_pairs.size(); idx++) {
        line(image, good_pairs.at(idx).first, good_pairs.at(idx).second, 255);
    }

    imshow("blabla", image);
    // Setting up the euclidian coordinate system with the origin at the center of the ball


    // finding the center of the ball
    vector<Vec3f> circles_ball;
    HoughCircles(src_gray, circles_ball, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0);

    if (circles_ball.size() == 0) {
        std::cerr << "Could'nt find the ball, aborting" << std::endl;
        exit(1);
    }

    Vec3f ballcenter = circles_ball[0];

    Point center(cvRound(ballcenter[0]), cvRound(ballcenter[1]));
    float radius = cvRound(ballcenter[2]);
    // circle center
    circle(image, center, 3, Scalar(0,255,0), -1, 8, 0);
    // circle outline
    circle(image, center, radius, Scalar(0,0,255), 3, 8, 0);


    Point unit_y = center - Point(0, radius);

    line(image, center, unit_y, 255);

    std::vector<MarkerEuclidian> markers_euclidians = euclidianProject(good_pairs, center, radius);


    imshow("centers", image);

    return markers_euclidians;

}


int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("usage: calibration <image>\n");
        return -1;
    }

    std::vector<MarkerEuclidian> firstview = treatImage(argv[1]);
    std::vector<MarkerEuclidian> secondview = treatImage(argv[2]);

    RANSACCorespondances(firstview, secondview);



    waitKey(0);

    return 0;
}