#include <cstdio>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace cv;

struct MarkerPair
{
    int first_index;
    int second_index;
    Point2f   first;
    Point2f   second;
    float         distance;

    MarkerPair(int first_index_, int second_index_, Point2f first_, Point2f second_, float distance_): first_index(first_index_), second_index(second_index_), first(first_), second(second_), distance(distance_) {}

    bool operator < (const MarkerPair& other) const
    {
        return (distance < other.distance);
    }
};

// Compute the distance between two 2D points
double distanceBetweenPoints(Point2f p1, Point2f p2) {
    // sqrt((x1 - x2)^2 + (y1 - y2)^2)
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        printf("usage: calibration <image>\n");
        return -1;
    }


    // Load the image
    Mat image;
    image = imread(argv[1], 1);

    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }


    resize(image, image, Size(), 0.1, 0.1);

    // Go into HSV space
    Mat hsv;
    cvtColor(image, hsv, CV_BGR2HSV);

    Mat reddot;
    // select red
    inRange(hsv, Scalar(0, 90, 90), Scalar(7, 255, 255), reddot);
    imshow("original", image);
    imshow("HSV", hsv);
    imshow("Red dot", reddot);


    Mat src_gray;
    cvtColor(image, src_gray, CV_BGR2GRAY);

    std::vector<Point2f> corners;

 
    goodFeaturesToTrack(reddot, corners, 500, 0.01, 10);
    std::cout << corners.size() << std::endl;

    for (size_t idx = 0; idx < corners.size(); idx++) {
            circle(image, corners.at(idx), 3,255, -1);
            std::cout << corners.at(idx) << std::endl;
    }

    /// Set the neeed parameters to find the refined corners
    Size winSize = Size(5, 5);
    Size zeroZone = Size(-1, -1);
    TermCriteria criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

    cornerSubPix(src_gray, corners, winSize, zeroZone, criteria);

    for (size_t idx = 0; idx < corners.size(); idx++) {
            circle(src_gray, corners.at(idx), 3, 255, -1);
            std::cout << corners.at(idx) << std::endl;
    }

    /// Show your results
    namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", image );

    imshow( "New corners", src_gray);

    std::vector<Point2f> circles = corners;

    std::pair<Point2f, Point2f> circle_pair;

    std::vector<MarkerPair> pairs;

    for (size_t i = 0; i < circles.size(); i++) {
        // find the closest point for a given point

        Point2f best_match;

        for (size_t j = 0; j < circles.size(); j++) {

            if (j == i) {
                continue;
            }

            double current_dist = distanceBetweenPoints(circles[i], circles[j]);

            pairs.push_back(MarkerPair(i, j, circles[i], circles[j], current_dist));
        }

        // if (min_dist != -1) {
        //     // Computing ROI between the two matches
        //     std::ostringstream oss;
        //     oss << "window " << i;

        //     Mat roi = src_gray(Rect(min(circles[i].x, best_match.x), min(circles[i].y, best_match.y), max(circles[i].x, best_match.x) - min(circles[i].x, best_match.x), max(circles[i].y, best_match.y) - min(circles[i].y, best_match.y)));

        //     vector<Vec2f> lines;

        // }


        //std::cout << "best match is " << best_match << " with min dist " << min_dist << std::endl;
    }

    // sort by distance
    std::sort(pairs.begin(), pairs.end());

    float mean_distance = 0;

    std::set<int> treated;

    std::vector<MarkerPair> good_pairs;
    int nbr_good_pairs = 0;



    for (std::vector<MarkerPair>::iterator i = pairs.begin(); i != pairs.end(); i++)  {
        std::cout << i->distance << std::endl;

        if (!treated.count(i->first_index) && !treated.count(i->second_index)) {

            if (mean_distance == 0 || i->distance <= mean_distance*1.10) {
                nbr_good_pairs++;
                mean_distance = ((mean_distance * (nbr_good_pairs - 1)) + i->distance) / nbr_good_pairs;
                treated.insert(i->first_index);
                treated.insert(i->second_index);
                good_pairs.push_back(*i);
            }
        }
    }

    for (std::vector<MarkerPair>::iterator i = good_pairs.begin(); i != good_pairs.end(); i++) {
        std::cout << "good pair " << std::endl;
        line(image, i->first, i->second, 255);
    }

    imshow( "Good pairs", image );


    waitKey(0);

    return 0;
}