#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 8) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        // (1+t)^3*p0 + 3t*(1-t)^2*p1 + 3t^2(1-t)*p2 + t^3*p3;
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2)
        return control_points[0] * (1 - t) + control_points[1] * t;
    std::vector<cv::Point2f> points;
    for (int i = 0; i < control_points.size() - 1; i++){
        points.push_back(control_points[i] * (1-t) + control_points[i + 1] * t);
    }
    return recursive_bezier(points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0.0; t <= 1; t += 0.0001){
        auto point = recursive_bezier(control_points, t);
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        cv::Point2i neigh0(point.x - 0.5, point.y - 0.5);
        cv::Point2i neigh1(point.x + 0.5, point.y - 0.5);
        cv::Point2i neigh2(point.x - 0.5, point.y + 0.5);
        cv::Point2i neigh3(point.x + 0.5, point.y + 0.5);
        printf("p:%f %f, p0: %d %d, p1: %d %d, p2: %d %d, p3: %d %d\n", point.x, point.y, 
            neigh0.x, neigh0.y, neigh1.x, neigh1.y, neigh2.x, neigh2.y, neigh3.x, neigh3.y);
        float d1 = std::pow(neigh0.x - point.x, 2) + std::pow(neigh0.y - point.y, 2);
        float d2 = std::pow(neigh1.x - point.x, 2) + std::pow(neigh1.y - point.y, 2);
        float d3 = std::pow(neigh2.x - point.x, 2) + std::pow(neigh2.y - point.y, 2);
        float d4 = std::pow(neigh3.x - point.x, 2) + std::pow(neigh3.y - point.y, 2);
        float d = std::min(std::min(std::min(d1,d2), d3), d4);
        printf("%f %f %f %f\n", d / d1, d / d2, d / d3, d / d4);
        window.at<cv::Vec3b>(neigh0.y, neigh0.x)[1] = std::max((uchar)(255 * d / d1), window.at<cv::Vec3b>(neigh0.y, neigh0.x)[1]);
        window.at<cv::Vec3b>(neigh1.y, neigh1.x)[1] = std::max((uchar)(255 * d / d2), window.at<cv::Vec3b>(neigh1.y, neigh1.x)[1]);
        window.at<cv::Vec3b>(neigh2.y, neigh2.x)[1] = std::max((uchar)(255 * d / d3), window.at<cv::Vec3b>(neigh2.y, neigh2.x)[1]);
        window.at<cv::Vec3b>(neigh3.y, neigh3.x)[1] = std::max((uchar)(255 * d / d4), window.at<cv::Vec3b>(neigh3.y, neigh3.x)[1]);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 8) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
