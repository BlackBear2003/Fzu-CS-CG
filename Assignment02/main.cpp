#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

float step = 0.001;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void draw(cv::Point2f point, cv::Mat &window) {
    window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
}

void draw_with_anti_aliasing(cv::Point2f point, cv::Mat &window) {
    for (int dx = 0; dx < 2; ++dx) {
        for (int dy = 0; dy < 2; ++dy) {

            int px = floor(point.x) + dx;
            int py = floor(point.y) + dy;

            cv::Point2f p = cv::Point2f(px, py);
            auto w_p = p - point;

            double max_d = sqrt(2);
            double d = sqrt(w_p.x * w_p.x + w_p.y * w_p.y);
            double weight = 1 - d / max_d;
            window.at<cv::Vec3b>(p.y, p.x)[1] = std::fmax(255 * weight, window.at<cv::Vec3b>(p.y, p.x)[1]);
        }
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
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm

    if (control_points.size() == 1) {
        return control_points[0];
    }

    std::vector<cv::Point2f> points;

    for (long i = 1; i < control_points.size(); i++) {
        auto p1 = control_points[i-1];
        auto p2 = control_points[i];
        // the de Casteljau's algorithm
        auto p = (1-t) * p1 + t * p2;
        points.push_back(p);
    }

    return recursive_bezier(points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += step) {
        auto point = recursive_bezier(control_points, t);

        draw_with_anti_aliasing(point, window);
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
        window.setTo(0);
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() >= 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(1);

        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
