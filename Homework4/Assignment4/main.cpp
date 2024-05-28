#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 递归实现阶乘

int factorial(int n)
{
    if (n == 0) return 1;
    else return n * factorial(n - 1);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int n =control_points.size() - 1;  // n+1 个控制点
    auto point = control_points[0] * std::pow((1 - t), n);  // 初始化第一个点
    for (int i = 1; i <= n; i++)
    {
        point += control_points[i] * factorial(n) / (factorial(i) * factorial(n-i)) * std::pow(t, i) * std::pow(1-t, n-i);
    }
    return point;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0.0f; t <= 1.0f; t += 0.001f)
    {
        auto point = recursive_bezier(control_points, t);

        // [1] 是黄线 [2] 是红线

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        std::vector<cv::Point2f> points(4);
        points[0] = cv::Point2f(std::floor(point.x + 0.5f), std::floor(point.y + 0.5f));
        points[1] = cv::Point2f(std::floor(point.x + 0.5f), std::floor(point.y - 0.5f));
        points[2] = cv::Point2f(std::floor(point.x - 0.5f), std::floor(point.y + 0.5f));
        points[3] = cv::Point2f(std::floor(point.x - 0.5f), std::floor(point.y - 0.5f));

        // 因为是向下取整，所以距离采样点最近的像素点坐标是 +0.5 后的点，取这个距离作为参照

        cv::Point2f distance = points[0] - point;
        float d0 = std::sqrt(distance.x * distance.x + distance.y * distance.y);

        for (auto p : points)
        {
            cv::Point2f d = p - point;
            float percent = d0 / std::sqrt(d.x * d.x + d.y * d.y);
            float color = window.at<cv::Vec3b>(p.x, p.y)[1];
            // 取最大值效果更好
            color = std::max(color, 255 * percent);
            window.at<cv::Vec3b>(p.x, p.y)[1] = color;
        }
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

        if (control_points.size() == 4) 
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
