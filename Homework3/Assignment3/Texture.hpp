//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        // limit uv range [0, 1]
        u = std::clamp(u, 0.0f, 0.999999f);
        v = std::clamp(v, 0.0f, 0.999999f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    // 双线性插值

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        // limit uv range [0, 1]
        u = std::clamp(u, 0.0f, (width-0.5f)/width);
        v = std::clamp(v, 0.0f, (height-0.5f)/height);
        
        float u_img = u * width;
        float v_img = (1-v) * height;

        // 找到中心坐标
        int cx = u_img;
        int cy = v_img;
        cx = (u_img - cx) > 0.5f ? std::ceil(u_img) : std::floor(u_img);
        cy = (v_img - cy) > 0.5f ? std::ceil(v_img) : std::floor(v_img);

        // 注意 image_data 第一个坐标对应 v，参考getColor()
        // 并且uv map和image的 v 是相反方向 ！！
        auto u00 = image_data.at<cv::Vec3b>(cy+0.5f, cx-0.5f);
        auto u10 = image_data.at<cv::Vec3b>(cy+0.5f, cx+0.5f);
        auto u01 = image_data.at<cv::Vec3b>(cy-0.5f, cx-0.5f);
        auto u11 = image_data.at<cv::Vec3b>(cy-0.5f, cx+0.5f);

        float s = u*width - (cx - 0.5f);
        float t = (1-v)*height - (cy-0.5f);

        auto u0 = (1-s)*u00 + s*u10;
        auto u1 = (1-s)*u01 + s*u11;

        auto result = (1-t)*u1 + t*u0;
        return Eigen::Vector3f(result[0], result[1], result[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
