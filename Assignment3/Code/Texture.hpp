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
        return getColorBilinear(u, v);
        if (u < 0) u = 0;
        if (u > 1) v = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    
    Eigen::Vector3f getColorBilinear(float u, float v){
        if (u < 0) u = 0;
        if (u > 1) v = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u0 = std::max(0.0, std::floor(u_img - 0.5));
        auto u1 = std::floor(u_img + 0.5);
        auto v0 = std::max(0.0, std::floor(v_img - 0.5));
        auto v1 = std::floor(v_img + 0.5);
        auto s = (u_img - u0) / (u1 - u0);
        auto t = (v_img - v0) / (v1 - v0);
        auto color0 = image_data.at<cv::Vec3b>(u0, v0);
        auto color1 = image_data.at<cv::Vec3b>(u0, v1);
        auto color2 = image_data.at<cv::Vec3b>(u1, v0);
        auto color3 = image_data.at<cv::Vec3b>(u1, v1);
        auto u_color = color0 + s * (color1 - color0);
        auto v_color = color2 + s * (color3 - color2);
        auto color = u_color + t * (v_color - u_color);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
