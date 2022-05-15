// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v[3];
    v[0] = Vector3f(_v[0].x(), _v[0].y(), 1);
    v[1] = Vector3f(_v[1].x(), _v[1].y(), 1);
    v[2] = Vector3f(_v[2].x(), _v[2].y(), 1);
    Vector3f p(x, y, 1);
    auto ret = (v[1] - v[0]).cross(p - v[0]);
    ret.normalize();
    auto res = (v[2] - v[1]).cross(p - v[1]);
    res.normalize();
    if (ret != res) return false;
    res = (v[0] - v[2]).cross(p - v[2]);
    res.normalize();
    if (ret != res) return false;
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float left = INT_MAX, right = INT_MIN, top = INT_MIN, botton = INT_MAX;
    for (auto & node: v){
        left = std::min(node.x(), left);
        right = std::max(node.x(), right);
        top = std::max(node.y(), top);
        botton = std::min(node.y(), botton);
    }
    /* super sample 4xSSAA */
    if (depth_buf.size() == width * height){
        depth_buf.resize(4 * width * height);
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    for (int j = top; j >= botton; j--){
        for (int i = left; i <= right; i++){
            /* normal method */
            /* 
            // centor of pixel: (x + 0.5, y + 0.5)
            if (insideTriangle(i + 0.5, j + 0.5, tri)){
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                auto ind = (height - 1 - j) * width + i;
                // depth_buf won't larger than 1, but may error because of float accuracy
                if (depth_buf[ind] > 1.5 || depth_buf[ind] < z_interpolated) {
                    set_pixel(Vector3f(i, j, 1), t.getColor());
                    depth_buf[ind] = z_interpolated;
                }
            }
            */
            /* super sample 4xSSAA */
            int count = 0;
            if (SSAA(i + 0.25, j + 0.25, t, 4 * (i * width + j))) count++; // (i,j)
            if (SSAA(i + 0.25, j + 0.75, t, 4 * (i * width + j) + 1)) count++; // (i,j+1)
            if (SSAA(i + 0.75, j + 0.25, t, 4 * (i * width + j) + 2)) count++; // (i+1,j)
            if (SSAA(i + 0.75, j + 0.75, t, 4 * (i * width + j) + 3)) count++; // (i+1,j+1)
            if (count > 0){
                auto ind = (height-1-j)*width + i;
                // can override color and merge color
                set_pixel(Vector3f(i, j, 1), t.getColor() * count / 4 + frame_buf[ind] * (4 - count) / 4);
            }
        }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

bool rst::rasterizer::SSAA(float x, float y, const Triangle& t, int idx){
    if (insideTriangle(x, y, t.v)){
        auto v = t.toVector4();
        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
        z_interpolated *= w_reciprocal;
        if (depth_buf[idx] > 2 || depth_buf[idx] < z_interpolated) {
            if (depth_buf[idx] < 2) {
                std::cout << "override:" << depth_buf[idx] << "->" << z_interpolated << std::endl;
            }
            depth_buf[idx] = z_interpolated;
            return true;
        }
    }
    return false;
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on