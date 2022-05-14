#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f gaze, Eigen::Vector3f view_up)
{
    // 变换到相机视角 view transformation
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    gaze.normalize();
    view_up.normalize();
    Eigen::Vector3f z = -1 * gaze;
    std::cout << z << std::endl;
    Eigen::Vector3f x = view_up.cross(z);
    x.normalize();
    Eigen::Vector3f y = z.cross(x);
    y.normalize();
    Eigen::Matrix4f translate;
    Eigen::Matrix4f move;
    move << 1, 0, 0, -eye_pos[0],
            0, 1, 0, -eye_pos[1],
            0, 0, 1, -eye_pos[2],
            0, 0, 0, 1;
    translate << x[0], y[0], z[0], 0,
                 x[1], y[1], z[1], 0,
                 x[2], y[2], z[2], 0,
                 0, 0, 0, 1;

    view = view * translate * move;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // 旋转模型 model transformation
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation;
    rotation << cos(rotation_angle * MY_PI / 180.0), -sin(rotation_angle * MY_PI / 180.0), 0, 0,
                sin(rotation_angle * MY_PI / 180.0), cos(rotation_angle * MY_PI / 180.0), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // projection transformation
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float fov_angle = eye_fov / 2.0 * MY_PI / 180.0;
    float t = abs(zNear) * tan(fov_angle);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;
    Eigen::Matrix4f persp;
    persp << zNear, 0, 0, 0,
             0, zNear, 0, 0,
             0, 0, zFar + zNear, -1 * zFar * zNear,
             0, 0, 1, 0;
    Eigen::Matrix4f orth;
    Eigen::Matrix4f scale;
    scale << 2.0 / (r - l), 0, 0, 0,
             0, 2.0 / (t - b), 0, 0,
             0, 0, 2.0 / (zNear - zFar), 0,
             0, 0, 0, 1;
    Eigen::Matrix4f move;
    move <<  1, 0, 0, -(l + r) / 2.0,
             0, 1, 0, -(b + t) / 2.0,
             0, 0, 1, -(zNear + zFar) / 2.0,
             0, 0, 0, 1;
    orth = scale * move;
    projection = orth * persp * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    float gaze_xaxis_angle = 0;
    float gaze_yaxis_angle = 0;
    Eigen::Vector3f gaze = {0, 0, -1}; // -z axis
    Eigen::Vector3f view_up = {0, 1, 0};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos, gaze, view_up));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        pos_id = r.load_positions(pos);
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        auto view_mat = get_view_matrix(eye_pos, gaze, view_up);
        r.set_view(view_mat);
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        std::cout << "z pos:" << pos[0][2] << std::endl;
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        if (key == 'c') {
            // z-y rotate camera
            gaze_yaxis_angle += 10;
            auto angle = gaze_yaxis_angle * MY_PI / 180.0;
            gaze[2] = -cos(angle);
            gaze[1] = sin(angle);
        }
        else if (key == 'v') {
            // z-y rotate camera
            gaze_yaxis_angle -= 10;
            auto angle = gaze_yaxis_angle * MY_PI / 180.0;
            gaze[2] = -cos(angle);
            gaze[1] = sin(angle);
        }
        if (key == 'z') {
            // x-z plane rotate camera
            gaze_xaxis_angle += 10;
            auto angle = gaze_xaxis_angle * MY_PI / 180.0;
            gaze[2] = -cos(angle);
            gaze[0] = sin(angle);
        }
        else if (key == 'x') {
            //  x-z plane rotate camera
            gaze_xaxis_angle -= 10;
            auto angle = gaze_xaxis_angle * MY_PI / 180.0;
            gaze[2] = -cos(angle);
            gaze[0] = sin(angle);
        }
        if (key == 'b') {
            gaze[2]++;
        }
        else if (key == 'n') {
            gaze[2]--;
        }
        if (key == 'q') {
            // change object z axis pos
            pos[0][2]++;
            pos[1][2]++;
            pos[2][2]++;
        }
        else if (key == 'e') {
            // change object z axis pos
            pos[0][2]--;
            pos[1][2]--;
            pos[2][2]--;
        }
        // change camera pos
        if (key == 'i') {
            // raise camera
            eye_pos[1]++;
        }
        else if (key == 'k') {
            // down camera
            eye_pos[1]--;
        } else if (key == 'j') {
            eye_pos[2]--;
        } else if (key == 'l') {
            eye_pos[2]++;
        }
    }

    return 0;
}
