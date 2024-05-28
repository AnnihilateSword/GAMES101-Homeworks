#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// 绕过原点的任意轴的旋转变换矩阵（罗德里格斯旋转公式）
Eigen::Matrix4f GetRotation(Eigen::Vector3f k, float rotAngle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // 归一化旋转轴
    k.normalize();
    rotAngle = rotAngle * MY_PI / 180;

    //罗德里格斯旋转
    Eigen::Matrix3f rotate = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f N = Eigen::Matrix3f::Identity();
	N << 0, -k.z(), k.y(),
		k.z(), 0, -k.x(),
		-k.y(), k.x(), 0;

    //代入罗德里格斯旋转公式
	rotate = cos(rotAngle) * Eigen::Matrix3f::Identity() + (1 - cos(rotAngle)) * k * k.transpose() + sin(rotAngle) * N;
    model << rotate(0, 0), rotate(0, 1), rotate(0, 2), 0,
            rotate(1, 0), rotate(1, 1), rotate(1, 2), 0,
            rotate(2, 0), rotate(2, 1), rotate(2, 2), 0,
            0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    rotation_angle = rotation_angle * MY_PI / 180;
    // cosx -sinx 0 0
    // sinx  cosx 0 0
    // 0     0    1 0
    // 0     0    0 1
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate << cos(rotation_angle), -sin(rotation_angle), 0, 0,
            sin(rotation_angle), cos(rotation_angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // Mortho
    // 2/(r-l)  0       0       -(r-l)/2
    // 0        2/(t-b) 0       -(t-b)/2
    // 0        0       2/(n-f) -(n-f)/2
    // 0        0       0       1
    // Mprojection => Mortho
    // n  0  0  0
    // 0  n  0  0
    // 0  0 n+f -nf
    // 0  0  1  0

    eye_fov = eye_fov * MY_PI / 180;
    float t = zNear * tan(eye_fov/2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    // 正交变换矩阵
    Eigen::Matrix4f Mortho = Eigen::Matrix4f::Identity();
    Mortho << 2/(r-l), 0, 0, -(r-l)/2,
            0, 2/(t-b), 0, -(t-b)/2,
            0, 0, 2/(zNear-zFar), -(zNear-zFar)/2,
            0, 0, 0, 1;

    // 投影矩阵 => 正交矩阵
    Eigen::Matrix4f Mp2o = Eigen::Matrix4f::Identity();
    Mp2o << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear+zFar, -zNear*zFar,
            0, 0, 1, 0;

    // 因为 Opencv 坐标系的原因，绘制出来的三角形是倒着的，这里需要再乘一个矩阵
    Eigen::Matrix4f Mz(4, 4);
    Mz << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    projection = Mortho * Mp2o * Mz * projection;

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
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        // 测试罗德里格斯旋转变换矩阵
        r.set_model(GetRotation(Eigen::Vector3f(1.0f, 0.0f, 0.0f), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
