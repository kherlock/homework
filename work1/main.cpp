#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/dense>
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

Eigen::Matrix4f get_model_matrix(float rotation_angle,float x,float y,float z)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f rotation;
    float angle=rotation_angle*MY_PI/180;
    Eigen::Matrix4f I=Eigen::Matrix4f::Identity();
    Eigen::Vector4f n;
    n<<x,y,z,0;
    Eigen::Matrix4f N;
    N<<0,-z,y,0,
       z,0,x,0,
       -y,x,0,0,
       0,0,0,1;
    rotation=cos(angle)*I+(1-cos(angle))*n*n.transpose()+sin(angle)*N;
    rotation(3,3)=1;
    model=rotation*model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mvp;
    mvp<<zNear,0,0,0,
         0,zNear,0,0,
         0,0,zNear+zFar,-zNear*zFar,
         0,0,1,0;
    Eigen::Matrix4f a;
    Eigen::Matrix4f b;
    float top=tan(eye_fov/2)*(-zNear);
    float bottom=-top;
    float right=aspect_ratio*top;
    float left=-right;
    a<<2/(right-left),0,0,0,
       0,2/(top-bottom),0,0,
       0,0,2/(zNear-zFar),0,
       0,0,0,1;
    b<<1,0,0,-(right+left)/2,
       0,1,0,-(top+bottom)/2,
       0,0,1,-(zNear+zFar)/2,
       0,0,0,1;
    projection=a*b*mvp*projection;
    return projection;
}


int main(int argc, const char** argv)
{
    float x,y,z;
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

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    std::cout<<"x:";
    std::cin>>x;
    std::cout<<"y:";
    std::cin>>y;
    std::cout<<"z:";
    std::cin>>z;
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle,0,0,1));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    while (key != 27) {
        
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle,x,y,z));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -1000));

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
