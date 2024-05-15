//
// Created by waxz on 23-1-6.
//


//https://github.com/kthohr/optim#installation-method-2-header-only-library
//https://optimlib.readthedocs.io/en/latest/settings.html
#include <chrono>

#define OPTIM_ENABLE_EIGEN_WRAPPERS

#include "optim.hpp"
//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>
//#include <autodiff/forward/real.hpp>
//#include <autodiff/forward/real/eigen.hpp>
#include <autodiff/reverse/var.hpp>
#include <autodiff/reverse/var/eigen.hpp>

#include "math/transform/eigen_transform.h"
#include "math/geometry/normal_estimation_3d.h"

autodiff::var
opt_fnd(const autodiff::ArrayXvar &x) {
    return (x(0) - 0.9) * (x(0) - 0.9)
           + (x(1) - 1.5) * (x(1) - 1.5)
           + (x(2) - 1.8) * (x(2) - 1.8);
}


double
opt_fn(const Eigen::VectorXd &x, Eigen::VectorXd *grad_out, void *opt_data) {

    autodiff::ArrayXvar xd = x.eval();

    autodiff::var y = opt_fnd(xd);

    if (grad_out) {
        Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

        *grad_out = grad_tmp;
    }

    return autodiff::val(y);
}


struct PlaneNormTarget{
    // plane center
    Eigen::Matrix<double,3,1> center;
    // center + normal
    Eigen::Matrix<double,3,1> center_to_norm;

    size_t major_axis = 0;
    // center(major_axis) = target_position
    double target_position = 0.0;
    // norm(major_axis) = target_norm
    double target_norm = 1.0;



};

autodiff::var
calib_plane_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<PlaneNormTarget>& targets, const std::vector<double>& weights ){
    autodiff::var r = 0.0;

    auto sensor_pose =transform::createSe3<autodiff::var>(x(0),x(1),x(2),x(3),x(4),x(5),x(6));

    Eigen::Matrix<autodiff::var,3,1> center;
    Eigen::Matrix<autodiff::var,3,1> center_absolute;

    Eigen::Matrix<autodiff::var,3,1> center_to_norm;
    Eigen::Matrix<autodiff::var,3,1> center_to_norm_absolute;

    Eigen::Matrix<autodiff::var,3,1> norm;


    for(auto& t: targets){
        center = t.center.template cast<autodiff::var>();
        center_to_norm = t.center_to_norm.template cast<autodiff::var>();

        center_absolute = sensor_pose*center;
        center_to_norm_absolute = sensor_pose*center_to_norm;

        norm = center_to_norm_absolute - center_absolute;

        r += (center_absolute(t.major_axis) - t.target_position)*(center_absolute(t.major_axis) - t.target_position)*weights[0]
            // +(norm(t.major_axis) - t.target_norm)*(norm(t.major_axis) - t.target_norm)*weights[1]
                ;
    }

    return r;
}
struct CalibPlaneCostFunction{
    std::vector<double> weights;
    std::vector<PlaneNormTarget> targets;
    CalibPlaneCostFunction(){
    }

    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = calib_plane_opt_fnd(xd,targets, weights);

        if (grad_out) {
            Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

            *grad_out = grad_tmp;
        }

        return autodiff::val(y);
    }

};
struct Point{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    float r = 0.0;
};


autodiff::var
circle_opt_fnd(const autodiff::ArrayXvar& x, const std::vector<Point>& points,int point_num, float radius_2)
{
    autodiff::var r = 0.0;
    autodiff::var t = 0.0;

    for(int i = 0 ; i < point_num ;i++){
        t = (x(0) - points[i].x)*(x(0) - points[i].x) + (x(1) - points[i].y)*(x(1) - points[i].y) - radius_2;
        // square error is fast than absolute error
        r += t*t;
//        r += abs(t);

    }
    return r;
}

struct CircleCostFunction{

    const std::vector<Point>& points;
    int point_num;
    float radius = 0.1;
    float radius_2 = radius;


    CircleCostFunction(const std::vector<Point>& t_points, int t_point_num, float t_radius):points(t_points),point_num(t_point_num), radius(t_radius),radius_2(radius*radius){
    }



    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd* grad_out, void* opt_data)
    {

        autodiff::ArrayXvar xd = x.eval();

        autodiff::var y = circle_opt_fnd(xd,points,point_num,radius_2);

        if (grad_out) {
            Eigen::VectorXd grad_tmp = autodiff::gradient(y, xd);

            *grad_out = grad_tmp;
        }

        return autodiff::val(y);
    }
};

void test_1() {
    Eigen::VectorXd x(5);
    x << 1, 2, 3, 4, 5;

    bool success = optim::bfgs(x, opt_fn, nullptr);

    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "solution: x = \n" << x << std::endl;

}

void test_2(){

    int point_num = 100;
    std::vector<Point> points(point_num);
    float angle_inc = 2.0f*M_PI/float(point_num);

    float sample_cx = 0.13, sample_cy = 0.06, sample_radius = 0.1;

    for(int i = 0 ; i < point_num;i++){
        points[i].x = sample_cx + sample_radius*std::cos(i*angle_inc);
        points[i].y = sample_cy + sample_radius*std::sin(i*angle_inc);

    }

    Eigen::VectorXd x(2);
    x << 0.05, 0.05;

    CircleCostFunction opt_fn_obj(points,point_num, sample_radius) ;



    // run Adam-based optim

//    optim::algo_settings_t settings;
//    optim::bfgs_settings_t settings;
    optim::algo_settings_t settings;
    settings.iter_max = 20;
    settings.bfgs_settings.wolfe_cons_1 = 1e-4;
    settings.bfgs_settings.wolfe_cons_2 = 0.8;

    settings.print_level = 1;

    settings.vals_bound = true;

    settings.lower_bounds = optim::ColVec_t::Zero(2);
    settings.lower_bounds(0) = 0.0;
    settings.lower_bounds(1) = 0.0;

    settings.upper_bounds = optim::ColVec_t::Zero(2);
    settings.upper_bounds(0) = 0.15;
    settings.upper_bounds(1) = 0.15;

    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;


    start = std::chrono::system_clock::now();
    bool success = optim::bfgs(x, opt_fn_obj, nullptr,settings);
    end = std::chrono::system_clock::now();
    elapsed_seconds = end-start;
    if (success) {
        std::cout << "bfgs: reverse-mode autodiff test completed successfully.\n" << std::endl;
    } else {
        std::cout << "bfgs: reverse-mode autodiff test completed unsuccessfully.\n" << std::endl;
    }

    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;
    std::cout << "solution: x = \n" << x << std::endl;

}


namespace test1{


}
int main(int argc, char **argv) {

//    test_1();
//    test_2();

}
