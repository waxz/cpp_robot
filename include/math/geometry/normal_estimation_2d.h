//
// Created by waxz on 23-1-30.
//

#ifndef SCAN_REPUBLISHER_NORMAL_ESTIMATION_2D_H
#define SCAN_REPUBLISHER_NORMAL_ESTIMATION_2D_H

#define EIGEN_MAX_ALIGN_BYTES 32
#define EIGEN_MAX_STATIC_ALIGN_BYTES 32
#include <Eigen/Core>
#include <Eigen/Dense>

namespace perception{

    struct NormalEst2d{
        using Scalar = float;
        Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
        Eigen::Matrix<Scalar, 2, 1> K{0.0, 0.0};
        Eigen::Matrix<Scalar, 4, 1> centroid;
        Eigen::Matrix<Scalar, 2, 2> covariance_matrix;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver{2};
        Eigen::Matrix<Scalar, 2, 1> view_point{0.0, 0.0};

        int point_count = 0;
        int min_point_num = 3;

        void setMinNum( int num ){

            min_point_num = (num >3) ? num :min_point_num;

        }

        void setViewerPoint(float x, float y){

            view_point(0) = x;
            view_point(1) = y;
        }
        void reset(){
            point_count = 0;
            accu = Eigen::Matrix<Scalar, 1, 5, Eigen::RowMajor>::Zero ();
        }

        void addCenter(float x, float y){

            K.x() = x;
            K.y() = y;
        }
        void addPoint(float x, float y){
            x -= K.x();
            y -= K.y();

            accu [0] += x * x;
            accu [1] += x * y;
            accu [2] += y * y;
            accu [3] += x;
            accu [4] += y;
            point_count ++;
        }

        int compute(float& normal_x,float& normal_y,float& normal_z,float& normal_angle ){

            if(point_count <=min_point_num){
                normal_x  = normal_y  = normal_z  =  normal_angle = std::numeric_limits<float>::quiet_NaN ();

                return -1;
            }
            accu /= static_cast<Scalar> (point_count);
            centroid[0] = accu[3] + K.x(); centroid[1] = accu[4] + K.y(); centroid[2] = 0.0;
            centroid[3] = 1;
            covariance_matrix.coeffRef (0) = accu [0] - accu [3] * accu [3];//xx
            covariance_matrix.coeffRef (1) = accu [1] - accu [3] * accu [4];//xy
            covariance_matrix.coeffRef (3) = accu [2] - accu [4] * accu [4];//yy
            covariance_matrix.coeffRef (2) = covariance_matrix.coeff (1);//yx

            eig_solver.compute(covariance_matrix);

            auto& eigen_values = eig_solver.eigenvalues();
            auto& eigen_vectors = eig_solver.eigenvectors() ;

            normal_x = eigen_vectors(0,0);
            normal_y = eigen_vectors(1,0);
            normal_z = 0.0;

            Eigen::Matrix <float, 2, 1> normal (normal_x, normal_y);
            Eigen::Matrix <float, 2, 1> vp ( view_point.x() - K.x(), view_point.y() - K.y());
            normal.normalize();
            vp.normalize();
            // Dot product between the (viewpoint - point) and the plane normal
            float cos_theta = vp.dot (normal);
            // Flip the plane normal
            if (cos_theta < 0)
            {
                normal = -normal;
            }
            normal_x = normal(0);
            normal_y = normal(1);
            normal_angle = std:: acos(std::abs(cos_theta));

            return 0;

        }


    };


}

#endif //SCAN_REPUBLISHER_NORMAL_ESTIMATION_2D_H
