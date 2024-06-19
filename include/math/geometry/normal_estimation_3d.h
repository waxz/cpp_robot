//
// Created by waxz on 23-2-20.
//

#ifndef SCAN_REPUBLISHER_NORMAL_ESTIMATION_3D_H
#define SCAN_REPUBLISHER_NORMAL_ESTIMATION_3D_H

#define EIGEN_MAX_ALIGN_BYTES 32
#define EIGEN_MAX_STATIC_ALIGN_BYTES 32
#include <Eigen/Core>
#include <Eigen/Dense>
#include "common/string_logger.h"
//#define MLOGI(...)
namespace perception {
    struct NormalEst3d{
        using Scalar = float;
        Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero ();
        Eigen::Matrix<Scalar, 3, 1> K{0.0, 0.0, 0.0};
        Eigen::Matrix<Scalar, 4, 1> centroid;
        Eigen::Matrix<Scalar, 3, 3> covariance_matrix;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver{3};
        Eigen::Matrix<Scalar, 3, 1> view_point{0.0, 0.0,0.0};

        int point_count = 0;
        int min_point_num = 3;

        void setMinNum( int num ){

            min_point_num = (num >3) ? num :min_point_num;

        }

        void setViewerPoint(float x, float y, float z){

            view_point(0) = x;
            view_point(1) = y;
            view_point(2) = z;

        }
        void reset(){
            point_count = 0;
            accu = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero ();
        }


        void addCenter(float x, float y, float z){

            K.x() = x;
            K.y() = y;
            K.z() = z;
        }
        void addPoint(float x, float y, float z){
            x -= K.x();
            y -= K.y();
            z -= K.z();

            accu [0] += x * x;
            accu [1] += x * y;
            accu [2] += x * z;
            accu [3] += y * y; // 4
            accu [4] += y * z; // 5
            accu [5] += z * z; // 8
            accu [6] += x;
            accu [7] += y;
            accu [8] += z;
            point_count ++;
        }

        int compute(float& normal_x,float& normal_y,float& normal_z,float& normal_angle ){

            if(point_count <=min_point_num){
                normal_x  = normal_y  = normal_z  =  normal_angle = std::numeric_limits<float>::quiet_NaN ();

                return -1;
            }
            accu /= static_cast<Scalar> (point_count);
            centroid[0] = accu[6] + K.x(); centroid[1] = accu[7] + K.y(); centroid[2] = accu[8] + K.z();
            centroid[3] = 1;
            covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
            covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
            covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
            covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
            covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
            covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
            covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
            covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
            covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);

            eig_solver.compute(covariance_matrix);

            auto& eigen_values = eig_solver.eigenvalues();
            auto& eigen_vectors = eig_solver.eigenvectors() ;

            normal_x = eigen_vectors(0,0);
            normal_y = eigen_vectors(1,0);
            normal_z = eigen_vectors(2,0);

            Eigen::Matrix <float, 3, 1> normal (normal_x, normal_y,normal_z);
//            Eigen::Matrix <float, 3, 1> vp ( view_point.x() - K.x(), view_point.y() - K.y(),   view_point.z() - K.z());

//            MLOGI("K: [%f,%f,%f]", K.x(), K.y(), K.z());
//            MLOGI("view_point: [%f,%f,%f]", view_point.x(), view_point.y(), view_point.z());

            Eigen::Matrix <float, 3, 1> vp ( view_point.x() - K.x(), view_point.y() - K.y(),   view_point.z() - K.z());

            normal.normalize();
            vp.normalize();
            // Dot product between the (viewpoint - point) and the plane normal
            float cos_theta = vp.dot (normal);
//            MLOGI("normal: [%f,%f,%f]", normal(0),normal(1),normal(2));
//            MLOGI("vp: [%f,%f,%f]", vp(0),vp(1),vp(2));
//            MLOGI("cos_theta: %f", cos_theta);

            // Flip the plane normal
            if (cos_theta < 0)
            {
                normal = -normal;
            }
            normal_x = normal(0);
            normal_y = normal(1);
            normal_z = normal(2);
            normal_angle = std:: acos(std::abs(cos_theta));
//            MLOGI("normal_angle: %f", normal_angle);

            return 0;

        }


    };




}
//#undef MLOGI
#endif //SCAN_REPUBLISHER_NORMAL_ESTIMATION_3D_H
