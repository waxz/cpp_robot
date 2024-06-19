//
// Created by waxz on 23-2-23.
//

#ifndef SCAN_REPUBLISHER_EIGEN_TRANSFORM_H
#define SCAN_REPUBLISHER_EIGEN_TRANSFORM_H

#define EIGEN_MAX_ALIGN_BYTES 32
#define EIGEN_MAX_STATIC_ALIGN_BYTES 32

//#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

//https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
// transformation should only use double

// if you use non-normalised float matrix ,  Eigen::Quaternion<FloatType> Q(transform.rotation()) , may block program

//typedef Transform<double,3,Isometry> Isometry3d;


namespace transform{
    template<class FloatType>
    using Transform = Eigen::Transform<FloatType,3,Eigen::Isometry>;


    //https://stackoverflow.com/questions/63341630/angle-axis-to-quaternion-using-eigen
    template<typename FloatType>
    Eigen::Quaternion<FloatType> createQuaternion(FloatType roll, FloatType pitch, FloatType yaw){
#if 0

        Eigen::Matrix<FloatType,3,1> rotation(roll, pitch, yaw);
        FloatType angle = rotation.norm();
        Eigen::Matrix<FloatType,3,1> axis = rotation.normalized();
        return Eigen::Quaternion<FloatType> (Eigen::AngleAxis<FloatType>(angle, axis));
#endif
        Eigen::Quaternion<FloatType> q;
#if 0
        q = Eigen::AngleAxis<FloatType>(roll, Eigen::Matrix<FloatType,3,1> ::UnitX())
            * Eigen::AngleAxis<FloatType>(pitch, Eigen::Matrix<FloatType,3,1> ::UnitY())
            * Eigen::AngleAxis<FloatType>(yaw, Eigen::Matrix<FloatType,3,1> ::UnitZ());

#endif
        q = Eigen::AngleAxis<FloatType>(yaw, Eigen::Matrix<FloatType,3,1> ::UnitZ())
            *
            Eigen::AngleAxis<FloatType>(pitch, Eigen::Matrix<FloatType,3,1> ::UnitY())
            *
            Eigen::AngleAxis<FloatType>(roll, Eigen::Matrix<FloatType,3,1> ::UnitX())

            ;



        return q;
    }
    template<typename FloatType>
    Eigen::Matrix<FloatType, 3, 1> ToEulerAngles(const Eigen::Quaternion<FloatType>& q) {
#if 0

        Eigen::Matrix<FloatType, 3, 1> angles;    //yaw pitch roll
        const auto x = q.x();
        const auto y = q.y();
        const auto z = q.z();
        const auto w = q.w();

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        angles[2] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        angles[0] = std::atan2(siny_cosp, cosy_cosp);
        return angles;
#endif


#if 0
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler;
#endif


#if 1
        Eigen::Matrix<FloatType, 3, 1> angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
#endif
    }


    template<typename FloatType>
    Eigen::Transform<FloatType,3,Eigen::Isometry> createSe3(FloatType tx, FloatType ty, FloatType tz, FloatType qw, FloatType qx, FloatType qy, FloatType qz){

        Eigen::Transform<FloatType,3,Eigen::Isometry> result = Eigen::Transform<FloatType,3,Eigen::Isometry>::Identity();;
//    Eigen::Quaternion<FloatType> Q = Eigen::Quaternion<FloatType>(qw, qx, qy, qz).normalized();
        Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);

        Q.normalize();
        Eigen::Matrix<FloatType,3,1> T(tx, ty, tz);

        result.rotate(Q.toRotationMatrix());
        result.pretranslate(T);
        return result;

    }

    template<typename FloatType>
    Eigen::Transform<FloatType,3,Eigen::Isometry> createSe3(FloatType tx, FloatType ty, FloatType tz, FloatType roll, FloatType pitch, FloatType yaw){

        Eigen::Transform<FloatType,3,Eigen::Isometry> result = Eigen::Transform<FloatType,3,Eigen::Isometry>::Identity();;
//    Eigen::Quaternion<FloatType> Q = Eigen::Quaternion<FloatType>(qw, qx, qy, qz).normalized();
        Eigen::Quaternion<FloatType> Q = createQuaternion(roll, pitch, yaw);
        Eigen::Matrix<FloatType,3,1> T(tx, ty, tz);

        result.rotate(Q.toRotationMatrix());
        result.pretranslate(T);
        return result;

    }


    template<typename FloatType>
    void extractSe3(const Eigen::Transform<FloatType,3,Eigen::Isometry>& transform, FloatType& tx, FloatType& ty, FloatType& tz, FloatType& qw, FloatType& qx, FloatType& qy, FloatType& qz){

        tx = transform.translation()(0);
        ty = transform.translation()(1);
        tz = transform.translation()(2);
        Eigen::Quaternion<FloatType> Q(transform.rotation());
        qw = Q.w();
        qx = Q.x();
        qy = Q.y();
        qz = Q.z();

    }
    template<typename FloatType>
    void extractSe3(const Eigen::Transform<FloatType,3,Eigen::Isometry>& transform, FloatType& tx, FloatType& ty, FloatType& tz, FloatType& roll, FloatType& pitch, FloatType& yaw){

        tx = transform.translation()(0);
        ty = transform.translation()(1);
        tz = transform.translation()(2);
        Eigen::Quaternion<FloatType> Q(transform.rotation());

        auto angle = ToEulerAngles(Q);
        roll = angle[2];
        pitch = angle[1];
        yaw = angle[0];
    }

//https://math.stackexchange.com/questions/90081/quaternion-distance
    template<typename FloatType>
    FloatType computeQuaternionDiff(const  Eigen::Quaternion<FloatType>& q1, const  Eigen::Quaternion<FloatType>& q2){
        FloatType qd1 = q1.x()*q2.x() + q1.y()*q2.y() + q1.z() * q2.z() + q1.w()*q2.w();
        FloatType diff = 2*acos(abs(qd1));
        return diff;
    }


    template<typename FloatType>
    Eigen::Quaternion<FloatType> createQuaternion( FloatType qw, FloatType qx, FloatType qy, FloatType qz){
        Eigen::Quaternion<FloatType> Q(qw, qx, qy, qz);
        Q.normalize();
        return Q;

    }
}

#endif //SCAN_REPUBLISHER_EIGEN_TRANSFORM_H
