/****************************************************************************
 * p2o: Petite Portable Pose-graph Optimizer
 * Copyright (C) 2010-2017 Kiyoshi Irie
 * Copyright (C) 2017 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 ****************************************************************************/
#ifndef P2O_H_
#define P2O_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using p2o_float_t = double;

namespace p2o
{
// === 2D geometry utility classes and functions ===

using Vec3D = Eigen::Matrix<p2o_float_t,3,1>;
using Mat3D = Eigen::Matrix<p2o_float_t,3,3>;

static inline p2o_float_t normalize_rad_pi_mpi(double rad);
static inline Mat3D rotMat2D(p2o_float_t th);

// === 2D pose-graph optimizer ===
static inline double robust_coeff(p2o_float_t squared_error, p2o_float_t delta);

struct Pose2D
{
    p2o_float_t x, y, th;
    Pose2D();
    Pose2D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_t);
    Vec3D vec() const;
    Pose2D oplus(const Pose2D r) const;
    Pose2D ominus(const Pose2D r) const;
};

using Pose2DVec = std::vector<Pose2D>;

struct Con2D
{
    int id1, id2;
    Pose2D t;
    Mat3D info;
    void setCovarianceMatrix( const Mat3D &mat );

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Con2DVec = std::vector<Con2D,Eigen::aligned_allocator<Con2D> >;

struct ErrorFunc2D
{
    static Vec3D error_func(const Pose2D &pa, const Pose2D &pb, const Pose2D &con);
    static Vec3D calcError(const Pose2D &pa, const Pose2D &pb, const Pose2D &con, Mat3D &Ja, Mat3D &Jb);
};

class Optimizer2D
{
    bool verbose;
    double lambda;
    double stop_thre;
    double robust_delta;
    typedef Eigen::Triplet<p2o_float_t> p2o_triplet;
    std::vector<p2o_triplet> tripletList;
public:
    Optimizer2D();
    ~Optimizer2D();
    void setLambda(double val);
    void setStopEpsilon(double val);
    void setRobustThreshold(double val);
    void setVerbose(bool val);
    Pose2DVec optimizePath(const Pose2DVec &in_graphnodes, const Con2DVec &constraints, int max_steps, int min_steps = 3);
    double optimizePathOneStep(Pose2DVec &out_nodes, const Pose2DVec &graphnodes, const Con2DVec &constraints, double prev_res);
    double globalCost(const Pose2DVec &poses, const Con2DVec &constraints);
    bool loadFile(const char *g2ofile, Pose2DVec &nodes, Con2DVec &constraints);
};

// === 3D utility classes and functions ===

using Vec6D = Eigen::Matrix<p2o_float_t,6,1>;
using Mat6D = Eigen::Matrix<p2o_float_t,6,6>;

struct RotVec
{
    p2o_float_t ax, ay, az;
    RotVec();
    RotVec(p2o_float_t x, p2o_float_t y, p2o_float_t z);
    RotVec(const Eigen::Quaterniond &in_q);
    p2o_float_t norm() const;
    RotVec inverted() const;
    Eigen::Quaterniond toQuaternion() const;
    Mat3D toRotationMatrix() const;
};

inline std::ostream& operator<<(std::ostream& os, const RotVec &p)
{
    os << p.ax << ' ' << p.ay << ' ' << p.az << ' ';
    return os;
}

static inline Eigen::Matrix<p2o_float_t, 4, 3> dQuat_dRV(const RotVec &rv);

static inline void dR_dRV(const RotVec &rv, Mat3D &dux, Mat3D &duy, Mat3D &duz);

static inline Eigen::Matrix<p2o_float_t, 3, 4> dRV_dQuat(const Eigen::Quaterniond &q);

static inline Eigen::Matrix<p2o_float_t,4,4> QMat(Eigen::Quaterniond &q);

static inline Eigen::Matrix<p2o_float_t,4,4> QMatBar(Eigen::Quaterniond &q);

// === 3D pose-graph optimizer ===

struct Pose3D
{
    p2o_float_t x, y, z;
    RotVec rv;
    Pose3D();
    Pose3D(p2o_float_t in_x, p2o_float_t in_y, p2o_float_t in_z, const RotVec &in_rv);
    Pose3D(p2o_float_t in_x,  p2o_float_t in_y,  p2o_float_t in_z,
           p2o_float_t ax, p2o_float_t ay, p2o_float_t az);

    Vec6D vec() const;
    Vec3D pos() const;

    Pose3D oplus(const Pose3D &rel) const;
    Pose3D ominus(const Pose3D &base) const;
};

using Pose3DVec = std::vector<Pose3D>;

struct Con3D
{
    int id1, id2;
    Pose3D t;
    Mat6D info;
    void setCovarianceMatrix( const Mat6D &mat )
    {
        info = mat.inverse();
    }
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using Con3DVec = std::vector<Con3D,Eigen::aligned_allocator<Con3D> >;

struct ErrorFunc3D
{
    static Vec6D error_func(const Pose3D &pa, const Pose3D &pb, const Pose3D &con);
    static Vec6D calcError(const Pose3D &pa, const Pose3D &pb, const Pose3D &con, Mat6D &Ja, Mat6D &Jb);
};

class Optimizer3D
{
    bool verbose;
    double lambda;
    double stop_thre;
    double robust_delta;
    typedef Eigen::Triplet<p2o_float_t> p2o_triplet;
    std::vector<p2o_triplet> tripletList;
public:
    Optimizer3D();
    ~Optimizer3D();
    void setLambda(double val);
    void setStopEpsilon(double val);
    void setRobustThreshold(double val);
    void setVerbose(bool val);
    Pose3DVec optimizePath(const Pose3DVec &in_graphnodes, const Con3DVec &constraints, int max_steps, int min_steps = 3);
    double optimizePathOneStep(Pose3DVec &out_nodes, const Pose3DVec &graphnodes, const Con3DVec &constraints, double prev_res);
    double globalCost(const Pose3DVec &poses, const Con3DVec &constraints);
    bool loadFile(const char *g2ofile, Pose3DVec &nodes, Con3DVec &constraints);
};


} //namespace p2o

#endif /* P2O_H_ */
