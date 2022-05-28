#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::minimumJerkTrajGen(    
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time){

    int pieceNum = Time.size();
    cout << "targetpos:" << Path.row(Path.rows()-1) << endl;
    Eigen::MatrixXd PolyCoeff(pieceNum, 3 * 6);
    Eigen::MatrixXd coefficientMatrix;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(pieceNum * 6, pieceNum * 6);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(pieceNum * 6, 3);
    Eigen::MatrixXd F_0(3, 6);
    Eigen::MatrixXd E_m(3, 6);
    Eigen::MatrixXd E_i(6, 6);
    Eigen::MatrixXd F_i(6, 6);
    Eigen::MatrixXd D_0(3, 3);
    Eigen::MatrixXd D_m(3, 3);
    Eigen::MatrixXd D_i(1, 3);
    auto get_Em = [&](const double& T){
        Eigen::MatrixXd res(3, 6);
        res <<  1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
                0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
                0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T,3);
        return res; 
    };
    E_m = get_Em(Time(pieceNum-1));
    // std::cout << E_m <<std::endl;
    F_0 << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0;
    D_0.row(0) = Path.row(0);     
    D_0.row(1) = Vel.row(0);
    D_0.row(2) = Acc.row(0);
    D_m.row(0) = Path.row(pieceNum);
    D_m.row(1) = Vel.row(1);
    D_m.row(2) = Acc.row(1);
    // std::cout << "D_0:" << D_0 << std::endl;
    // std::cout << "D_m: " << D_m << std::endl;
    auto get_Ei = [&] (const double& T){
        Eigen::MatrixXd res(6,6);
        res <<  1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
                1, T, pow(T,2), pow(T,3), pow(T,4), pow(T,5),
                0, 1, 2*T, 3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
                0, 0, 2, 6*T, 12*pow(T,2), 20*pow(T,3),
                0, 0, 0, 6, 24*T, 60*pow(T, 2),
                0, 0, 0, 0, 24, 120*T; 
        return res;
    };
    F_i << 0, 0, 0, 0, 0, 0,
          -1, 0, 0, 0, 0, 0,
          0, -1, 0, 0, 0, 0,
          0, 0, -2, 0, 0, 0,
          0, 0, 0, -6, 0, 0,
          0, 0, 0, 0, -24, 0; 
    M.block(0,0,3,6) = F_0;
    M.block(6*pieceNum-3, 6*pieceNum-6, 3, 6) = E_m;
    b.block(0, 0, 3, 3) = D_0;
    b.block(6*pieceNum-3,0, 3, 3) = D_m;

    for(int i = 0;i < pieceNum-1; i++){
        E_i = get_Ei(Time(i));
        M.block(6*i +3, 6*i, 6, 6) = E_i;
        M.block(6*i +3, 6*i +6, 6, 6) = F_i;
        D_i = Path.row(i + 1);
        b.block(6*i+3,0, 1,3) = D_i;
        // std::cout << "D_i: " << D_i<< std::endl;
    }
    std::cout << "b: " << b<< std::endl;
    std::cout << "M:" << M <<std::endl;
    coefficientMatrix = M.colPivHouseholderQr().solve(b);
    std::cout << "coefficientMatrix:" << coefficientMatrix <<std::endl;
    for (int i = 0; i < pieceNum; i++)
    {
      PolyCoeff.block(i, 0, 1, 6) = coefficientMatrix.col(0).segment(i * 6, 6).transpose();
      PolyCoeff.block(i, 6, 1, 6) = coefficientMatrix.col(1).segment(i * 6, 6).transpose();
      PolyCoeff.block(i, 12, 1, 6) = coefficientMatrix.col(2).segment(i * 6, 6).transpose();
    }
    return PolyCoeff;
}
Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);
  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m); // 每个轴的多项式系数
  int num_f, num_p;
  int num_d;
  const static auto factorial = [](int x){
    int fac = 1;
    for(int i = x; i > 0; i--){
      fac = fac * i;
    }
    return fac;
  };
   /* ---------- end point derivative ---------- */
  VectorXd Dx = VectorXd::Zero(m * 6);
  VectorXd Dy = VectorXd::Zero(m * 6);
  VectorXd Dz = VectorXd::Zero(m * 6);
  for (int k = 0; k < m; k++)
  {
    /* position to derivative */
    Dx(k * 6) = Path(k, 0);
    Dx(k * 6 + 1) = Path(k + 1, 0);
    Dy(k * 6) = Path(k, 1);
    Dy(k * 6 + 1) = Path(k + 1, 1);
    Dz(k * 6) = Path(k, 2);
    Dz(k * 6 + 1) = Path(k + 1, 2);
    // 只固定多段轨迹的起点和终点的速度、加速度
    if (k == 0)
    {
      Dx(k * 6 + 2) = Vel(0, 0);
      Dy(k * 6 + 2) = Vel(0, 1);
      Dz(k * 6 + 2) = Vel(0, 2);

      Dx(k * 6 + 4) = Acc(0, 0);
      Dy(k * 6 + 4) = Acc(0, 1);
      Dz(k * 6 + 4) = Acc(0, 2);
    }
    else if (k == m - 1)
    {
      Dx(k * 6 + 3) = Vel(1, 0);
      Dy(k * 6 + 3) = Vel(1, 1);
      Dz(k * 6 + 3) = Vel(1, 2);

      Dx(k * 6 + 5) = Acc(1, 0);
      Dy(k * 6 + 5) = Acc(1, 1);
      Dz(k * 6 + 5) = Acc(1, 2);
    }
  }
  MatrixXd Ab;
  MatrixXd A = Eigen::MatrixXd::Zero(m * 6, m * 6);
  for (int k = 0; k < m; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = factorial(j) / factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }
  /* ---------- Produce Selection Matrix C' ---------- */
  Eigen::MatrixXd Ct, C;
  // 固定变量=轨迹点起点和终点的pva的3+3 + 段之间连接的点的起点和终点p
  num_f = 2 * m + 4; // 3 + 3 + (m - 1) * 2 = 2m + 4
    // 剩余的段之间连接的点速度和加速度
  num_p = 2 * m - 2; //(m - 1) * 2 = 2m - 2
  num_d = 6 * m;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
    // 计算置换矩阵c 起点的pva放到置1，连接点的p起点和终点
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * m + 4) = 1;
  Ct(5, 2 * m + 5) = 1;

  Ct(6 * (m - 1) + 0, 2 * m + 0) = 1;
  Ct(6 * (m - 1) + 1, 2 * m + 1) = 1; // Stack the end point
  Ct(6 * (m - 1) + 2, 4 * m + 0) = 1;
  Ct(6 * (m - 1) + 3, 2 * m + 2) = 1; // Stack the end point
  Ct(6 * (m - 1) + 4, 4 * m + 1) = 1;
  Ct(6 * (m - 1) + 5, 2 * m + 3) = 1; // Stack the end point
  for (int j = 2; j < m; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * m + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * m + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * m + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * m + 4 + 2 * (j - 1) + 1) = 1;
  }
  C = Ct.transpose();
  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;
  /* ---------- minimum snap matrix ---------- */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(m * 6, m * 6);
  for (int k = 0; k < m; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }
  /* ---------- R matrix ---------- */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;
  // 拆分四块
  Eigen::VectorXd Dxf(2 * m + 4), Dyf(2 * m + 4), Dzf(2 * m + 4);

  Dxf = Dx1.segment(0, 2 * m + 4);
  Dyf = Dy1.segment(0, 2 * m + 4);
  Dzf = Dz1.segment(0, 2 * m + 4);

  Eigen::MatrixXd Rff(2 * m + 4, 2 * m + 4);
  Eigen::MatrixXd Rfp(2 * m + 4, 2 * m - 2);
  Eigen::MatrixXd Rpf(2 * m - 2, 2 * m + 4);
  Eigen::MatrixXd Rpp(2 * m - 2, 2 * m - 2);

  Rff = R.block(0, 0, 2 * m + 4, 2 * m + 4);
  Rfp = R.block(0, 2 * m + 4, 2 * m + 4, 2 * m - 2);
  Rpf = R.block(2 * m + 4, 0, 2 * m - 2, 2 * m + 4);
  Rpp = R.block(2 * m + 4, 2 * m + 4, 2 * m - 2, 2 * m - 2);

  /* ---------- close form solution ---------- */
  // 按照公式求解dp
  Eigen::VectorXd Dxp(2 * m - 2), Dyp(2 * m - 2), Dzp(2 * m - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  Dx1.segment(2 * m + 4, 2 * m - 2) = Dxp;
  Dy1.segment(2 * m + 4, 2 * m - 2) = Dyp;
  Dz1.segment(2 * m + 4, 2 * m - 2) = Dzp;
  // 得到p
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;
  // 无约束的p求解得到
  for (int i = 0; i < m; i++)
  {
    PolyCoeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    PolyCoeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    PolyCoeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }
  // /* ---------- use polynomials ---------- */

  // PolynomialTraj poly_traj;
  // //  将每一段的轨迹系数参数保存下来
  // for (int i = 0; i < poly_coeff.rows(); ++i)
  // {
  //   vector<double> cx(6), cy(6), cz(6);
  //   for (int j = 0; j < 6; ++j)
  //   {
  //     cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
  //   }
  //   reverse(cx.begin(), cx.end());
  //   reverse(cy.begin(), cy.end());
  //   reverse(cz.begin(), cz.end());
  //   double ts = Time(i);
  //   poly_traj.addSegment(cx, cy, cz, ts);
  // }

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}