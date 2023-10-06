/*
 * mpc.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: andreyminin
 */

#include "../include/mpc.h"

#include <ros/ros.h>

USING_NAMESPACE_ACADO
namespace mpc_controller
{

MPC::MPC(int steps, double dt, double max_vel, double max_acc, double max_delta, double max_delta_rate, double L,
         double kcte, double kepsi, double kv, double ksteer_cost):
    t_end(steps * dt),
    steps(steps),
    max_vel(max_vel),
    max_acc(max_acc),
    max_delta(max_delta),
    max_delta_rate(max_delta_rate),
    L(L),
    kcte(kcte),
    kepsi(kepsi),
    kev(kv),
    ksteer_cost(ksteer_cost)
{
  // 系统微分方程
  // discrete time system
  f << dot(x) == vel*cos(fi);
  f << dot(y) == vel*sin(fi);
  f << dot(fi) == vel*tan(delta)/L;
  f << dot(delta) == delta_rate;
  f << dot(vel) == acc;
}

/**
 * @brief 求解 MPC 问题
 * 
 * @param v0 初始速度
 * @param delta0 初始转向角
 * @param traj_coef 轨迹参数，三阶多项式
 * @param rate_u 当前最优转向速率
 * @param acc_u 当前最优加速度
 * @param res_x 预估轨迹点
 * @param res_y 预估轨迹点
*/
void MPC::solve(double v0, double delta0, std::vector<double>& traj_coef, double& rate_u, double& acc_u,
                std::vector<double>& res_x, std::vector<double>& res_y)
{
  ROS_DEBUG_STREAM("solve mpc for v0 = "<<v0<<" angle0 = "<<delta0);
  // 断言  初始前轮转向角小于最大角度
  assert(std::abs(delta0) < max_delta);
  // 最优化控制问题类
  ACADO::OCP ocp(t_start, t_end, steps);
  // constrains
  ocp.subjectTo(f); // 系统方程
  ocp.subjectTo( AT_START, x == 0 ); // 初始位置
  ocp.subjectTo( AT_START, y == 0 ); // 初始位置
  ocp.subjectTo( AT_START, fi == 0 ); // 初始姿态
  ocp.subjectTo( -max_acc <= acc <= max_acc ); // 加速度限制
  ocp.subjectTo( -max_delta_rate <= delta_rate <= max_delta_rate ); // 前轮转向速率限制
  ocp.subjectTo( -max_delta <= delta <= max_delta ); // 前轮转角限制
  ocp.subjectTo( AT_START, vel == v0 ); // 初始速度
  ocp.subjectTo( AT_START, delta == delta0 ); // 初始前轮转角

  // 断言，轨迹参数为四
  assert(traj_coef.size() == 4);
  double& a0 = traj_coef[0];
  double& a1 = traj_coef[1];
  double& a2 = traj_coef[2];
  double& a3 = traj_coef[3];

  // 最小二乘项
  // minimization objectives
  Expression cte = pow(y - a0 - a1*x - a2*x*x -a3*x*x*x, 2); // cross track error 位置误差
  Expression epsi = pow(fi - atan(a1 + 2*a2*x + 3*a3*x*x), 2); // 角度误差
  Expression verr = pow(vel - max_vel, 2); // 速度损失  希望速度尽可能快
  Expression steer_cost = pow(delta_rate, 2); // 转向角损失  希望角速度尽可能小
//  double emax = std::max(0.25, a0*1.2);
//  ocp.subjectTo(cte <= emax);

  // 目标函数  梅耶项?  希望终端的误差尽量小？
  ocp.minimizeMayerTerm(kcte*cte + kepsi*epsi + kev*verr + ksteer_cost*steer_cost);
  
//  ocp.subjectTo(AT_END, cte <= 0.1);
//  ocp.subjectTo(AT_END, epsi <= 0.01);

  // 优化算法  什么方法？
  OptimizationAlgorithm alg(ocp);
  alg.set(PRINTLEVEL, NONE);
  ROS_INFO_STREAM("start solving mpc");
  alg.solve();
  ROS_INFO_STREAM("finished solving mpc");
  VariablesGrid controls;
  alg.getControls(controls); // 获取控制量     怎么确定哪些是控制量？ 顺序？
//  controls.print();
  VariablesGrid states;
  alg.getDifferentialStates(states); // 获取状态     同上?
  //states.print();

  rate_u = controls(0, 0); // 转向改变速率
  acc_u = controls(0, 1); // 加速度
  ROS_INFO_STREAM("delta = "<<rate_u<<" acc = "<<acc_u);

  // 计算预估状态的结果，作为估计轨迹，进行显示
  res_x.resize(states.getNumPoints());
  res_y.resize(res_x.size());
  for (std::size_t i = 0; i < res_x.size(); ++i) {
    res_x[i] = states(i, 0);
    res_y[i] = states(i, 1);
  }
}

MPC::~MPC()
{

}

} /* namespace mpc_controller */
