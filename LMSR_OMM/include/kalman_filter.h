#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

class KalmanFilter
{
private:
    /*x=Ax_+Bu+w
    z=Hx+v
    w~N(0,Q)
    v~N(0,R)*/
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;//真实值
    Eigen::VectorXd z;//测量值
    Eigen::MatrixXd A;//状态转移矩阵
    Eigen::MatrixXd B;//控制变量矩阵
    Eigen::VectorXd u;//状态控制向量
    Eigen::MatrixXd P;//后验估计的协方差矩阵coveriance
    Eigen::MatrixXd H;//状态到测量的转换矩阵

public:
    bool isKalmanInit=false;
    double last_filtered_heading;
    Eigen::MatrixXd R;//测量噪声协方差矩阵measurement noise covariance
    Eigen::MatrixXd Q;//系统噪声协方差矩阵process noise covariance
    KalmanFilter(int stateSize_, int measSize_,int uSize_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    void update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};
#endif