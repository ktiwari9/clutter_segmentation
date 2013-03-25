#ifndef EX_NCUTS_SPMTIMESD_H
#define EX_NCUTS_SPMTIMESD_H

#include <Eigen/Core>
#include <Eigen/Sparse>

/*
 * compute d1^T * m * d2, where m is intended to be sparse
 */
Eigen::MatrixXd spmtimesd(const Eigen::MatrixXd& m, const Eigen::VectorXd& d1, const Eigen::VectorXd& d2);
Eigen::SparseMatrix<double> spmtimesd(const Eigen::SparseMatrix<double>& m, const Eigen::VectorXd& d1, const Eigen::VectorXd& d2);

#endif //header
