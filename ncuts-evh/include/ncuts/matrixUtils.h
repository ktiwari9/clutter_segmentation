/*
 * matrixUtils: stuff Eigen ought to have
 *
 * Evan Herbst
 * 10 / 20 / 11
 */

#ifndef EX_EIGEN_UTILS_H
#define EX_EIGEN_UTILS_H

#include <Eigen/Sparse>

/*
 * create a diagonal SparseMatrix
 */
Eigen::SparseMatrix<double> sparseFromDiag(const Eigen::VectorXd& d);

/*
 * m.array().abs().rowwise().sum().matrix()
 */
Eigen::VectorXd rowwiseAbsSums(const Eigen::SparseMatrix<double>& m);
/*
 * m.rowwise().sum()
 */
Eigen::VectorXd rowwiseSums(const Eigen::SparseMatrix<double>& m);

bool isApproxSymmetric(const Eigen::SparseMatrix<double>& m, const double eps);

#endif //header
