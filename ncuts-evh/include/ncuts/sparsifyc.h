#ifndef EX_NCUTS_SPARSIFYC_H
#define EX_NCUTS_SPARSIFYC_H

#include <Eigen/Core>
#include <Eigen/Sparse>

/*
 * return a matrix with zeros wherever mx(.) is <= thres
 */
Eigen::SparseMatrix<double> sparsify(const Eigen::MatrixXd& mx, const double thres);
Eigen::SparseMatrix<double> sparsify(const Eigen::SparseMatrix<double>& mx, const double thres);

#endif //header
