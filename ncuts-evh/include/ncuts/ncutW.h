/*
 * ncutW: normalized cuts on a general graph
 *
 * translated from Shi et al's matlab implementation
 *
 * Evan Herbst
 * 10 / 13 / 11
 */

#ifndef EX_NCUT_W_H
#define EX_NCUT_W_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

/*
 * run normalized cuts on a similarity matrix W
 */
std::vector<unsigned int> ncutW(const Eigen::MatrixXd& W, const unsigned int nbcluster);
std::vector<unsigned int> ncutW(const Eigen::SparseMatrix<double>& W, const unsigned int nbcluster);

/*
 * compute the value of the normalized cuts objective for the given clustering (asgns[i] in [0, numClusters))
 */
double ncutObjective(const Eigen::MatrixXd& W, const unsigned int numClusters, const std::vector<unsigned int>& asgns);
double ncutObjective(const Eigen::SparseMatrix<double>& W, const unsigned int numClusters, const std::vector<unsigned int>& asgns);

#endif //EX_NCUT_W_H
