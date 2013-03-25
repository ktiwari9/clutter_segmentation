/*
 * matrixUtils: stuff Eigen ought to have
 *
 * Evan Herbst
 * 10 / 20 / 11
 */

#include "ncuts/matrixUtils.h"
using namespace Eigen;

/*
 * create a diagonal SparseMatrix
 */
SparseMatrix<double> sparseFromDiag(const VectorXd& d)
{
	SparseMatrix<double> result(d.rows(), d.rows());
	for(unsigned int j = 0; j < d.rows(); j++)
	{
		result.startVec(j);
		result.insertBack(j, j) = d[j];
	}
	result.finalize();
	return result;
}

/*
 * m.array().abs().rowwise().sum().matrix()
 */
VectorXd rowwiseAbsSums(const SparseMatrix<double>& m)
{
	VectorXd result(m.rows());
	result.fill(0);
	for(unsigned int j = 0; j < m.cols(); j++)
		for(SparseMatrix<double>::InnerIterator it(m, j); it; ++it)
			result[it.row()] += fabs(it.value());
	return result;
}

/*
 * m.rowwise().sum()
 */
VectorXd rowwiseSums(const SparseMatrix<double>& m)
{
	VectorXd result(m.rows());
	result.fill(0);
	for(unsigned int j = 0; j < m.cols(); j++)
		for(SparseMatrix<double>::InnerIterator it(m, j); it; ++it)
			result[it.row()] += it.value();
	return result;
}

bool isApproxSymmetric(const SparseMatrix<double>& m, const double eps)
{
	for(unsigned int j = 0; j < m.cols(); j++)
		for(SparseMatrix<double>::InnerIterator it(m, j); it; ++it)
			for(SparseMatrix<double>::InnerIterator it2(m, it.row()); it2; ++it2)
				if(it2.row() == j)
					if(fabs(it.value() - it2.value()) > eps)
						return false;
	return true;
}
