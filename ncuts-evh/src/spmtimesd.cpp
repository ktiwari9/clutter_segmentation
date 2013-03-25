#include <cassert>
#include "ncuts/matrixUtils.h"
#include "ncuts/spmtimesd.h"
using namespace Eigen;

/*
* spmtimesd.c
* This routine computes a sparse matrix times a diagonal matrix
* whose diagonal entries are stored in a full vector.
*
* Examples:
*     spmtimesd(m,d,[]) = diag(d) * m,
*     spmtimesd(m,[],d) = m * diag(d)
*     spmtimesd(m,d1,d2) = diag(d1) * m * diag(d2)
*     m could be complex, but d is assumed real
*/
MatrixXd spmtimesd(const MatrixXd& m, const VectorXd& d1, const VectorXd& d2)
{
	assert(m.rows() == d1.rows() || d1.rows() == 0);
	assert(m.cols() == d2.rows() || d2.rows() == 0);

	MatrixXd result(m.rows(), m.cols());
	result = m;
	if(d1.rows() > 0)
		result = DiagonalMatrix<double, Dynamic, Dynamic>(d1) * result;
	if(d2.rows() > 0)
		result = result * DiagonalMatrix<double, Dynamic, Dynamic>(d2);
	return result;
}
SparseMatrix<double> spmtimesd(const SparseMatrix<double>& m, const VectorXd& d1, const VectorXd& d2)
{
	assert(m.rows() == d1.rows() || d1.rows() == 0);
	assert(m.cols() == d2.rows() || d2.rows() == 0);

	SparseMatrix<double> result(m.rows(), m.cols());
	result = m;
	if(d1.rows() > 0)
		result = sparseFromDiag(d1) * result;
	if(d2.rows() > 0)
		result = result * sparseFromDiag(d2);

	return result;
}
