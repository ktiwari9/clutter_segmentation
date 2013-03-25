#include "ncuts/sparsifyc.h"
using namespace Eigen;

/*
 * return a matrix with zeros wherever mx(.) is <= thres
 */
SparseMatrix<double> sparsify(const MatrixXd& mx, const double thres)
{
	SparseMatrix<double> spmx(mx.rows(), mx.cols());
	for(unsigned int j = 0; j < mx.cols(); j++)
	{
		spmx.startVec(j);
		for(unsigned int i = 0; i < mx.rows(); i++)
			if(fabs(mx(i, j)) > thres)
			{
				spmx.insertBack(i, j) = mx(i, j);
			}
	}
	spmx.finalize();
	return spmx;
}
SparseMatrix<double> sparsify(const SparseMatrix<double>& mx, const double thres)
{
	SparseMatrix<double> spmx(mx.rows(), mx.cols());
	for(unsigned int j = 0; j < mx.cols(); j++)
	{
		spmx.startVec(j);
		for(SparseMatrix<double>::InnerIterator it(mx, j); it; ++it)
			if(fabs(it.value()) > thres)
			{
				spmx.insertBack(it.row(), j) = it.value();
			}
	}
	spmx.finalize();
	return spmx;
}
