/*
 * discretisation
 */

#include <iostream>
#include <iterator>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "ncuts/discretisation.h"
using std::vector;
using std::cout;
using std::endl;
using namespace Eigen;

/*
 * replace each row with a row that has a one in the largest-valued spot and zeros everywhere else
 */
MatrixXd discretisationEigenVectorData(const MatrixXd& EigenVector)
{
	const unsigned int n = EigenVector.rows(), k = EigenVector.cols();
	vector<unsigned int> rowMaxIndices(n);
	for(unsigned int i = 0; i < n; i++) EigenVector.row(i).maxCoeff(&rowMaxIndices[i]);
	MatrixXd Y = MatrixXd::Zero(n, k); //TODO was sparse in orig code
	for(unsigned int i = 0; i < n; i++) Y(i, rowMaxIndices[i]) = 1;
	return Y;
}

/*
 * Input: EigenVectors = continuous Ncut vector, size = ndata x nbEigenvectors
 * Output EigenvectorsDiscrete = discrete Ncut vector, size = ndata x nbEigenvectors
 */
vector<unsigned int> discretisation(MatrixXd& EigenVectors)
{
	const unsigned int n = EigenVectors.rows(), k = EigenVectors.cols();

	const VectorXd vm = EigenVectors.array().square().rowwise().sum().sqrt().matrix();
	for(unsigned int i = 0; i < EigenVectors.rows(); i++) EigenVectors.row(i) /= vm[i];

	/*
	 * R is a rotation for the eigenvectors
	 */
	MatrixXd R = MatrixXd::Zero(k, k);
	R.col(0) = EigenVectors.row(0/*rand() % n TODO*/).transpose();
	VectorXd c = VectorXd::Zero(n, 1);
	for(unsigned int j = 1; j < k; j++)
	{
		 c += (EigenVectors * R.col(j - 1)).array().abs().matrix();
		 const unsigned int i = std::distance(c.data(), std::min_element(c.data(), c.data() + c.size())); //min index of c
		 R.col(j) = EigenVectors.row(i).transpose();
	}

	double lastObjectiveValue = 0;
	bool exitLoop = 0;
	unsigned int nbIterationsDiscretisation = 0;
	const unsigned int nbIterationsDiscretisationMax = 20;
	MatrixXd EigenvectorsDiscrete;
	while(exitLoop == 0)
	{
		 nbIterationsDiscretisation++;
		 EigenvectorsDiscrete = discretisationEigenVectorData(EigenVectors * R);
		 const MatrixXd evdev = EigenvectorsDiscrete.transpose() * EigenVectors;
		 /*
		  * TODO the biggest difference between this implementation of ncuts and Shi's matlab one is that the signs of the singular vectors from the svd
		  * differ; this affects the rotations applied to the eigenvectors and, ultimately, the ncut objective value; not sure how often this makes
		  * our results worse; see e.g. the results vs. hard-coded outputs from Shi's code in testNcutW.cpp
		  */
		 JacobiSVD<MatrixXd> svd;
		 svd.compute(evdev, ComputeThinU | ComputeThinV);
		 MatrixXd U = svd.matrixU();
		 VectorXd S = svd.singularValues();
		 MatrixXd V = svd.matrixV();

		 //TODO set very small elements to 0?

		 /*
		  * flip signs of singular vecs to try to look more like the matlab results
		  * (from http://www.models.life.ku.dk/signflipsvd)
		  *
		  * 20111014: doing this does improve (ie decrease) the ncut value some for at least a few test cases; TODO haven't done a large test
		  */
		 for(unsigned int i = 0; i < U.cols(); i++)
		 {
			 const VectorXd dots = evdev * U.col(i);
			 unsigned int numPos = 0;
			 for(unsigned int j = 0; j < dots.size(); j++)
				 if(dots[j] >= 0)
					 numPos++;
			 if(numPos < dots.size() / 2) U.col(i) *= -1;
		 }
		 for(unsigned int i = 0; i < V.cols(); i++)
		 {
			 const VectorXd dots = evdev * V.col(i);
			 unsigned int numPos = 0;
			 for(unsigned int j = 0; j < dots.size(); j++)
				 if(dots[j] >= 0)
					 numPos++;
			 if(numPos < dots.size() / 2) V.col(i) *= -1;
		 }

		 const double NcutValue = 2 * (n - S.sum());
		 static const double eps = 1e-7;
		 if(fabs(NcutValue - lastObjectiveValue) < eps || nbIterationsDiscretisation > nbIterationsDiscretisationMax)
			  exitLoop=1;
		 else
		 {
			  lastObjectiveValue = NcutValue;
			  R = V * U.transpose();
		 }
	}

	MatrixXd q = EigenVectors * R;
	MatrixXd corrs(EigenVectors.rows(), EigenVectors.cols());
	for(unsigned int i = 0; i < EigenVectors.rows(); i++)
		for(unsigned int j = 0; j < EigenVectors.cols(); j++)
			corrs(i, j) = EigenVectors.row(i).dot(q.row(j));

	vector<unsigned int> assignments(EigenvectorsDiscrete.rows(), 0);
	for(unsigned int i = 0; i < EigenvectorsDiscrete.rows(); i++)
		for(unsigned int j = 0; j < EigenvectorsDiscrete.cols(); j++)
			if(EigenvectorsDiscrete(i, j) > .1) //the value will be 0 or 1, so any threshold in between is ok
			{
				assignments[i] = j;
				break; //go to next row
			}
	return assignments;
}
