#ifndef EX_NCUTS_DISCRETISATION_H
#define EX_NCUTS_DISCRETISATION_H

#include <vector>
#include <iostream>
#include <Eigen/Core>

/*
 * Input: EigenVectors = continuous Ncut vector, size = ndata x nbEigenvectors
 * Output EigenvectorsDiscrete = discrete Ncut vector, size = ndata x nbEigenvectors
 */
std::vector<unsigned int> discretisation(Eigen::MatrixXd& EigenVectors);

#endif //header
