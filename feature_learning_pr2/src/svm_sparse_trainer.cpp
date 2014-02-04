/*********************************************************************
*
*  Copyright (c) 2014, Computational Learning and Motor Control Laboratory
*  University of Southern California
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************
/**
 * \author Bharath Sankaran
 *
 * @b SVM trainer. Trains SVMs from file.
 *
 */
#include <iostream>
#include <dlib/svm.h>
#include <Eigen/Dense>

using namespace std;
using namespace dlib;

void load(std::string filename, Eigen::MatrixXf& m)
{

	ifstream input_file(filename.c_str());
	std::vector<float> input_vector;
	std::copy(std::istream_iterator<float>(input_file), // this denotes "start of stream"
			std::istream_iterator<float>(),   // this denodes "end of stream"
			std::back_inserter< std::vector< float > >(input_vector));
	Eigen::Map<Eigen::MatrixXf> value(input_vector.data(),1,input_vector.size());
	m = value;
	// Test to check if data is being loaded correctly
}

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		std::cout<<" Please provide a filename sequence for features and filename for labels"<<std::endl;
		return -1;
	}
	std::string filename;
	std::ifstream input_stream(argv[1]);
	std::string label_filename(argv[2]);

	std::vector<Eigen::MatrixXf> feature_list;
	int cols;
	while(input_stream.good()){

		std::getline(input_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

		std::cout<<" Processing "<<filename<<std::endl;

		Eigen::MatrixXf new_feature;
		load(filename, new_feature);
		cols = new_feature.cols();
		feature_list.push_back(new_feature);
	}


	std::cout<<" Finished reading feature files, Now reading labels from "<<label_filename<<std::endl;
	// Now read the labels

	std::ifstream label_file(label_filename.c_str());
	std::vector<double> labels;
	std::copy(std::istream_iterator<double>(label_file), // this denotes "start of stream"
			std::istream_iterator<double>(),   // this denodes "end of stream"
			std::back_inserter< std::vector< double > >(labels));

	std::cout<<" Finished reading "<<labels.size()<<" label files"<<std::endl;


	typedef matrix<double,80,1> sample_type;
	std::vector<sample_type> samples;

	for(int i = 0 ; i < feature_list.size() ; i++)
	{
		sample_type new_datum;
		std::cout<<" Saving datum : "<<i<<std::endl;
		for(int j = 0 ; j < cols ; j++)
			{
			Eigen::MatrixXf current_sample = feature_list[i];
			new_datum(j) = static_cast<double>(current_sample(0,j));
			}
		samples.push_back(new_datum);
	}

	// Now try loading the data in to the native data type
	typedef one_vs_one_trainer<any_trainer<sample_type> > ovo_trainer;

	// Finally, make the one_vs_one_trainer.
	ovo_trainer trainer;


	// Next, we will make two different binary classification trainer objects.  One
	// which uses kernel ridge regression and RBF kernels and another which uses a
	// support vector machine and polynomial kernels.  The particular details don't matter.
	// The point of this part of the example is that you can use any kind of trainer object
	// with the one_vs_one_trainer.
	typedef polynomial_kernel<sample_type> poly_kernel;
	typedef radial_basis_kernel<sample_type> rbf_kernel;

	// make the binary trainers and set some parameters
	krr_trainer<rbf_kernel> rbf_trainer;
	svm_nu_trainer<poly_kernel> poly_trainer;
	poly_trainer.set_kernel(poly_kernel(0.1, 1, 2));
	rbf_trainer.set_kernel(rbf_kernel(0.1));

    // Now lets do 5-fold cross-validation using the one_vs_one_trainer we just setup.
    // As an aside, always shuffle the order of the samples before doing cross validation.
    // For a discussion of why this is a good idea see the svm_ex.cpp example.
    randomize_samples(samples, labels);
    cout << "cross validation: \n" << cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;

    one_vs_one_decision_function<ovo_trainer> df = trainer.train(samples, labels);

    one_vs_one_decision_function<ovo_trainer,
            decision_function<poly_kernel>,  // This is the output of the poly_trainer
            decision_function<rbf_kernel>    // This is the output of the rbf_trainer
        > df2, df3;


    // Put df into df2 and then save df2 to disk.  Note that we could have also said
    // df2 = trainer.train(samples, labels);  But doing it this way avoids retraining.
    df2 = df;
    ofstream fout("df.dat", ios::binary);
    serialize(df2, fout);
    fout.close();
	// Now setup a kernel

    // Test df3 to see that this worked.
    cout << endl;
    cout << "predicted label: "<< df2(samples[0])  << ", true label: "<< labels[0] << endl;
    cout << "predicted label: "<< df2(samples[90]) << ", true label: "<< labels[90] << endl;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(df2, samples, labels) << endl;

}

