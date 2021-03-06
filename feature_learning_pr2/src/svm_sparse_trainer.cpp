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

typedef matrix<double,78,1> sample_type;

typedef one_vs_one_trainer<any_trainer<sample_type> > ovo_trainer;
typedef one_vs_all_trainer<any_trainer<sample_type> > ova_trainer;

// Next, we will make two different binary classification trainer objects.  One
// which uses kernel ridge regression and RBF kernels and another which uses a
// support vector machine and polynomial kernels.  The particular details don't matter.
// The point of this part of the example is that you can use any kind of trainer object
// with the one_vs_one_trainer.
typedef polynomial_kernel<sample_type> poly_kernel;
typedef radial_basis_kernel<sample_type> rbf_kernel;
typedef linear_kernel<sample_type> linear_kernl;
typedef histogram_intersection_kernel<sample_type> hist_kernel;


class cross_validation_objective
{
    /*!
        WHAT THIS OBJECT REPRESENTS
            This object is a simple function object that takes a set of model
            parameters and returns a number indicating how "good" they are.  It
            does this by performing 10 fold cross validation on our dataset
            and reporting the accuracy.

            See below in main() for how this object gets used.
    !*/
public:

    cross_validation_objective (
        const std::vector<sample_type>& samples_,
        const std::vector<double>& labels_
    ) : samples(samples_), labels(labels_) {}

    double operator() (
        const matrix<double>& params
    ) const
    {
        // Pull out the two SVM model parameters.  Note that, in this case,
        // I have setup the parameter search to operate in log scale so we have
        // to remember to call exp() to put the parameters back into a normal scale.
        const double gamma = exp(params(0));
        const double nu    = exp(params(1));

        // Make an SVM trainer and tell it what the parameters are supposed to be.
        svm_nu_trainer<rbf_kernel> trainer;
        trainer.set_kernel(rbf_kernel(gamma));
        trainer.set_nu(nu);

        // Finally, perform 10-fold cross validation and then print and return the results.
        matrix<double> result = cross_validate_trainer(trainer, samples, labels, 10);
        cout << "gamma: " << setw(11) << gamma << "  nu: " << setw(11) << nu <<  "  cross validation accuracy: " << result;

        // Here I'm just summing the accuracy on each class.  However, you could do something else.
        // For example, your application might require a 90% accuracy on class +1 and so you could
        // heavily penalize results that didn't obtain the desired accuracy.  Or similarly, you
        // might use the roc_c1_trainer() function to adjust the trainer output so that it always
        // obtained roughly a 90% accuracy on class +1.  In that case returning the sum of the two
        // class accuracies might be appropriate.
        return sum(result);
    }

    const std::vector<sample_type>& samples;
    const std::vector<double>& labels;

};


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

void process_stream(std::string filenames, std::string label_names, std::vector<Eigen::MatrixXf>& feature_list, std::vector<double>& labels,int &cols){


	std::ifstream input_stream(filenames.c_str());
	std::ifstream label_stream(label_names.c_str());
	std::string filename;

	while(input_stream.good()){

		std::getline(input_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

//		std::cout<<" Processing "<<filename<<"..."<<std::endl;

		Eigen::MatrixXf new_feature;
		load(filename, new_feature);
		cols = new_feature.cols();
		feature_list.push_back(new_feature);
	}

	while(label_stream.good()){

		std::getline(label_stream, filename);
		if(filename.empty() || filename.at(0) == '#') // skip blank lines or comments
			continue;

		std::fstream label_file(filename.c_str(), ios::in);
		double label;
		label_file >> label;
		label_file.close();

		if(label > 0)
		{
			if(filename.find("push_r")!=std::string::npos)
				label=2;
			if(filename.find("push_l")!=std::string::npos)
				label=2;
			if(filename.find("push_f")!=std::string::npos)
			    label=2;

		}
		else{
			label = 3;
		}

//		std::cout<<" Processing "<<filename<<"..."<<" Extracted label value: "<<label<<std::endl;

		labels.push_back(label);
	}

}

int main(int argc, char **argv)
{
	if(argc < 3)
	{
		std::cout<<" Please provide a filename sequence for features and filename for labels"<<std::endl;
		return -1;
	}

	std::vector<Eigen::MatrixXf> feature_list;
	std::vector<double> labels;
	int cols;

	process_stream(argv[1],argv[2],feature_list,labels,cols);
	std::cout<<" Finished reading "<<feature_list.size()<<" feature files and "<<labels.size()<<" label files"<<std::endl;
	std::cout<<" Number of cols per feature: "<<cols<<std::endl;



	std::vector<sample_type> samples;

	for(int i = 0 ; i < feature_list.size() ; i++)
	{
		sample_type new_datum;
		//std::cout<<" Saving datum : "<<i<<std::endl;
		for(int j = 0 ; j < cols-2 ; j++)
			{
			Eigen::MatrixXf current_sample = feature_list[i];
			new_datum(j) = static_cast<double>(current_sample(0,j));
			}
		samples.push_back(new_datum);
	}


	// Vector Normalizer
	vector_normalizer<sample_type> normalizer;
	// let the normalizer learn the mean and standard deviation of the samples
	normalizer.train(samples);

	// now normalize each sample
	for (unsigned long i = 0; i < samples.size(); ++i)
		samples[i] = normalizer(samples[i]);

	std::cout<<" Randomizing samples for training"<<std::endl;
	randomize_samples(samples, labels);

	// Now try loading the data in to the native data type
	std::cout << "Data loading complete"<<std::endl;


	// Finally, make the one_vs_one_trainer.
	ovo_trainer trainer;
	ova_trainer trainer_a;

	// make the binary trainers and set some parameters
	krr_trainer<rbf_kernel> rbf_trainer;
	svm_nu_trainer<poly_kernel> poly_trainer;
	svm_c_trainer<rbf_kernel> svm_rbf_trainer;
	svm_c_linear_trainer<linear_kernl> svm_linear_trainer;
	svm_nu_trainer<rbf_kernel> nu_rbf_trainer;


	std::cout<<" Setting kernel parameters"<<std::endl;
	poly_trainer.set_kernel(poly_kernel(0.1,1,2));
	rbf_trainer.set_kernel(rbf_kernel(0.01));
	svm_rbf_trainer.set_kernel(rbf_kernel(0.01));
	nu_rbf_trainer.set_kernel(rbf_kernel(0.01));

	std::cout<<"-------------------------"<<std::endl;
	std::cout<<"-------One vs One--------"<<std::endl;
	std::cout<<"-------------------------"<<std::endl;


    one_vs_one_decision_function<ovo_trainer,
            decision_function<poly_kernel>,  // This is the output of the poly_trainer
            decision_function<rbf_kernel>,
            decision_function<linear_kernl>,
            decision_function<rbf_kernel>// This is the output of the rbf_trainer
        > dfa;

	std::cout<<"setting svm c linear kernel Results:"<<std::endl;
	trainer.set_trainer(svm_linear_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df1 = trainer.train(samples, labels);
    dfa = df1;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;

	std::cout<<"setting svm_c rbf kernel Results:"<<std::endl;
	trainer.set_trainer(svm_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df2 = trainer.train(samples, labels);
    dfa = df2;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;


    std::cout<<"setting svm_nu_trainer poly kernel Results:"<<std::endl;
	trainer.set_trainer(poly_trainer); // Works best
    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df3 = trainer.train(samples, labels);
    dfa = df3;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;



	std::cout<<"setting krr rbf poly kernel Results:"<<std::endl;
	trainer.set_trainer(rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df4 = trainer.train(samples, labels);
    dfa = df4;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;



	std::cout<<"setting svm_nu_trainer rbf poly kernel Results:"<<std::endl;
	trainer.set_trainer(nu_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df5 = trainer.train(samples, labels);
    dfa = df5;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;

	std::cout<<"-------------------------"<<std::endl;
	std::cout<<"-------One vs all--------"<<std::endl;
	std::cout<<"-------------------------"<<std::endl;


    one_vs_all_decision_function<ova_trainer,
            decision_function<poly_kernel>,  // This is the output of the poly_trainer
            decision_function<rbf_kernel>,
            decision_function<linear_kernl>,
            decision_function<rbf_kernel>// This is the output of the rbf_trainer
        > dfb;

	std::cout<<"setting svm c linear kernel Results:"<<std::endl;
	trainer_a.set_trainer(svm_linear_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df6 = trainer_a.train(samples, labels);
    dfb = df6;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;

	std::cout<<"setting svm_c rbf kernel Results:"<<std::endl;
	trainer_a.set_trainer(svm_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df7 = trainer_a.train(samples, labels);
    dfb = df7;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;


    std::cout<<"setting svm_nu_trainer poly kernel Results:"<<std::endl;
    trainer_a.set_trainer(poly_trainer); // Works best
    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df8 = trainer_a.train(samples, labels);
    dfb = df8;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;



	std::cout<<"setting krr rbf poly kernel Results:"<<std::endl;
	trainer_a.set_trainer(rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df9 = trainer_a.train(samples, labels);
    dfb = df9;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;



	std::cout<<"setting svm_nu_trainer rbf poly kernel Results:"<<std::endl;
	trainer_a.set_trainer(nu_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df10 = trainer_a.train(samples, labels);
    dfb = df10;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;


    // Now check crossvalidate and check results.

}

