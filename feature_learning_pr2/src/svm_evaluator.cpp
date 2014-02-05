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
#include "feature_learning/macros_time.hpp"

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
				label=3;
			if(filename.find("push_f")!=std::string::npos)
			    label=4;

		}
		else{
			label = 5;
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
	svm_nu_trainer<poly_kernel> poly_trainer;
	svm_nu_trainer<rbf_kernel> nu_rbf_trainer;


	std::cout<<" Setting kernel parameters"<<std::endl;
	poly_trainer.set_kernel(poly_kernel(0.1,1,2));
	nu_rbf_trainer.set_kernel(rbf_kernel(0.01));

	std::cout<<"-------------------------"<<std::endl;
	std::cout<<"-------One vs One--------"<<std::endl;
	std::cout<<"-------------------------"<<std::endl;


    one_vs_one_decision_function<ovo_trainer,
            decision_function<poly_kernel>,  // This is the output of the poly_trainer
            decision_function<rbf_kernel>// This is the output of the rbf_trainer
        > dfa;


	INIT_PROFILING

    std::cout<<"setting svm_nu_trainer poly kernel Results:"<<std::endl;
	trainer.set_trainer(poly_trainer); // Works best
    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df3 = trainer.train(samples, labels);
    dfa = df3;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;

    MEASURE("SVM NU POLY")

    RESET

/*    std::cout<<"setting svm_nu_trainer rbf kernel Results:"<<std::endl;
	trainer.set_trainer(nu_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer, samples, labels, 5) << endl;
    one_vs_one_decision_function<ovo_trainer> df5 = trainer.train(samples, labels);
    dfa = df5;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfa, samples, labels) << endl;

    MEASURE("SVM NU RBF")*/

	std::cout<<"-------------------------"<<std::endl;
	std::cout<<"-------One vs all--------"<<std::endl;
	std::cout<<"-------------------------"<<std::endl;


    one_vs_all_decision_function<ova_trainer,
            decision_function<poly_kernel>,  // This is the output of the poly_trainer
            decision_function<rbf_kernel>
        > dfb;

    std::cout<<"setting svm_nu_trainer poly kernel Results:"<<std::endl;
    trainer_a.set_trainer(poly_trainer); // Works best
    std::cout << "cross validation: \n" << std::endl;
    matrix<double> result_param = cross_validate_multiclass_trainer(trainer_a, samples, labels, 5);
    std::cout<<result_param << endl;
    one_vs_all_decision_function<ova_trainer> df8 = trainer_a.train(samples, labels);
    dfb = df8;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;


    MEASURE("SVM NU POLY")

/*    RESET

	std::cout<<"setting svm_nu_trainer rbf poly kernel Results:"<<std::endl;
	trainer_a.set_trainer(nu_rbf_trainer);

    std::cout << "cross validation: \n" << std::endl;
    std::cout<<cross_validate_multiclass_trainer(trainer_a, samples, labels, 5) << endl;
    one_vs_all_decision_function<ova_trainer> df10 = trainer_a.train(samples, labels);
    dfb = df10;
    // Test df3 on the samples and labels and print the confusion matrix.
    cout << "test serialized function: \n" << test_multiclass_decision_function(dfb, samples, labels) << endl;

    MEASURE("SVM NU RBF")*/

    cout<<"saving params for best result......"<<std::endl;
    // Now check crossvalidate and check results
    double row_1_sum = 23;//result_param(0,0) + result_param(0,1) +result_param(0,2);
    double row_2_sum = 30;//result_param(1,0) + result_param(1,1) +result_param(1,2);
    double row_3_sum = 245;//result_param(2,0) + result_param(2,1) +result_param(2,2);

    cout<<"Number of class 1 "<<row_1_sum<<" Number of class 2 "<<row_2_sum<<" Number of class 3 "<<row_3_sum<<std::endl;
    // The nu parameter has a maximum value that is dependent on the ratio of the +1 to -1
    // labels in the training data.  This function finds that value.  The 0.999 is here because
    // the maximum allowable nu is strictly less than the value returned by maximum_nu().  So
    // rather than dealing with that below we can just back away from it a little bit here and then
    // not worry about it.
    std::cout<<"Setting max nu"<<std::endl;
    const double max_nu = 0.999*2;


    // The first kind of model selection we will do is a simple grid search.  That is, below we just
    // generate a fixed grid of points (each point represents one possible setting of the model parameters)
    // and test each via cross validation.

    // This code generates a 4x4 grid of logarithmically spaced points.  The result is a matrix
    // with 2 rows and 16 columns where each column represents one of our points.
    matrix<double> params = cartesian_product(logspace(log10(5.0), log10(1e-5), 4),  // gamma parameter
    		logspace(log10(max_nu), log10(1e-5), 4) // nu parameter
    );

    // Next we loop over all the points we generated and check how good each is.
    cout<<"-------------------------------------------------"<<endl;
    cout << "------Doing a grid search for ova poly kernel----" << endl;
    cout<<"-------------------------------------------------"<<endl;
    double best_result;
    best_result = 0;
    double final_result = 0;
    double best_gamma = 0.1, best_nu, best_poly = 1;
    for(int poly = 1; poly <= 4; poly++)
    {
    	for (long col = 0; col < params.nc(); ++col)
    	{
    		// pull out the current set of model parameters
    		const double gamma = params(0, col);
    		const double nu    = params(1, col);

    		// setup a training object using our current parameters
    		ova_trainer trainer_temp1;
    		svm_nu_trainer<poly_kernel> trainer_t;
    		trainer_t.set_kernel(poly_kernel(gamma,nu,poly));
    		trainer_temp1.set_trainer(trainer_t);

    		// Finally, do 10 fold cross validation and then check if the results are the best we have seen so far.
    		matrix<double> result = cross_validate_multiclass_trainer(trainer_temp1, samples, labels, 5);

    		// save the best results
    		double class_accuracy = (result(0,0)/row_1_sum + result(1,1)/row_2_sum + result(2,2)/row_3_sum)/3;
    		double new_result = result(0,0) + result(1,1);
    		cout << "gamma: " << setw(11) << gamma << "  nu: " << setw(11) << nu << " poly : "<<setw(11)<<poly<<  "  cross validation accuracy: "<<class_accuracy<<" action result:"<<  new_result<<" bad data result: "<< result(2,2)<<endl;
    		if (class_accuracy > best_result)
    		{
    			best_result = class_accuracy;
    			best_gamma = gamma;
    			best_nu = nu;
    			best_poly = poly;
    			final_result = new_result + result(2,2);
    		}
    	}
    }

    cout << "Best Parameters: gamma: " << setw(11) << best_gamma << "  nu: " << setw(11) << best_nu << " poly : "<<setw(11)<<best_poly<<  "  cross validation accuracy: " << final_result/labels.size() <<endl;

    // Next we loop over all the points we generated and check how good each is.
	cout<<"-------------------------------------------------"<<endl;
    cout<<"------Doing a grid search for ovo poly kernel----" << endl;
    cout<<"-------------------------------------------------"<<endl;

    best_result = 0;
    final_result = 0;
    for(int poly = 1; poly <= 4; poly++)
    {
    	for (long col = 0; col < params.nc(); ++col)
    	{
    		// pull out the current set of model parameters
    		const double gamma = params(0, col);
    		const double nu    = params(1, col);

    		// setup a training object using our current parameters
    		ovo_trainer trainer_temp2;
    		svm_nu_trainer<poly_kernel> trainer_t;
    		trainer_t.set_kernel(poly_kernel(gamma,nu,poly));
    		trainer_temp2.set_trainer(trainer_t);

    		// Finally, do 10 fold cross validation and then check if the results are the best we have seen so far.
    		matrix<double> result = cross_validate_multiclass_trainer(trainer_temp2, samples, labels, 5);

    		// save the best results
    		double class_accuracy = (result(0,0)/row_1_sum + result(1,1)/row_2_sum + result(2,2)/row_3_sum)/3;
    		double new_result = result(0,0) + result(1,1);
    		cout << "gamma: " << setw(11) << gamma << "  nu: " << setw(11) << nu << " poly : "<<setw(11)<<poly<<  "  cross validation accuracy: " <<class_accuracy<<" action result:"<<  new_result<<" bad data result: "<< result(2,2)<<endl;
    		if (class_accuracy > best_result)
    		{
    			best_result = class_accuracy;
    			best_gamma = gamma;
    			best_nu = nu;
    			best_poly = poly;
    			final_result = new_result + result(2,2);
    		}
    	}
    }

    cout << "Best Parameters: gamma: " << setw(11) << best_gamma << "  nu: " << setw(11) << best_nu << " poly : "<<setw(11)<<best_poly<<  "  cross validation accuracy: " << final_result/labels.size() <<endl;
    cout<< "Number of labels "<<labels.size()<<std::endl;



}

