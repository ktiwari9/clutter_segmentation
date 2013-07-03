%   Copyright (c) 2013, University of Southern California
%   All rights reserved.
%  
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE.
%  
% 
%    \author Bharath Sankaran
%  
%   @b Temaplte Rank Learning
%==========================================================================

%==========================================================================
% Loading datasets
%==========================================================================

clc
clear all;
load training_data.mat;


% Running L-1 regularized logistic regression for each action

threshold_values = mean(rewards_matrix,2);

index_assignment = rewards_matrix > repmat(threshold_values,1,size(rewards_matrix,2));

[i_positive,j_positive] = find(index_assignment == 1); 
[i_negative,j_negative] = find(index_assignment == 0); 

weights = [];

for i = 1: size(index_assignment,1)
    
    
    X_features = [];
    X_labels = [];
    %pos_indices = sub2ind(size(index_assignment),i_positive(i_positive == i),j_positive(i_positive == i));
    %neg_indices = sub2ind(size(index_assignment),i_negative(i_positive == i),j_negative(i_positive == i));
    
    index_i  = i_positive(i_positive == i);
    index_j  = j_positive(i_positive == i);
    
    X_positive = zeros(length(index_i),size(features_matrix,3));
    for j = 1:length(index_i)        
        X_positive(j,:)  = features_matrix(index_i(j),index_j(j),:);        
    end
    
    pos_labels  = ones(size(X_positive,1),1);
        
    
    index_i  = i_negative(i_negative == i);
    index_j  = j_negative(i_negative == i);
    
    X_negative = zeros(length(index_i),size(features_matrix,3));
    for j = 1:length(index_i)        
        X_negative(j,:)  = features_matrix(index_i(j),index_j(j),:);        
    end
    
    neg_labels  = -1*ones(size(X_negative,1),1);
    
    X_features = [X_positive;X_negative];
    X_labels = [pos_labels;neg_labels];
    %mmwrite('ex_X',X_features);
    %mmwrite('ex_b',X_labels);
    
    %system('l1_logreg_train -s ex_X ex_b 0.01 model_iono');
    
    %model_iono = mmread('model_iono');
    
    % Least Squares Solution
    wLS = X_features\X_labels;
    
    % Ridge Regression
    lambda = 100*ones(size(X_features,2),1); % Penalize each element by the same amount
    R = chol(X_features'*X_features + diag(lambda));
    wRR = R\(R'\(X_features'*X_labels));
    
    % LASSO
    lambda = 100*ones(size(X_features,2),1); % Penalize the absolute value of each element by the same amount
    funObj = @(w)SquaredError(w,X_features,X_labels); % Loss function that L1 regularization is applied to
    w_init = wRR; % Initial value for iterative optimizer
    fprintf('\nComputing LASSO Coefficients...\n');
    wLASSO = L1General2_PSSgb(funObj,w_init,lambda);
    
    %L-1 regularized logistic regression
    
    % adding a bias element to the features
    X_new_features = [ones(size(X_features,1),1) X_features];
    
    funObj = @(w)LogisticLoss(w,X_new_features,X_labels);
    w_init = zeros(size(X_features,2)+1,1);
    
    
    % Maximum Likelihood
    fprintf('\nComputing Maximum Likelihood Logistic Regression Coefficients\n');
    mfOptions.Method = 'newton';
    wLogML = minFunc(funObj,w_init,mfOptions);

    % L2-Regularized Logistic Regression
    fprintf('\nComputing L2-Regularized Logistic Regression Coefficients...\n');
    lambda = 15*ones(size(X_features,2)+1,1);
    lambda(1) = 0; % Do not penalize bias variable
    funObjL2 = @(w)penalizedL2(w,funObj,lambda);
    wLogL2 = minFunc(funObjL2,w_init,mfOptions);

    % L1-Regularized Logistic Regression
    fprintf('\nComputing L1-Regularized Logistic Regression Coefficients...\n');
    wLogL1 = L1General2_PSSgb(funObj,w_init,lambda);
    
    
    fprintf('Number of Features Selected by Maximum Likelihood Logistic Regression classifier: %d (out of %d)\n',nnz(wLogML(2:end)),size(X_features,2));
    fprintf('Number of Features Selected by L2-regualrized Logistic Regression classifier: %d (out of %d)\n',nnz(wLogL2(2:end)),size(X_features,2));
    fprintf('Number of Features Selected by L1-regualrized Logistic Regression classifier: %d (out of %d)\n',nnz(wLogL1(2:end)),size(X_features,2));
    fprintf('Number of non-zero variables in Least Squares solution: %d\n',nnz(wLS));
    fprintf('Number of non-zero variables in Ridge Regression solution: %d\n',nnz(wRR));
    fprintf('Number of non-zero variables in LASSO solution: %d\n',nnz(wLASSO));
    
    figure;
    clf;hold on;
    subplot(2,3,1);
    stem(wLS,'r');
    xlim([1 size(X_features,2)]);
    yl = ylim;
    title('Least Squares');
    subplot(2,3,2);
    stem(wRR,'b');
    xlim([1 size(X_features,2)]);
    ylim(yl);
    title('Ridge Regression');
    subplot(2,3,3);
    stem(wLASSO,'g');
    xlim([1 size(X_features,2)]);
    title('LASSO');
    ylim(yl);
    
    subplot(2,3,4);
    stem(wLogML,'r');
    xlim([1 size(X_features,2)+1]);
    title('Maximum Likelihood Logistic Regression');
    subplot(2,3,5);
    stem(wLogL2,'b');
    xlim([1 size(X_features,2)+1]);
    title('L2-Regularized Logistic Regression');
    subplot(2,3,6);
    stem(wLogL1,'g');
    xlim([1 size(X_features,2)+1]);
    title('L1-Regularized Logistic Regression');
    %pause;
    
    
    filename = strcat('action_a_',num2str(i),'.mat');
    save(filename,'wLogL1');
    
end




