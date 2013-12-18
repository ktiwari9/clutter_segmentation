
load('example-A.dat');



p = 0.7;
k = 1;
p_a = (p^k)*((1-p)^(1-k));

beta_function = @(alpha_v,beta_v)factorial(alpha_v - 1).*factorial(beta_v - 1)./factorial(alpha_v+beta_v-1);
beta_prior = @(x,alpha_v,beta_v)(1./beta_function(alpha_v,beta_v)).*(x.^(alpha_v-1)).*((1-x).^(beta_v - 1));

alpha_val = [4,3,2,1,2,3,4];
beta_val = [1,2,3,4,3,2,1];
x = repmat(0.5,1,length(alpha_val));
value = beta_prior(x,alpha_val,beta_val);
adj = example_A;


p = 0.01:0.01:0.99;

entropy = -(1-p).*log((1-p))-p.*log(p);