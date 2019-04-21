clear all;
close all;

X = zeros(1,1000000);

for i = 1:1000000
    X(i) = sample_triangular_distribution(4);
end

histogram(X,'Normalization','probability');

function result = sample_triangular_distribution(b)
result = 2*b*(rand()+rand())-2*b;
result = result * sqrt(6)/2;
end