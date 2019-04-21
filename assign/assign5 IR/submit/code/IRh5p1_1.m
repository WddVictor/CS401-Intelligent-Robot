clear all;
close all;

X = zeros(1,1000000);

for i = 1:1000000
    X(i) = sample_normal_distribution(10);
end

histogram(X,'Normalization','probability');

function result = sample_normal_distribution(b)
result = sum(rand(1,12)*2*b-b)/12;
end