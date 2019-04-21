clear all;
close all;

num = 1000000;
X = zeros(1,num);

f = @func;
b = 1;

fmax = max(f(-b:b));

for i = 1:num
    X(i) = sample_distribution(f,fmax,b);
end

histogram(X,'Normalization','probability');

function y = func(x)
if((x>=-1) & (x<=1))
    y = abs(x);
else
    y = 0;
end
end

function result = sample_distribution(f,fmax,b)
x = rand()*2*b-b;
y = rand()*fmax;

while y >= f(x)
    x = rand()*2*b-b;
    y = rand()*fmax;
end
result = x;
end