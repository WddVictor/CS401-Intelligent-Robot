clear all;
close all;

K = 10000;


z_MAX = 5;

zt = linspace(0,z_MAX,K);

p = ones(1,K);

obstacle = 3;

for i = 1:K
    z_tk = zt(i);
    z_tk_star = obstacle;
    p(i) = beam_range_finder_model(z_tk,z_tk_star,z_MAX);

end

plot(zt,p/sum(p(:)));

function q = beam_range_finder_model(z_tk,z_tk_star,z_MAX)
sigma_hit= 0.4;
lambda_short=0.5;

z_hit= 0.1;
z_short= 0.05;
z_max=0.05;
z_rand=0.8;

q = 1;
p =z_hit*p_hit(z_tk,z_tk_star,sigma_hit,z_MAX)+ ...
    z_short*p_short(z_tk,z_tk_star,lambda_short)+...
    z_rand*p_max(z_tk,z_MAX)+...
    z_max*p_rand(z_tk,z_MAX);

q = q*p;
end


function p = p_hit(z_tk,z_tk_star,sigma_hit,z_MAX)
if(z_tk<=z_MAX&&z_tk>=0)
    eta = normcdf([0 z_MAX],z_tk_star,sigma_hit);
    eta = 1/(eta(2)-eta(1));
    p = eta*normpdf(z_tk,z_tk_star,sigma_hit);
else
    p = 0;
end
end

function p = p_short(z_tk,z_tk_star,lambda_short)
if(0<=z_tk&&z_tk<=z_tk_star)
    eta = 1/(1-exp(-lambda_short*z_tk_star));
    p = eta*exppdf(z_tk,1/lambda_short);
else
    p = 0;
end
end

function p = p_max(z_tk,z_MAX)
if(z_tk==z_MAX)
    p = 1;
else
    p = 0;
end
end

function p = p_rand(z_tk,z_MAX)
if(0<=z_tk&&z_tk<=z_MAX)
    p = 1/z_MAX;
else
    p = 0;
end
end

function result = sample_distribution(f,fmax,z_tk_star,sigma_hit,z_MAX)
z_tk = rand()*z_MAX;
p = rand()*fmax;

while p >= f(z_tk,z_tk_star,sigma_hit,z_MAX)
    z_tk = rand()*z_MAX;
    p = rand()*fmax;
end
result = z_tk;
end

