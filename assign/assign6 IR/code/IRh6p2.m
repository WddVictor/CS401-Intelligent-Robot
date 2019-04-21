clear all;
close all;

K = 100;
obstacle = cat(2,[1.5,2],linspace(3,4,1000));
z_MAX = 5;
zt = linspace(0,z_MAX,K);
[n,m] = size(obstacle);
final_p = zeros(1,K);
for i = 1:m
p = get_p(zt,z_MAX,obstacle(i),K);
final_p=final_p.*(final_p>=p)+p.*(final_p<p);
end
plot(zt,final_p/sum(final_p(:)));


function p = get_p(zt,z_MAX,obstacle,K)
p = ones(1,K);
for i = 1:K
    z_tk = zt(i);
    z_tk_star = obstacle;
    p(i) = beam_range_finder_model(z_tk,z_tk_star,z_MAX);
end
end

function q = beam_range_finder_model(z_tk,z_tk_star,z_MAX)
sigma_hit= 0.4;

z_hit= 0.8;
z_max=0.1;
z_rand=0.1;

q = 1;
p =z_hit*p_hit(z_tk,z_tk_star,sigma_hit,z_MAX)+ ...
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

