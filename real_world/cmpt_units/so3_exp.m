function y = so3_exp(x)
    N = size(x,1);
        theta=norm(x);
        if theta==0
            y=eye(3);
        else     
            omega =x/theta;
            y=eye(3,3) + sin(theta) * skew(omega) + (1 - cos(theta))*skew(omega)^2;  
        end
end

