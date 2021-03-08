function M = estimatePoseDLT(p,P,K)
p_k = [p;ones(1,12)];
p_h = K^(-1)*p_k;
P_h = [P,ones(12,1)]';
Q = [];
for i =1:12
    Ql = [kron(eye(2),P_h(:,i)')];
    Qr = -[p_h(1,i)*P_h(:,i)';p_h(2,i)*P_h(:,i)'];
    Q = [Q;[Ql,Qr]];
end
[~,~,V] = svd(Q);
M_vec = V(:,end);
M = reshape(M_vec,4,3)';
if M(3,4) < 0 
    M = -M;
end
R = M(1:3,1:3);
t = M(:,4);
[U,~,V] = svd(R);
R_t = U*V';
if norm(det(R_t)-1)>1e-5 ||  norm(R_t'*R_t - eye(3))>1e-5
    disp('Rotation Matrix error');
end
alpha = norm(R_t)/norm(R);
M = [R_t,alpha*t];

end