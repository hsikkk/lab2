function p=find_rotation_matrix(u,up,vp)
% This finds a 3x3 rotation matrix p such that up=p*u and vp=p*v, where v=[0;0;1].
% 3x3 matrix is a 3-dimensional rotation matrix if row (and column) vectors have unit norm
% and are orthogonal to each other and the determinant is one.
% u: unit norm vector of size 3x1
% up,vp: unit norm vectors of size 3x1
% Written by Sae-Young Chung in 2012
% Last update: 2014/4/8

u=reshape(u,3,1);	% make it 3x1 (even if it was 1x3, it will become 3x1)
v=[0;0;1];
up=reshape(up,3,1);
vp=reshape(vp,3,1);
u=u/norm(u);		% if not already normalized, this will normalize it
up=up/norm(up);
vp=vp/norm(vp);

% The inner product (or equivalently the angle) between up and vp may be different
% from that between u and v due to some sensing noise.
% But, it doesn't matter for the algorithm below.

w=cross(u,v);
w=w/norm(w);
x=cross(u,w);
x=x/norm(x);
% now [u w x] is orthonormal

wp=cross(up,vp);
wp=wp/norm(wp);
xp=cross(up,wp);
xp=xp/norm(xp);
% now [up wp xp] is orthonormal

p=[up wp xp]*[u w x]';
