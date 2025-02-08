function J = jacobiano(Q)
global L1 L2 L3
c1 = cos(Q(1));
s1 = sin(Q(1));
c2 = cos(Q(2));
s2 = sin(Q(2));
c3 = cos(Q(3));
s3 = sin(Q(3));

c12 = cos(Q(1) + Q(2));
c23 = cos(Q(2) + Q(3));
s12 = sin(Q(1) + Q(2));
s23 = sin(Q(2) + Q(3));

J = [-s1*(L2*c2 + L3*c23) -c1*(L2*s2 + L3*s23) -L3*c1*s23;
      c1*(L2*c2 + L3*c23) -s1*(L2*s2 + L3*s23) -L3*s1*s23;
      0                   L2*c2 + L3*c23     L3*c23];
end


