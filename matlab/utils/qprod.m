function qp = qprod(q1,q2)
    qp = [q1(1)*q2(1) - dot(q1(2:4),q2(2:4));q1(2:4).*q2(1)+q1(1).*q2(2:4)+hat(q1(2:4))*q2(2:4)];
end