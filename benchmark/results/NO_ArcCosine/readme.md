Computing Momentum requires arccosine, but given that the cost function is:

Cost = 1/2 + ArcCos(angle\_dif) / pi
Cost = ArcSin(angle\_diff) / pi

Taylor series:

ArcSin(x) = ax + bx^3 + cx^5 + ...
ArcSin(x) = x + (1/6) x^3 + (3/40) x^5 + ...

Thus, taylor up to the third element:
Cost(x) = (1/pi) * (x + 1/6x^3 + 3/40x^5)

This creates an error in the tails, meaning, around -1, 1, to over come this issue, we know that arount that tail the cost has to be +/-0.5

Cost(1) = (1/pi) (1) + 1/6(1/pi) + c(1^5) = 0.5

we picked c, because it is the tiny adjustment, we need.

c = 0.128638
