from sympy import *
import numpy as np

t1,t2,t3,t4,t5,l0,l1,l2,l3,l4,l5 = symbols('t1 t2 t3 t4 t5 l0 l1 l2 l3 l4 l5')
T1 = Matrix([[cos(t1) ,-sin(t1)*np.cos(90), sin(t1)*np.sin(90), 0 ],
             [sin(t1),cos(t1)*np.cos(90), -cos(t1)*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),l0 ], 
             [0,0,0,1]])

T2 = Matrix([[cos(t2) ,-sin(t2)*np.cos(0), sin(t2)*np.sin(0), l2*cos(t2) ],
             [sin(t2),cos(t2)*np.cos(0), -cos(t2)*np.sin(0), l2*sin(t2) ],
             [0,np.sin(0), np.cos(0),(l3-l1) ], 
             [0,0,0,1]])

T3 = Matrix([[-sin(t3) ,-sin(t3)*np.cos(90), sin(t3)*np.sin(90), 0 ],
             [sin(t3),-sin(t3)*np.cos(90), sin(t3)*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),0 ], 
             [0,0,0,1]])

T4 = Matrix([[ cos(t4) ,-cos(t4)*np.cos(-90), sin(t4)*np.sin(-90), 0 ],
             [ sin(t4),cos(t4)*np.cos(-90), -cos(t4)*np.sin(-90), 0 ],
             [0,np.sin(-90), np.cos(-90),l4 ], 
             [0,0,0,1]])

T5 = Matrix([[cos(t5) ,-sin(t5)*np.cos(90), sin(t5)*np.sin(90), 0 ],
             [sin(t5),cos(t5)*np.cos(90), -cos(t5)*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),-l5 ], 
             [0,0,0,1]])

T05 =   T1*T2*T3*T4*T5

print(f"Tx = {T05[3]} /n Ty = {T05[7]} /n Tz = {T05[11]}")

Tx, Ty,Tz = T05[3], T05[7], T05[11]


Tx_1 = diff(Tx, t1 )
Tx_2 = diff(Tx , t2)
Tx_3 = diff(Tx , t3)
Tx_4 = diff(Tx , t4)
Tx_5 = diff(Tx , t5)

print(f"Tx1 = {Tx_1} /n Tx2 = {Tx_2} /n Tx3 = {Tx_3} /n Tx4 = {Tx_4} /n Tx5 = {Tx_5}")
print("--------------------------------------------------------------------------------------------------")

Ty_1 = diff(Ty, t1 )
Ty_2 = diff(Ty , t2)
Ty_3 = diff(Ty , t3)
Ty_4 = diff(Ty , t4)
Ty_5 = diff(Ty , t5)

print(f"Ty1 = {Ty_1} /n Ty2 = {Ty_2} /n Ty3 = {Ty_3} /n Ty4 = {Ty_4} /n Ty5 = {Ty_5}")
print("--------------------------------------------------------------------------------------------------")


Tz_1 = diff(Tz, t1 )
Tz_2 = diff(Tz , t2)
Tz_3 = diff(Tz , t3)
Tz_4 = diff(Tz , t4)
Tz_5 = diff(Tz , t5)

print(f"Tz1 = {Tz_1} /n Tz2 = {Tz_2} /n Tz3 = {Tz_3} /n Tz4 = {Tz_4} /n Tz5 = {Tz_5}")
print("--------------------------------------------------------------------------------------------------")

