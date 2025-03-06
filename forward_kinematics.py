from sympy import *
import numpy as np

c1,s1,c2,s2,c3,s3,c4,s4,c5,s5,l0,l1,l2,l3 = symbols('c1 s1 c2 s2 c3 s3 c4 s4 c5 s5 l0 l1 l2 l3')
T1 = Matrix([[c1 ,-s1*np.cos(90), s1*np.sin(90), 0 ],
             [s1,c1*np.cos(90), -c1*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),l0 ], 
             [0,0,0,1]])

T2 = Matrix([[c2 ,-s2*np.cos(0), s2*np.sin(0), l1*c2 ],
             [s2,c2*np.cos(0), -c2*np.sin(0), l1*s2 ],
             [0,np.sin(0), np.cos(0),0 ], 
             [0,0,0,1]])

T3 = Matrix([[c3 ,-s3*np.cos(90), s3*np.sin(90), 0 ],
             [s3,c3*np.cos(90), -c3*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),0 ], 
             [0,0,0,1]])

T4 = Matrix([[ -s4 ,c4*np.cos(-90), c4*np.sin(-90), 0 ],
             [ c4,-s4*np.cos(-90), s4*np.sin(-90), 0 ],
             [0,np.sin(-90), np.cos(-90),l2 ], 
             [0,0,0,1]])

T5 = Matrix([[c5 ,-s5*np.cos(90), s5*np.sin(90), 0 ],
             [s5,c5*np.cos(90), -c5*np.sin(90), 0 ],
             [0,np.sin(90), np.cos(90),l3 ], 
             [0,0,0,1]])

T05 =   T1*T2*T3*T4*T5

print(f"Tx = {T05[3]} /n Ty = {T05[7]} /n Tz = {T05[11]}")

