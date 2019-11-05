from sympy import *

a = symbols('a')

b = 1
c = 1 
global m
m = a+b+c 
print(m)


def get_m(a1):
	
	return m.subs(a,a1)

if __name__== "__main__":
	a1 = 1
	m1 = get_m(a1)
	print(m1)



